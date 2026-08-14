// Live variant of run_video_msckf.cpp: feeds OpenVINS from live ROS2 topics
// (mavros IMU + a camera image topic) instead of imu.csv/video.mkv, and
// publishes each state as nav_msgs/Odometry on /vio/odom instead of writing
// a CSV row per frame -- a CSV is still written alongside, for post-flight
// debugging, with the same columns the offline runner uses.
//
// The IMU lookahead is not optional here either: the propagator interpolates
// between the IMU samples bracketing each camera instant, so a frame must be
// held until IMU has been received PAST it -- see the offline runner's
// comment on this. Live, that is a real ~100ms latency added to every
// published pose, implemented below by making the processing thread wait on
// the IMU deque rather than draining whatever happens to be there yet.
//
// Threading: IMU and image callbacks each just append to a small
// mutex-guarded queue and return immediately (staying off the ROS executor
// thread matters here, since blocking a subscription callback would stall
// the other one too). A dedicated processing thread drains queued frames,
// each time waiting for IMU to catch up before feeding VioManager -- this is
// the live equivalent of the offline runner's single serial loop.
//
//   run_video_msckf_live <config.yaml>
//     [--ros-args -p imu_topic:=/mavros/imu/data_raw
//                 -p image_topic:=/drone/camera/image_raw
//                 -p odom_topic:=/vio/odom
//                 -p csv_path:=vio_vpe/logs/vio_live_<ts>.csv]

#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;
using namespace std::chrono_literals;

namespace {
// The offline runner's kImuLookahead, unchanged -- see run_video_msckf.cpp.
constexpr double kImuLookahead = 0.10;
// How long the processing thread will wait for IMU to catch up before giving
// up on a frame and feeding it anyway. This should never trigger in normal
// operation (IMU arrives at ~200-340 Hz); it exists so a stalled IMU topic
// degrades to a loud warning instead of an unbounded stall of VIO output.
constexpr double kMaxImuWaitS = 1.0;
} // namespace

class VioLiveNode : public rclcpp::Node {
public:
  explicit VioLiveNode(const std::string &config_path) : rclcpp::Node("vio_live_node") {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data_raw");
    image_topic_ = declare_parameter<std::string>("image_topic", "/drone/camera/image_raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/vio/odom");
    std::string default_csv = "vio_vpe/logs/vio_live_" + std::to_string(this->now().seconds()) + ".csv";
    csv_path_ = declare_parameter<std::string>("csv_path", default_csv);

    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    track_hz_ = 15.0;
    parser->parse_config("track_frequency", track_hz_);

    VioManagerOptions params;
    params.print_and_load(parser);
    params.use_multi_threading_subs = false;
    sys_ = std::make_shared<VioManager>(params);
    if (!parser->successful()) {
      throw std::runtime_error("unable to parse all OpenVINS parameters");
    }
    cfg_w_ = params.camera_intrinsics.at(0)->w();
    cfg_h_ = params.camera_intrinsics.at(0)->h();

    csv_.open(csv_path_);
    csv_ << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
    csv_.precision(9);
    csv_ << std::fixed;

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, sensor_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); });
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });

    PRINT_INFO(GREEN "[vio_live] cfg %dx%d, track_hz=%.1f, imu=%s image=%s -> %s\n" RESET,
               cfg_w_, cfg_h_, track_hz_, imu_topic_.c_str(), image_topic_.c_str(),
               odom_topic_.c_str());

    shutdown_ = false;
    proc_thread_ = std::thread([this] { process_loop(); });
  }

  ~VioLiveNode() override {
    {
      std::lock_guard<std::mutex> lk(frame_mtx_);
      shutdown_ = true;
    }
    frame_cv_.notify_all();
    imu_cv_.notify_all();
    if (proc_thread_.joinable())
      proc_thread_.join();
    csv_.close();
  }

private:
  // -- callbacks: must stay cheap, no VioManager calls here ----------------
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    ov_core::ImuData m;
    m.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    m.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    m.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    {
      std::lock_guard<std::mutex> lk(imu_mtx_);
      imu_q_.push_back(m);
    }
    imu_cv_.notify_all();
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    double tf = rclcpp::Time(msg->header.stamp).seconds();
    // Subsample the live feed (likely 30 fps) down to track_frequency,
    // mirroring the offline runner's --frame-stride.
    if (last_queued_t_ > 0.0 && tf - last_queued_t_ < (1.0 / track_hz_) - 1e-3)
      return;
    last_queued_t_ = tf;

    if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "unexpected image encoding '%s', expected rgb8/bgr8",
                           msg->encoding.c_str());
      return;
    }
    cv::Mat color(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()),
                  msg->step);
    cv::Mat gray;
    cv::cvtColor(color, gray,
                 msg->encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
    if (gray.cols != cfg_w_ || gray.rows != cfg_h_)
      cv::resize(gray, gray, cv::Size(cfg_w_, cfg_h_), 0, 0, cv::INTER_AREA);

    {
      std::lock_guard<std::mutex> lk(frame_mtx_);
      frame_q_.emplace_back(tf, gray.clone());
    }
    frame_cv_.notify_all();
  }

  // -- processing thread: the only thread that touches sys_ ----------------
  void process_loop() {
    while (rclcpp::ok()) {
      double tf;
      cv::Mat gray;
      {
        std::unique_lock<std::mutex> lk(frame_mtx_);
        frame_cv_.wait(lk, [this] { return shutdown_ || !frame_q_.empty(); });
        if (shutdown_)
          return;
        std::tie(tf, gray) = frame_q_.front();
        frame_q_.pop_front();
      }

      // Wait for IMU to have data past tf + kImuLookahead (see file header).
      {
        std::unique_lock<std::mutex> lk(imu_mtx_);
        bool have_lookahead = imu_cv_.wait_for(lk, std::chrono::duration<double>(kMaxImuWaitS),
                                                [this, tf] {
                                                  return shutdown_ || (!imu_q_.empty() &&
                                                         imu_q_.back().timestamp > tf + kImuLookahead);
                                                });
        if (shutdown_)
          return;
        if (!have_lookahead) {
          PRINT_WARNING(YELLOW "[vio_live] IMU stalled: no sample past t+%.2fs after "
                        "%.1fs wait -- feeding camera anyway, expect dead reckoning "
                        "until IMU resumes\n" RESET, kImuLookahead, kMaxImuWaitS);
        }
        while (!imu_q_.empty() && imu_q_.front().timestamp <= tf + kImuLookahead) {
          ov_core::ImuData m = imu_q_.front();
          imu_q_.pop_front();
          lk.unlock();
          sys_->feed_measurement_imu(m);
          lk.lock();
        }
      }

      ov_core::CameraData cam;
      cam.timestamp = tf;
      cam.sensor_ids.push_back(0);
      cam.images.push_back(gray);
      cam.masks.push_back(cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1));
      sys_->feed_measurement_camera(cam);
      n_fed_++;

      if (!sys_->initialized()) {
        if (n_fed_ % 100 == 0)
          PRINT_INFO("[vio_live] not yet initialised, fed=%zu\n", n_fed_);
        continue;
      }
      publish_and_log(tf);
    }
  }

  void publish_and_log(double tf) {
    auto state = sys_->get_state();
    Eigen::Matrix<double, 4, 1> q = state->_imu->quat();
    Eigen::Matrix<double, 3, 1> p = state->_imu->pos();
    Eigen::Matrix<double, 3, 1> v = state->_imu->vel();
    Eigen::Matrix<double, 3, 1> bg = state->_imu->bias_g();
    Eigen::Matrix<double, 3, 1> ba = state->_imu->bias_a();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(static_cast<int64_t>(state->_timestamp * 1e9));
    // Frame name is deliberately not "odom"/"map": position/velocity here are
    // in OpenVINS' own gravity-aligned, YAW-ARBITRARY world frame. Anything
    // downstream (fusion_live_node) must resolve that yaw against VPE before
    // treating this as ENU.
    odom.header.frame_id = "vio_yaw_arbitrary";
    odom.pose.pose.position.x = p(0);
    odom.pose.pose.position.y = p(1);
    odom.pose.pose.position.z = p(2);
    odom.pose.pose.orientation.x = q(0);
    odom.pose.pose.orientation.y = q(1);
    odom.pose.pose.orientation.z = q(2);
    odom.pose.pose.orientation.w = q(3);
    odom.twist.twist.linear.x = v(0);
    odom.twist.twist.linear.y = v(1);
    odom.twist.twist.linear.z = v(2);
    pub_odom_->publish(odom);

    csv_ << state->_timestamp << "," << p(0) << "," << p(1) << "," << p(2) << "," << q(0) << ","
         << q(1) << "," << q(2) << "," << q(3) << "," << v(0) << "," << v(1) << "," << v(2) << ","
         << bg(0) << "," << bg(1) << "," << bg(2) << "," << ba(0) << "," << ba(1) << "," << ba(2)
         << "\n";
    if (n_fed_ % 200 == 0) {
      csv_.flush();
      PRINT_INFO("[vio_live] t=%.1f fed=%zu |p|=%.1f m\n", tf, n_fed_, p.norm());
    }
  }

  // config / topics
  std::string imu_topic_, image_topic_, odom_topic_, csv_path_;
  double track_hz_ = 15.0;
  int cfg_w_ = 0, cfg_h_ = 0;

  // OpenVINS
  std::shared_ptr<VioManager> sys_;
  size_t n_fed_ = 0;
  std::ofstream csv_;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;

  // queues shared with the processing thread
  std::mutex imu_mtx_, frame_mtx_;
  std::condition_variable imu_cv_, frame_cv_;
  std::deque<ov_core::ImuData> imu_q_;
  std::deque<std::pair<double, cv::Mat>> frame_q_;
  double last_queued_t_ = -1.0;
  bool shutdown_ = false;
  std::thread proc_thread_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 2) {
    std::cout << "usage: run_video_msckf_live <config.yaml> [--ros-args -p imu_topic:=... "
                 "-p image_topic:=... -p odom_topic:=... -p csv_path:=...]\n";
    return 1;
  }
  try {
    auto node = std::make_shared<VioLiveNode>(args[1]);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[vio_live] fatal: %s\n" RESET, e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

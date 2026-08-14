// Offline OpenVINS runner for this project's survey recordings.
//
// The flight logger writes video.mkv + frame_times.csv + imu.csv rather than a
// rosbag, so this feeds those straight into VioManager with no ROS at all and
// dumps the state to CSV. Interleaving is strictly by timestamp: every IMU
// sample before a frame's time is fed first, so the filter sees the same
// ordering it would live.
//
//   run_video_msckf <config.yaml> <video.mkv> <frame_times.csv> <imu.csv> \
//                   <out.csv> [start_off_s] [end_off_s] [frame_stride]
//
// start/end offsets are seconds relative to the first video frame.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

namespace {

struct ImuRow {
  double t;
  double wx, wy, wz, ax, ay, az;
};

std::vector<std::string> split(const std::string &s, char d) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream ss(s);
  while (std::getline(ss, cur, d))
    out.push_back(cur);
  return out;
}

std::vector<ImuRow> load_imu(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    PRINT_ERROR(RED "cannot open imu csv %s\n" RESET, path.c_str());
    std::exit(EXIT_FAILURE);
  }
  std::string line;
  std::getline(f, line); // header: stamp_ros,recv_unix,wx,wy,wz,ax,ay,az
  std::vector<ImuRow> rows;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;
    auto c = split(line, ',');
    if (c.size() < 8)
      continue;
    ImuRow r;
    r.t = std::stod(c[0]);
    r.wx = std::stod(c[2]); r.wy = std::stod(c[3]); r.wz = std::stod(c[4]);
    r.ax = std::stod(c[5]); r.ay = std::stod(c[6]); r.az = std::stod(c[7]);
    rows.push_back(r);
  }
  return rows;
}

std::vector<double> load_frame_times(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    PRINT_ERROR(RED "cannot open frame_times csv %s\n" RESET, path.c_str());
    std::exit(EXIT_FAILURE);
  }
  std::string line;
  std::getline(f, line); // header: frame_idx,unix_time
  std::vector<double> t;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;
    auto c = split(line, ',');
    if (c.size() < 2)
      continue;
    t.push_back(std::stod(c[1]));
  }
  return t;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 6) {
    std::cout << "usage: run_video_msckf <config.yaml> <video.mkv> "
                 "<frame_times.csv> <imu.csv> <out.csv> "
                 "[start_off_s] [end_off_s] [frame_stride]\n";
    return 1;
  }
  std::string config_path = argv[1];
  std::string video_path = argv[2];
  std::string ftimes_path = argv[3];
  std::string imu_path = argv[4];
  std::string out_path = argv[5];
  double start_off = (argc > 6) ? std::atof(argv[6]) : 0.0;
  double end_off = (argc > 7) ? std::atof(argv[7]) : -1.0;
  int stride = (argc > 8) ? std::atoi(argv[8]) : 1;
  if (stride < 1)
    stride = 1;

  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = false;
  auto sys = std::make_shared<VioManager>(params);
  if (!parser->successful()) {
    PRINT_ERROR(RED "unable to parse all parameters\n" RESET);
    return 1;
  }

  auto imu = load_imu(imu_path);
  auto ftimes = load_frame_times(ftimes_path);
  if (imu.empty() || ftimes.empty()) {
    PRINT_ERROR(RED "empty imu (%zu) or frame times (%zu)\n" RESET, imu.size(),
                ftimes.size());
    return 1;
  }
  double t_video0 = ftimes.front();
  double t_start = t_video0 + start_off;
  double t_end = (end_off > 0) ? t_video0 + end_off : ftimes.back() + 1.0;
  PRINT_INFO("[offline] %zu imu samples (%.1f Hz), %zu frames\n", imu.size(),
             imu.size() / (imu.back().t - imu.front().t), ftimes.size());
  PRINT_INFO("[offline] window %.1f..%.1f s of video, stride %d\n", start_off,
             (end_off > 0 ? end_off : ftimes.back() - t_video0), stride);

  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    PRINT_ERROR(RED "cannot open video %s\n" RESET, video_path.c_str());
    return 1;
  }

  std::ofstream out(out_path);
  out << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
  out.precision(9);
  out << std::fixed;

  // Camera images are resized to whatever the config declares, so a config
  // written for a downscaled stream can be used without touching the video.
  int cfg_w = params.camera_intrinsics.at(0)->w();
  int cfg_h = params.camera_intrinsics.at(0)->h();

  size_t imu_i = 0;
  size_t n_fed = 0, n_rows = 0;
  cv::Mat frame, gray;
  for (size_t fi = 0; fi < ftimes.size(); fi++) {
    double tf = ftimes[fi];
    bool want = (tf >= t_start && tf <= t_end && (fi % (size_t)stride == 0));
    // The decoder has no random access worth using here, so every frame is
    // read; only the wanted ones are converted and fed.
    if (!cap.read(frame))
      break;
    if (tf > t_end)
      break;
    if (!want)
      continue;

    // Feed IMU up to a little PAST the image time, not up to it. The
    // propagator interpolates between the samples bracketing the camera
    // instant, so stopping exactly at tf leaves it with no trailing sample and
    // it bails out with "Missing inertial measurements to propagate with" --
    // silently, on every single frame, leaving the filter on dead reckoning.
    const double kImuLookahead = 0.10;
    while (imu_i < imu.size() && imu[imu_i].t <= tf + kImuLookahead) {
      const auto &r = imu[imu_i];
      if (r.t >= t_start - 1.0) {
        ov_core::ImuData m;
        m.timestamp = r.t;
        m.wm << r.wx, r.wy, r.wz;
        m.am << r.ax, r.ay, r.az;
        sys->feed_measurement_imu(m);
      }
      imu_i++;
    }

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    if (gray.cols != cfg_w || gray.rows != cfg_h)
      cv::resize(gray, gray, cv::Size(cfg_w, cfg_h), 0, 0, cv::INTER_AREA);

    ov_core::CameraData cam;
    cam.timestamp = tf;
    cam.sensor_ids.push_back(0);
    cam.images.push_back(gray.clone());
    cam.masks.push_back(cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1));
    sys->feed_measurement_camera(cam);
    n_fed++;

    if (!sys->initialized())
      continue;
    auto state = sys->get_state();
    Eigen::Matrix<double, 4, 1> q = state->_imu->quat();
    Eigen::Matrix<double, 3, 1> p = state->_imu->pos();
    Eigen::Matrix<double, 3, 1> v = state->_imu->vel();
    Eigen::Matrix<double, 3, 1> bg = state->_imu->bias_g();
    Eigen::Matrix<double, 3, 1> ba = state->_imu->bias_a();
    out << state->_timestamp << "," << p(0) << "," << p(1) << "," << p(2) << ","
        << q(0) << "," << q(1) << "," << q(2) << "," << q(3) << "," << v(0)
        << "," << v(1) << "," << v(2) << "," << bg(0) << "," << bg(1) << ","
        << bg(2) << "," << ba(0) << "," << ba(1) << "," << ba(2) << "\n";
    n_rows++;

    if (n_fed % 200 == 0) {
      PRINT_INFO("[offline] t=%.1fs fed=%zu rows=%zu |p|=%.1f m\n",
                 tf - t_video0, n_fed, n_rows, p.norm());
      out.flush();
    }
  }
  out.close();
  PRINT_INFO(GREEN "[offline] fed %zu images, wrote %zu states to %s\n" RESET,
             n_fed, n_rows, out_path.c_str());
  if (n_rows == 0)
    PRINT_ERROR(RED "[offline] filter never initialised\n" RESET);
  return n_rows > 0 ? 0 : 2;
}

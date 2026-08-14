cmake_minimum_required(VERSION 3.16)
project(vio_vpe_msckf CXX)

# Links straight against the OpenVINS build already sitting at OV_WS on this
# Jetson (~/openvins_ws) instead of expecting a colcon "install/" tree -- that
# tree does not exist here. ~/openvins_ws/build-ov was configured directly
# from ov_msckf's own CMakeLists (ov_msckf_SOURCE_DIR in its CMakeCache.txt
# points at src/open_vins/ov_msckf), which is why this project's include paths
# below mirror flags.make from that build: headers live under each package's
# src/ tree, not include/, and there is a single libov_msckf_lib.so with
# ov_core/ov_init statically folded in -- no separate libov_core_lib.so or
# libov_init_lib.so to link.
#
# That build has ENABLE_ROS=OFF (confirmed via `readelf -d` on
# libov_msckf_lib.so: only Boost/OpenCV/CUDA/libc, no rclcpp/tf2 DT_NEEDED
# entries) -- unlike the desktop x86 build the original deploy README
# describes. So unlike that README's warning, this project's own ROS2
# dependencies (added below, for the live node's topics) do NOT have to
# reconcile with anything OpenVINS itself pulled in.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OV_WS "$ENV{HOME}/openvins_ws" CACHE PATH
    "openvins_ws root holding src/open_vins and build-ov")
set(OV_SRC "${OV_WS}/src/open_vins")
set(OV_BUILD "${OV_WS}/build-ov")

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

set(OV_INCLUDE_DIRS
  ${OV_SRC}/ov_msckf/src
  ${OV_SRC}/ov_core/src
  ${OV_SRC}/ov_init/src
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
set(OV_LIBS
  ${OV_BUILD}/libov_msckf_lib.so
  ${OpenCV_LIBS}
  Boost::filesystem
  Boost::system
)
# cublas/cusolver (used by ov_init's MLE, compiled in even though init_dyn_use
# is off in our config) -- same -L this build's own link.txt uses.
set(OV_LINK_DIRS /usr/local/cuda-12.6/lib64)

# --- offline runner (unchanged from instructions/jetson_deploy/vio/) --------
# Kept for regression testing: replaying a recorded flight through this
# binary must keep matching session_2026-08-12_imx900_vio.md's numbers, so it
# stays available as a check against the live node's behavior.
add_executable(run_video_msckf run_video_msckf.cpp)
target_include_directories(run_video_msckf PRIVATE ${OV_INCLUDE_DIRS})
target_link_directories(run_video_msckf PRIVATE ${OV_LINK_DIRS})
target_link_libraries(run_video_msckf ${OV_LIBS})
set_target_properties(run_video_msckf PROPERTIES
  BUILD_RPATH "${OV_BUILD}"
  INSTALL_RPATH "${OV_BUILD}"
)

# --- live node (Phase 1) -----------------------------------------------------
find_package(ament_cmake QUIET)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(run_video_msckf_live run_video_msckf_live.cpp)
target_include_directories(run_video_msckf_live PRIVATE ${OV_INCLUDE_DIRS})
target_link_directories(run_video_msckf_live PRIVATE ${OV_LINK_DIRS})
target_link_libraries(run_video_msckf_live ${OV_LIBS} rclcpp::rclcpp)
ament_target_dependencies(run_video_msckf_live rclcpp sensor_msgs nav_msgs)
set_target_properties(run_video_msckf_live PROPERTIES
  BUILD_RPATH "${OV_BUILD}"
  INSTALL_RPATH "${OV_BUILD}"
)

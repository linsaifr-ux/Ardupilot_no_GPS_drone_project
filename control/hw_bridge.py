#!/usr/bin/env python3
"""
Hardware bridge: publishes /drone/state, /drone/pose, /drone/agl from MAVROS.

Converts EKF ENU local position (from /mavros/local_position/pose) to:
  /drone/state   PoseStamped  position=(East_m, North_m, alt_msl_m)  ← ardupilot_commander
  /drone/pose    PoseStamped  position=(lat, lon, alt_msl_m)          ← anyloc, yolo nodes
  /drone/agl     Float64      metres AGL above home                   ← anyloc, yolo nodes

Run: python3 control/hw_bridge.py
"""
import json
import math
import os
import sys

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64

_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_elevation.json")
with open(_HOME_CFG) as _f:
    _h = json.load(_f)
HOME_LAT     = float(_h["lat"])
HOME_LON     = float(_h["lon"])
HOME_ALT_MSL = float(_h["centre_elev_m"])
COS_LAT      = math.cos(math.radians(HOME_LAT))
M_PER_DEG    = 111_320.0

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=10)


class HWBridge(rclpy.node.Node):
    def __init__(self):
        super().__init__("hw_bridge")
        self._agl = 0.0

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, _SENSOR_QOS)
        self.create_subscription(Altitude, "/mavros/altitude",
                                 self._cb_alt, _SENSOR_QOS)

        self._pub_state = self.create_publisher(PoseStamped, "/drone/state", 10)
        self._pub_pose  = self.create_publisher(PoseStamped, "/drone/pose",  10)
        self._pub_agl   = self.create_publisher(Float64,     "/drone/agl",   10)

        self.get_logger().info(
            f"HW bridge ready  HOME={HOME_LAT:.5f},{HOME_LON:.5f}  MSL={HOME_ALT_MSL:.1f} m")

    def _cb_alt(self, msg):
        # msg.relative = AGL above home from ArduPilot baro
        if msg.relative > -100:
            self._agl = float(msg.relative)

    def _cb_pose(self, msg):
        # EKF ENU: x=East, y=North, z=Up from home origin
        east_m  = msg.pose.position.x
        north_m = msg.pose.position.y
        up_m    = msg.pose.position.z   # AGL from home

        alt_msl = HOME_ALT_MSL + up_m
        lat     = HOME_LAT + north_m / M_PER_DEG
        lon     = HOME_LON + east_m  / (M_PER_DEG * COS_LAT)

        stamp = msg.header.stamp

        # /drone/state — ENU metres (used by ardupilot_commander.py)
        s = PoseStamped()
        s.header.stamp = stamp; s.header.frame_id = "map"
        s.pose.position.x = east_m
        s.pose.position.y = north_m
        s.pose.position.z = alt_msl
        s.pose.orientation = msg.pose.orientation
        self._pub_state.publish(s)

        # /drone/pose — WGS84 (used by anyloc and yolo nodes)
        p = PoseStamped()
        p.header.stamp = stamp; p.header.frame_id = "wgs84"
        p.pose.position.x = lat
        p.pose.position.y = lon
        p.pose.position.z = alt_msl
        p.pose.orientation = msg.pose.orientation
        self._pub_pose.publish(p)

        # /drone/agl — prefer barometer (smoother) over EKF z
        agl = self._agl if abs(self._agl) < 500 else up_m
        a = Float64(); a.data = float(max(0.0, agl))
        self._pub_agl.publish(a)


def main():
    rclpy.init()
    node = HWBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Hardware bridge: publishes /drone/state, /drone/pose, /drone/agl from MAVROS.

  /drone/state   PoseStamped  position=(East_m, North_m, alt_msl_m)  ← ardupilot_commander
                 Local ENU from /mavros/local_position/pose, relative to home_elevation.json.
                 Used for velocity/waypoint control math only.

  /drone/pose    PoseStamped  position=(lat, lon, alt_amsl_m)          ← anyloc, yolo nodes
  /drone/agl     Float64      metres AGL above home                    ← anyloc, yolo nodes
                 Both read straight from ArduPilot's own EKF output (/mavros/global_position/*),
                 i.e. whatever position source (GPS or ExternalNav/VPE) is currently active —
                 the same numbers Mission Planner/QGC show. Not reconstructed from local ENU,
                 so they stay correct regardless of where the vehicle actually is.

Run: python3 control/hw_bridge.py
"""
import json
import os
import sys

_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64

_HOME_CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "home_elevation.json")
with open(_HOME_CFG) as _f:
    _h = json.load(_f)
HOME_LAT     = float(_h["lat"])
HOME_LON     = float(_h["lon"])
HOME_ALT_MSL = float(_h["centre_elev_m"])

_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE, depth=10)


class HWBridge(rclpy.node.Node):
    def __init__(self):
        super().__init__("hw_bridge")
        self._lat      = None
        self._lon      = None
        self._alt_amsl = None
        self._agl      = None

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose, _SENSOR_QOS)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._cb_gps, _SENSOR_QOS)
        self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                 self._cb_rel_alt, _SENSOR_QOS)

        self._pub_state = self.create_publisher(PoseStamped, "/drone/state", 10)
        self._pub_pose  = self.create_publisher(PoseStamped, "/drone/pose",  10)
        self._pub_agl   = self.create_publisher(Float64,     "/drone/agl",   10)

        self.get_logger().info(
            f"HW bridge ready  HOME={HOME_LAT:.5f},{HOME_LON:.5f}  MSL={HOME_ALT_MSL:.1f} m")

    def _cb_gps(self, msg):
        self._lat      = msg.latitude
        self._lon      = msg.longitude
        self._alt_amsl = msg.altitude

    def _cb_rel_alt(self, msg):
        self._agl = float(msg.data)

    def _cb_pose(self, msg):
        # EKF ENU: x=East, y=North, z=Up from home origin
        east_m  = msg.pose.position.x
        north_m = msg.pose.position.y
        up_m    = msg.pose.position.z   # AGL from home

        stamp = msg.header.stamp

        # /drone/state — local ENU metres (used by ardupilot_commander.py for control)
        s = PoseStamped()
        s.header.stamp = stamp; s.header.frame_id = "map"
        s.pose.position.x = east_m
        s.pose.position.y = north_m
        s.pose.position.z = HOME_ALT_MSL + up_m
        s.pose.orientation = msg.pose.orientation
        self._pub_state.publish(s)

        # /drone/pose — real WGS84 position from ArduPilot's own EKF (GPS or VPE, whichever
        # source is active). Not published until the first GLOBAL_POSITION_INT arrives.
        if self._lat is not None:
            p = PoseStamped()
            p.header.stamp = stamp; p.header.frame_id = "wgs84"
            p.pose.position.x = self._lat
            p.pose.position.y = self._lon
            p.pose.position.z = self._alt_amsl if self._alt_amsl is not None else HOME_ALT_MSL + up_m
            p.pose.orientation = msg.pose.orientation
            self._pub_pose.publish(p)

        # /drone/agl — ArduPilot's own relative-altitude output; falls back to EKF z only
        # until the first rel_alt reading arrives.
        agl = self._agl if self._agl is not None else up_m
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

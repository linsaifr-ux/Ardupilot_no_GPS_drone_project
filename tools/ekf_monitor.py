#!/usr/bin/env python3
"""Monitor EKF_STATUS_REPORT (MAVLink msgid 193) from /uas1/mavlink_source."""
import struct, sys, os
_ROS2_SITE = "/opt/ros/humble/lib/python3.10/site-packages"
if os.path.isdir(_ROS2_SITE) and _ROS2_SITE not in sys.path:
    sys.path.insert(0, _ROS2_SITE)
import rclpy, rclpy.node
from mavros_msgs.msg import Mavlink

EKF_FLAGS = {
    0x001: "ATTITUDE",
    0x002: "VEL_HORIZ",
    0x004: "VEL_VERT",
    0x008: "POS_REL",
    0x010: "POS_ABS",   # ← commander gates here
    0x020: "POS_VERT",
    0x040: "POS_AGL",
    0x080: "CONST_POS",
    0x100: "PRED_REL",
    0x200: "PRED_ABS",
}

class EKFMonitor(rclpy.node.Node):
    def __init__(self):
        super().__init__("ekf_monitor")
        self.create_subscription(Mavlink, "/uas1/mavlink_source", self._cb, 10)
        self.get_logger().info("Listening for EKF_STATUS_REPORT (msgid 193) ...")

    def _cb(self, msg):
        if msg.msgid != 193:
            return
        raw = b"".join(p.to_bytes(8, "little") for p in msg.payload64)
        if len(raw) < 22:
            return
        vel_var, ph_var, pv_var, comp_var, terr_var = struct.unpack_from("<5f", raw, 0)
        flags = struct.unpack_from("<H", raw, 20)[0]
        active = [name for bit, name in EKF_FLAGS.items() if flags & bit]
        ok = "✓ POS_ABS accepted" if flags & 0x010 else "✗ waiting for POS_ABS"
        print(f"flags=0x{flags:03x}  {ok}")
        print(f"  active : {', '.join(active) or 'none'}")
        print(f"  var    : vel={vel_var:.3f}  pos_h={ph_var:.3f}  pos_v={pv_var:.3f}  compass={comp_var:.3f}")
        print()

def main():
    rclpy.init()
    node = EKFMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

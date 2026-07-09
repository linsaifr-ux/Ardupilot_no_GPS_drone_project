# Shared ROS2 environment — source this in every launcher, after ROS setup.
#
# FastDDS's default shared-memory segment (512 KB) is smaller than one
# 1640x1232 rgb8 camera frame (6 MB), so /drone/camera/image_raw silently
# falls back to BEST_EFFORT UDP fragmented across ~4400 datagrams, and the
# kernel's default 208 KB socket buffers drop ~30% of frames *per subscriber*
# (measured 2026-07-09: two parallel subscribers each lost a different ~27%).
# This profile gives every participant a 64 MB SHM segment instead:
# bench went from 228/359 to 333/359 frames delivered.
#
# Only publishers of big messages strictly need it (the segment lives on the
# sending side), but exporting it everywhere keeps all nodes consistent.
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_shm_profile.xml"

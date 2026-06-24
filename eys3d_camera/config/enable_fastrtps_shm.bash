# eys3d_camera — opt-in FastRTPS transport tuning for large-image streaming.
#
# Sourcing this script exports FASTRTPS_DEFAULT_PROFILES_FILE so the current
# shell (and every ROS process launched from it) uses the bundled XML profile
# that enlarges the SHM transport segment to 32 MB. The default 512 KB segment
# is smaller than a single image frame and forces a UDP loopback fallback that
# can drop fragments under bursty publishing.
#
# Both publisher and subscriber shells need this enabled — asymmetric setting
# has no effect.
#
# This script is NOT sourced automatically. Customers opt in explicitly via:
#
#   One-shot (this terminal only):
#     source $(ros2 pkg prefix eys3d_camera)/share/eys3d_camera/config/enable_fastrtps_shm.bash
#
#   Persistent (every future terminal of this user — recommended):
#     echo 'source $(ros2 pkg prefix eys3d_camera)/share/eys3d_camera/config/enable_fastrtps_shm.bash' \
#       >> ~/.bashrc
#     # Make sure the line sits AFTER your `source /opt/ros/<distro>/setup.bash`
#     # line so that `ros2 pkg prefix` resolves correctly.
#
# To undo, delete the line from ~/.bashrc and open a fresh terminal.
#
# Cost: each ROS 2 participant on this host claims ~32 MB in /dev/shm.

_eys3d_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export FASTRTPS_DEFAULT_PROFILES_FILE="${_eys3d_dir}/fastrtps_shm.xml"
unset _eys3d_dir

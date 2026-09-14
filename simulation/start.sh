#!/usr/bin/env bash
set -e
EP_V2_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export IGN_IP=127.0.0.1
export PYTHONNOUSERSITE=1
case "${1:-launch}" in
  run) ros2 service call /pick_place/run std_srvs/srv/Trigger '{}' ;;
  stop) ros2 service call /pick_place/stop std_srvs/srv/Trigger '{}' ;;
  check) python3 "$EP_V2_ROOT/test_model.py" ;;
  launch)
    if pgrep -f '^ign gazebo (server|gui|.*ep_.*world)' >/dev/null; then
      echo 'Stop the existing Gazebo experiment first.' >&2; exit 1
    fi
    exec ros2 launch "$EP_V2_ROOT/ep_transfer.launch.py" ;;
  *) echo 'Usage: bash start.sh [launch|run|stop|check]' >&2; exit 2 ;;
esac

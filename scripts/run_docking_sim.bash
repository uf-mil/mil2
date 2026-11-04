#!/usr/bin/env bash
# Run in a subprocess so the user's workspace environment stays unchanged.
set -e
DOCKING_REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# The Gazebo launcher scans all AMENT packages. Exclude stale unrelated
# workspace installs, including dangling resource-index symlinks.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
for docking_package in prop_vision prop_missions prop_localization prop_controller prop_gazebo; do
    docking_setup="$DOCKING_REPO/install/$docking_package/share/$docking_package/package.bash"
    if [[ ! -f "$docking_setup" ]]; then
        echo "Missing $docking_package. Build with:" >&2
        echo "colcon build --packages-select prop_vision prop_missions prop_localization prop_controller prop_gazebo --symlink-install" >&2
        exit 1
    fi
    source "$docking_setup"
done
export AMENT_PREFIX_PATH="$DOCKING_REPO/install/prop_vision:$DOCKING_REPO/install/prop_missions:$DOCKING_REPO/install/prop_gazebo:$DOCKING_REPO/install/prop_controller:$DOCKING_REPO/install/prop_localization:/opt/ros/jazzy"
# scripts/setup.bash selects Zenoh/domain 37 for the vehicle. Keep this
# exercise independent of that router and domain. Overrides are explicit.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${DOCKING_ROS_DOMAIN_ID:-75}"
export GZ_PARTITION="${DOCKING_GZ_PARTITION:-mil2_docking_${USER}}"
exec ros2 launch prop_gazebo docking_sim.launch.py "$@"

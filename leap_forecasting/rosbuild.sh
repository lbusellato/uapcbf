# get name of the current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PACKAGE_NAME=$(basename $DIR)

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select $PACKAGE_NAME
# colcon build --packages-select collaborice_leap_mockup
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/mnt/c/Users/fcunico/github/collaborice
ros2 run $PACKAGE_NAME talker
# ros2 run collaborice_leap_mockup listener

DT_ID=1
DT_PATH=$(dirname $(readlink -f "${BASH_SOURCE:-$0}"))
DT_IP=$(expr 100 + $DT_ID)
source $DT_PATH/catkin_ws/devel/setup.bash
export ROS_IP=192.168.100.$DT_IP
export ROS_MASTER_URI="http://192.168.100.100:11311/"
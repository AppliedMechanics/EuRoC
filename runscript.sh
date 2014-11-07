source /opt/ros/hydro/setup_client.bash
source /opt/euroc_c2s1/code/devel/setup.bash

export ROS_LOG_DIR=/tmp/euroc_c2

roslaunch am_launchfiles statemachine.launch task_name:="task1_v1"
roslaunch am_launchfiles statemachine.launch task_name:="task2_v1_1"
roslaunch am_launchfiles statemachine.launch task_name:="task3_v1"
roslaunch am_launchfiles statemachine.launch task_name:="task4_v1_1"
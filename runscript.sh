#source /opt/ros/hydro/setup_client.bash
#source /opt/euroc_c2s1/code/devel/setup.bash

name="task1_v1"
echo "starting " $name
export RELAUNCH=TRUE
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
if [ "$RELAUNCH" == "TRUE" ]
  then
    echo "relaunching " $name
    roslaunch am_launchfiles final_eval.launch task_name:=$name
  else
    echo "finished " $name
fi

name="task1_v2"
echo "starting " $name
export RELAUNCH=TRUE
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
if [ "$RELAUNCH" == "TRUE" ]
  then
    echo "relaunching " $name
    roslaunch am_launchfiles final_eval.launch task_name:=$name
  else
    echo "finished " $name
fi

name="task1_v3"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v1_1"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v1_2"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v1_3"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task3_v1"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task3_v2"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task3_v3"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v1_1"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v1_2"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v1_3"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v2_1"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v2_2"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task4_v2_3"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task4_v3_1"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task4_v3_2"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v3_1"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v3_2"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
# name="task2_v3_3"
# export ROS_LOG_DIR=~/roslog/$name
# roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task3_v1"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task3_v2"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task3_v3"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task4_v1_1"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task4_v1_2"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task4_v1_3"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
name="task5_v1"
export ROS_LOG_DIR=~/roslog/$name
roslaunch am_launchfiles final_eval.launch task_name:=$name
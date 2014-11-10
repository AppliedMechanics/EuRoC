#source /opt/ros/hydro/setup_client.bash
#source /opt/euroc_c2s1/code/devel/setup.bash

array=(task1_v1 task1_v2 task1_v3 task2_v1_1 task2_v1_2 task2_v1_3 task2_v2_1 task2_v2_2 task2_v2_v3 task3_v1 task3_v2 task4_v1_1 task4_v1_2 task4_v1_3 task4_v2_1 task4_v2_2 task4_v2_3)

for task in ${array[*]}
do
    name=$task
    echo "starting " $name
    export RELAUNCH=TRUE
    export ROS_LOG_DIR=~/roslog/$name
    roslaunch am_launchfiles final_eval.launch task_name:=$name
    # if [ "$RELAUNCH" == "TRUE" ]
    # then
    # 	echo "relaunching " $name
    # 	roslaunch am_launchfiles final_eval.launch task_name:=$name
    # else
    # 	echo "finished " $name
    # fi

    while [ "$RELAUNCH" == "TRUE" ]
    do
	echo "relaunching " $name
	roslaunch am_launchfiles final_eval.launch task_name:=$name
    done

    echo "finished " $name
done



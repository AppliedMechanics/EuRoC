#source /opt/ros/hydro/setup_client.bash
#source /opt/euroc_c2s1/code/devel/setup.bash

#array=(task1_v1 task1_v2 task1_v3 task2_v1_1 task2_v1_2 task2_v1_3 task2_v2_1 task2_v2_2 task2_v2_v3 task3_v1 task3_v2 task4_v1_1 task4_v1_2 task4_v1_3 task4_v2_1 task4_v2_2 task4_v2_3)

#array=(task2_v1_1 task2_v1_2 task2_v1_3 task2_v2_1 task2_v2_2 task2_v2_v3 task3_v1 task3_v2 task4_v1_1 task4_v1_2 task4_v1_3 task4_v2_1 task4_v2_2 task4_v2_3)
array=(task5_v1 task5_v2)
#end version:
#array=("task 1" "task 2/1" "task 2/2" "task 2/3" "task 3") # "task 4/1" "task 4/2" "task 4/3" "task 5" "task 6")

mkdir /tmp/euroc_c2s1

for ((i = 0; i < ${#array[@]}; i++))
do
    rm -rf /tmp/euroc_c2s1/relaunch.txt

    name=${array[$i]}
    echo "starting " $name
    export ROS_LOG_DIR=~/roslog/$name
    roslaunch am_launchfiles final_eval.launch task_name:="$name"

    while [ -f "/tmp/euroc_c2s1/relaunch.txt" ]
    do
	rm -rf /tmp/euroc_c2s1/relaunch.txt
	echo "relaunching " $name
	roslaunch am_launchfiles final_eval.launch task_name:="$name"
    done

    echo "finished " $name
done



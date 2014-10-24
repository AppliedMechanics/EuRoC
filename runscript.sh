roslaunch am_launchfiles statemachine.launch skip_vision:=true task_nr:=11

rosnode kill /am_motion_planning
rosnode kill /am_move_group
rosnode kill /am_grasping_srv
rosnode kill /am_gripper_interface
rosnode kill /am_gui
rosnode kill /am_stateobserver
rosnode kill /am_tf_broadcaster
rosnode kill /am_vision
rosnode kill /move_group
rosnode kill /octomap_server


roslaunch am_launchfiles statemachine.launch skip_vision:=true task_nr:=211

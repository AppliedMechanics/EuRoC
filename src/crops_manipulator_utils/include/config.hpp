#ifndef _CONFIG_
#define _CONFIG_

#include <stdint.h>
#include <fsm_state.hpp>
#include <utils.hpp>

typedef enum {
	POSITION=0,
	FF_FORCE,
	FB_FORCE,
	RELEASE
} gripping_mode_t;

typedef enum {
	OBJ_NOT_LOCATED=0,
	OBJ_LOCATED,
	OBJ_GRIPPING,
	OBJ_GRABED,
	OBJ_PLACED,
	OBJ_FINISHED,
	OBJ_REMOVE_SHADOW
} object_action_t;

typedef enum {
	OBJ_STATE_IN_WORLD=0,
	OBJ_STATE_NOT_IN_WORLD,
	OBJ_STATE_GRABBED
} object_state_t;


typedef enum {
	EXECUTE_NOW=1,
	EXECUTE_LATER=0,
}scheduler_mode_t;

typedef enum {
	STANDARD_IK_7DOF = 0,
	MOVE_IT_2DOF,
	MOVE_IT_7DOF,
	MOVE_IT_9DOF,
	MOVE_IT_7DOF_MOVE_TO_OBJECT,
	MOVE_IT_9DOF_MOVE_TO_OBJECT,
	HOMING_7DOF,
	T6_MOVE_IT_9DOF_BELTHOMING,
	T6_MOVE_IT_9DOF_TARGET,
	T6_MOVE_IT_2DOF,
	T6_STANDARD_IK_7DOF,
	HOMING_MOVE_IT_2DOF,
	HOMING_MOVE_IT_7DOF,
	HOMING_MOVE_IT_9DOF,
	MOVE_IT_JT_2DOF,
	MOVE_IT_JT_7DOF,
	MOVE_IT_JT_9DOF
} motion_planning_mode_t;

typedef enum {
	NO_ERROR = 0,
	STOP_COND,
	MAX_LIMIT_REACHED,
	NO_IK,
	OTHER
} motion_planning_error_t;

typedef enum {
	SINGLE_POSE_TARGET=-1,
	POSE_TARGET=0,
	JOINT_VALUE_TARGET_KDL_IK,
	JOINT_VALUE_TARGET_EUROC_IK,
	HOMING,
	JOINT_VALUE_TARGET_9DOF,
	APPROX_JOINT_VALUE_TARGET_9DOF,
	POSE_TARGET_EXP,
	APPROXIMATE_JOINT_VALUE_TARGET=20
} planning_target_type_t;

typedef enum {
	OPEN=0,
	RUNNING,
	FINISHED,
	FINISHEDWITHERROR
} thread_state_t;

typedef enum {
	SCENE_CAM=0,
	TCP_CAM,
	SCENE_CAM_WITHOUT_ROBOT
} camera_type_t;

typedef enum {
	GLOBAL_POSE_ESTIMATION=0,
	CLOSE_RANGE_POSE_ESTIMATION,
	CHECKING_FOR_OBJECT_IN_TARGET_ZONE
} vision_mode_t;

typedef enum {
	OBJECT_UNKNOWN=0,
	OBJECT_HANDLE,
	OBJECT_CYLINDER,
	OBJECT_CUBE,
	OBJECT_PUZZLE
} object_type_t;

typedef enum {
	SHAPE_UNKNOWN=0,
	SHAPE_BOX,
	SHAPE_CYLINDER
} shape_type_t;

typedef enum {
	OBJECT_POSE_UNKNOWN=0,
	OBJECT_POSE_CUBE_X_UP,
	OBJECT_POSE_CUBE_Y_UP,
	OBJECT_POSE_CUBE_Z_UP,
	OBJECT_POSE_CUBE_TASK6,
	OBJECT_POSE_CYLINDER_HORIZONTAL,
	OBJECT_POSE_CYLINDER_VERTICAL,
	OBJECT_POSE_HANDLE_HORIZONTAL_XUP,	//x-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_HORIZONTAL_YUP,	//y-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_VERTICAL_ZUP,	//z-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_VERTICAL_ZDOWN, 	//z-axis of cylinder shape points downwards
	OBJECT_POSE_PUZZLE_XUP,
	OBJECT_POSE_PUZZLE_XDOWN,
	OBJECT_POSE_PUZZLE_YUP,
	OBJECT_POSE_PUZZLE_YDOWN,
	OBJECT_POSE_PUZZLE_ZUP,
	OBJECT_POSE_PUZZLE_ZDOWN
} object_pose_type_t;

typedef enum {
	GRIP_POSE_CUBE_X_UP=0,
	GRIP_POSE_CUBE_X_UP_45byY,			//rotated 45° by y-axis of object
	GRIP_POSE_CUBE_X_UP_45byZ,			//rotated 45° by z-axis of object
	GRIP_POSE_CUBE_Y_UP,
	GRIP_POSE_CUBE_Y_UP_45byX,			//rotated 45° by x-axis of object
	GRIP_POSE_CUBE_Y_UP_45byZ,			//rotated 45° by z-axis of object
	GRIP_POSE_CUBE_Z_UP,
	GRIP_POSE_CUBE_Z_UP_45byX,			//rotated 45° by x-axis of object
	GRIP_POSE_CUBE_Z_UP_45byY,			//rotated 45° by y-axis of object
	GRIP_POSE_CUBE_TASK6,
	GRIP_POSE_CYLINDER_VERTICAL,
	GRIP_POSE_CYLINDER_VERTICAL_45,
	GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ,	//z-axis of gripper points in (+-) x-direction of cylinder y-axis of gripper points in positive z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ,	//z-axis of gripper points in (+-) x-direction of cylinder y-axis of gripper points in negative z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ,	//z-axis of gripper points in (+-) y-direction of cylinder y-axis of gripper points in positive z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ,	//z-axis of gripper points in (+-) y-direction of cylinder y-axis of gripper points in negative z-direction of cylinder
	GRIP_POSE_HANDLE_BOX1_ZEQX_YPOSZ,
	GRIP_POSE_HANDLE_BOX1_ZEQX_YNEGZ,
	GRIP_POSE_HANDLE_BOX1_ZEQY_YPOSZ,
	GRIP_POSE_HANDLE_BOX1_ZEQY_YNEGZ,
	GRIP_POSE_HANDLE_BOX2_ZEQX_YPOSZ,
	GRIP_POSE_HANDLE_BOX2_ZEQX_YNEGZ,
	GRIP_POSE_HANDLE_BOX2_ZEQY_YPOSZ,
	GRIP_POSE_HANDLE_BOX2_ZEQY_YNEGZ,
	GRIP_POSE_PUZZLE_FROM_TOP,
	GRIP_POSE_PUZZLE_FROM_SIDE,
	GRIP_POSE_PUZZLE_FLIPPING
} grip_pose_type_t;

typedef enum {
	PLACE_POSE_CUBE_X_UP=0,
	PLACE_POSE_CUBE_X_UP_45byY,
	PLACE_POSE_CUBE_X_UP_45byZ,
	PLACE_POSE_CUBE_Y_UP,
	PLACE_POSE_CUBE_Y_UP_45byX,
	PLACE_POSE_CUBE_Y_UP_45byZ,
	PLACE_POSE_CUBE_Z_UP,
	PLACE_POSE_CUBE_Z_UP_45byX,
	PLACE_POSE_CUBE_Z_UP_45byY,
	PLACE_POSE_CUBE_TASK6,
	PLACE_POSE_CYLINDER_VERTICAL,
	PLACE_POSE_CYLINDER_VERTICAL_45,
	PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL,
	PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL,
	PLACE_POSE_HANDLE_BOX1_YPOSZ_VERTICAL,
	PLACE_POSE_HANDLE_BOX1_YNEGZ_VERTICAL,
	PLACE_POSE_HANDLE_BOX2_YPOSZ_VERTICAL,
	PLACE_POSE_HANDLE_BOX2_YNEGZ_VERTICAL,
	PLACE_POSE_PUZZLE_XUP,
	PLACE_POSE_PUZZLE_XDOWN,
	PLACE_POSE_PUZZLE_YUP,
	PLACE_POSE_PUZZLE_YDOWN,
	PLACE_POSE_PUZZLE_ZUP,
	PLACE_POSE_PUZZLE_ZDOWN
} place_pose_type_t;

typedef enum {
	EXPLORE_STD_1=0,
	EXPLORE_STD_2,
	EXPLORE_SNAKE
} explore_pose_type_t;

static const uint32_t slow_moving_speed = 10; // in percent
static const uint32_t std_moving_speed = 30; // in percent
static const uint32_t fast_moving_speed = 60; // in percent
static const uint32_t std_inter_steps = 10;//5
static const double   wait_duration = 2; //wait duration in wait state

const std::string ORIGIN = "Origin";
const std::string LA_0 = "LA_0";
const std::string LWR_0 = "LWR_0";
const std::string LWR_TCP = "LWR_TCP";
const std::string GP_0 = "GP_0";
const std::string GP_TCP = "GP_TCP";
const std::string T_RGB = "T_RGB";
const std::string T_DEPTH = "T_DEPTH";
const std::string CM = "CM";
const std::string PT_0 = "PT_0";
const std::string PT_TCP = "PT_TCP";
const std::string S_RGB = "S_RGB";
const std::string S_DEPTH = "S_DEPTH";
const std::string PUZZLE_FIXTURE = "Puzzle";
//! KUKA joints
const std::string lwr_1 = "joint1";
const std::string lwr_2 = "joint2";
const std::string lwr_3 = "joint3";
const std::string lwr_4 = "joint4";
const std::string lwr_5 = "joint5";
const std::string lwr_6 = "joint6";
const std::string lwr_7 = "joint7";
const std::string OBJ_POSE = "obj_pose";

// Vision Empty Point Cloud flag
static bool emptyCloudCritical;

#define FREQ 100 //rosnode frequency

#endif

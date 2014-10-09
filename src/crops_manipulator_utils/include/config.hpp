#ifndef _CONFIG_
#define _CONFIG_

#include <stdint.h>
#include <fsm_state.hpp>

typedef enum {
	POSITION=0,
	FF_FORCE,
	FB_FORCE,
	RELEASE
} gripping_mode_t;

typedef enum {
	EXECUTE_NOW=1,
	EXECUTE_LATER=0,
}scheduler_mode_t;

typedef enum {
	STANDARD_IK_7DOF = 0,
	MOVE_IT,
	HOMING_7DOF
} motion_planning_mode_t;

typedef enum {
	NO_ERROR = 0,
	STOP_COND,
	MAX_LIMIT_REACHED,
	NO_IK,
	OTHER
} motion_planning_error_t;

typedef enum {
	OPEN=0,
	RUNNING,
	FINISHED,
	FINISHEDWITHERROR
} thread_state_t;

typedef enum {
	SCENE_CAM=0,
	TCP_CAM
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
	OBJECT_CUBE
} object_type_t;

typedef enum {
	SHAPE_UNKNOWN=0,
	SHAPE_BOX,
	SHAPE_CYLINDER
} shape_type_t;

typedef enum {
	OBJECT_POSE_CUBE_X_UP=0,
	OBJECT_POSE_CUBE_Y_UP,
	OBJECT_POSE_CUBE_Z_UP,
	OBJECT_POSE_CYLINDER_HORIZONTAL,
	OBJECT_POSE_CYLINDER_VERTICAL,
	OBJECT_POSE_HANDLE_HORIZONTAL_XUP,	//x-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_HORIZONTAL_YUP,	//y-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_VERTICAL_ZUP,	//z-axis of cylinder shape points upwards
	OBJECT_POSE_HANDLE_VERTICAL_ZDOWN, 	//z-axis of cylinder shape points downwards
} object_pose_type_t;

typedef enum {
	GRIP_POSE_CUBE_X_UP=0,
	GRIP_POSE_CUBE_Y_UP,
	GRIP_POSE_CUBE_Z_UP,
	GRIP_POSE_CYLINDER_VERTICAL,
	GRIP_POSE_HANDLE_CYLINDER_ZEQX_YPOSZ,	//z-axis of gripper points in (+-) x-direction of cylinder y-axis of gripper points in positive z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQX_YNEGZ,	//z-axis of gripper points in (+-) x-direction of cylinder y-axis of gripper points in negative z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQY_YPOSZ,	//z-axis of gripper points in (+-) y-direction of cylinder y-axis of gripper points in positive z-direction of cylinder
	GRIP_POSE_HANDLE_CYLINDER_ZEQY_YNEGZ,	//z-axis of gripper points in (+-) y-direction of cylinder y-axis of gripper points in negative z-direction of cylinder
} grip_pose_type_t;

typedef enum {
	PLACE_POSE_CUBE_X_UP,
	PLACE_POSE_CUBE_Y_UP,
	PLACE_POSE_CUBE_Z_UP,
	PLACE_POSE_CYLINDER_VERTICAL,
	PLACE_POSE_HANDLE_CYLINDER_YPOSZ_VERTICAL,
	PLACE_POSE_HANDLE_CYLINDER_YNEGZ_VERTICAL,
} place_pose_type_t;

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
//! KUKA joints
const std::string lwr_1 = "joint1";
const std::string lwr_2 = "joint2";
const std::string lwr_3 = "joint3";
const std::string lwr_4 = "joint4";
const std::string lwr_5 = "joint5";
const std::string lwr_6 = "joint6";
const std::string lwr_7 = "joint7";
const std::string OBJ_POSE = "obj_pose";

#define FREQ 100 //rosnode frequency

#endif

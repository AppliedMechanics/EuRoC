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
	STANDARD_IK_7DOF = 0,
	MOVE_IT,
	HOMING_7DOF
} motion_planning_mode_t;

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

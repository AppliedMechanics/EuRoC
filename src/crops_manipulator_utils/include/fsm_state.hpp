#ifndef _FSM_STATE_
#define _FSM_STATE_

#include <stdint.h>

namespace fsm{
typedef union {
	uint64_t all;
	struct {
		uint8_t one;
		uint8_t two;
		uint8_t three;
		uint8_t four;
		uint8_t event_one;
		uint8_t event_two;
		uint8_t event_three;
		uint8_t event_four;
	} sub;
} fsm_state_t;

typedef enum {
	INITIAL_STATE = 0,
	REQUEST_TASK,
	START_SIM,
	PARSE_YAML,
	SCHEDULER,
	WATCH_SCENE,
	EXPLORE_ENVIRONMENT,
	EXPLORE_ENVIRONMENT_INIT,
	EXPLORE_ENVIRONMENT_MOTION,
	EXPLORE_ENVIRONMENT_IMAGE,
	EXPLORE_ENVIRONMENT_CHECK,
	SOLVE_TASK,
	STOP_SIM,
	FINISHED,
	RESET,
	LOCATE_OBJECT_GLOBAL,
	LOCATE_ALL_OBJECTS_GLOBAL,
	LOCATE_OBJECT_CLOSE_RANGE,
	GET_GRASPING_POSE,
	GET_GRASPING_POSE_T5,
	GET_GRASPING_POSE_T6,
	GRAB_OBJECT,
	PLACE_OBJECT,
	MOVE_TO_OBJECT_VISION,
	MOVE_TO_OBJECT_SAFE,
	MOVE_TO_OBJECT,
	MOVE_TO_OBJECT_T6,
	MOVE_TO_TARGET_ZONE_VISION,
	MOVE_TO_TARGET_ZONE_SAFE,
	MOVE_TO_TARGET_ZONE,
	GRIPPER_RELEASE,
	GRIPPER_CLOSE,
	HOMING,
	CHECK_OBJECT_FINISHED,
	CHECK_OBJECT_GRIPPED,
	PAUSE,
	NEW_OBJECT_T6,

	//events
	NOP,
	RETRY,
	SIM_SRV_NA,
	SKIP_OBJECT,
	OBJECT_LOST,

	//motion planning error events:
	MOTION_PLANNING_ERROR,
	NO_IK_SOL,
	NO_DK_SOL,
	MAX_LIMIT_REACHED,
	STOP_COND,

	//gripper error events:
	GRIPPING_ERROR,

	//vision error events:
	DATA_ERROR,
	VISION_ERROR,
	POSE_NOT_FOUND

} fsm_t;
}
#endif

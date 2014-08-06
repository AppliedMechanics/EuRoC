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
		uint8_t event_one;
		uint8_t event_two;
		uint8_t event_three;
	} sub;
} fsm_state_t;

typedef enum {
	REQUEST_TASK = 0,
	START_SIM,
	PARSE_YAML,
	SOLVE_TASK,
	STOP_SIM,
	FINISHED,
	// sub-states for solve task
	LOCATE_OBJECT,
	GET_GRASPING_POSE,
	MOVE_TO_OBJECT,
	GRIP,
	MOVE_TO_TARGET_ZONE,
	HOMING,
	//sub-states for move_to_object
	MOVE_WITH_METHOD_A,
	MOVE_WITH_METHOD_B,
	VISUAL_SERVOING,

	//events
	OBJECT_NOT_DETECTED,
	MOTION_PLANNING_ERROR,
	OBJECT_LOST
} fsm_t;
}
#endif

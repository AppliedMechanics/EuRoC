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
	FINISHED
} thread_state_t;

#define FREQ 10 //rosnode frequency

#endif

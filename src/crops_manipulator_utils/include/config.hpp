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
	MOVE_IT
} motion_planning_mode_t;

#endif


#ifndef __trajectoryh__
#define __trajectoryh__

#ifdef DEBUG_TRAJECTORY
#include "can1.h"
#endif

Int16 init_trajectory (byte jj, Int32 current, Int32 final, Int16 speed);
Int16 abort_trajectory (byte jj, Int32 limit);
Int32 step_trajectory (byte jj);
Int32 step_trajectory_delta (byte jj);

#endif 

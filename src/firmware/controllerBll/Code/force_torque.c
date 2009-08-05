#include "force_torque.h"

Int16 _torque[TN] = INIT_ARRAY (0);
Int16 _torque_init[TN] = INIT_ARRAY (0);
Int16 _torque_old[TN] = INIT_ARRAY (0);
Int32 _Feq = 0;
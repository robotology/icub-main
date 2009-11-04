#ifndef __force_torqueh__
#define __force_torqueh__

#include "controller.h"

#define TN					6		/* number of tensioners */

extern Int16 _torque[TN];
extern Int16 _torque_init[TN];
extern Int16 _torque_old[TN];
extern Int32 _Feq;

#endif
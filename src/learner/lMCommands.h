// lMCommands.h : interacting with a learner application
//

#ifndef __lMCommandsh__
#define __lMCommandsh__

#include <yarp/os/begin_pack_for_net.h>

namespace lMCommand {
	enum {
		Ok = 0,
		Failed = 1,
		Reset = 2,
		Quit = 3
	};
};

#include <yarp/os/end_pack_for_net.h>

#endif

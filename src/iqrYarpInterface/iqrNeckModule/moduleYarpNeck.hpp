/**
 * @class moduleYarpNeck
 *
 *
 * @brief Provide an example for using the Proxy class iCubLimbIF
 *
 *
 *
 * @author Zenon Mathews, Ugo Pattacini $
 *
 * @version $Revision: 1.0 $
 *
 * @date $Date: 2009/09/23 14:16:20 $
 *
 * Contact: zenon.mathews@gmail.com
 *
 *
 *
 */


#ifndef MODULEYARPNECK_HPP
#define MODULEYARPNECK_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#include <Common/Item/threadModule.hpp>
#include "yarpIFlib.hpp"

namespace iqrcommon {

    class moduleYarpNeck : public ClsThreadModule {
        public:
            moduleYarpNeck();
            ~moduleYarpNeck();

            void init();

	    /** this method is called in every cycle
	     */
            void update();

	
            void cleanup();

        private:
        
	    /** for input from the robot to IQR
	     */
            ClsStateVariable *headEncs;

	    /** for commands from IQR to the robot
	     */ 
            StateVariablePtr *headCmds;

            yarpIF::iCubLimbIF *head;
            int nAxes;
    };
}
#endif

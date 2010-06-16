#ifndef MODULEYARPED_HPP
#define MODULEYARPED_HPP

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

#include <vector>


#include <Common/Item/threadModule.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <math.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

struct Point{
    float x,y;
};

namespace iqrcommon {

    class moduleYarped : public ClsThreadModule {
        public:
            moduleYarped();
            ~moduleYarped();

            void init();
            void update();
            void cleanup();

	bool firstTime;

	int attentionCount;

        private:
	
	ClsStateVariable *bot_up_saliency_left_faces; // receives bottom up saliency from left cam
	ClsStateVariable *bot_up_saliency_right_faces; // receives bottom up saliency from right cam
	                                             // in future make more group for multimodality 
	ClsStateVariable *bot_up_saliency_left_torsos; 
	ClsStateVariable *bot_up_saliency_right_torsos;

	ClsStateVariable *bot_up_saliency_left_moves; 
	ClsStateVariable *bot_up_saliency_right_moves;
	
	StateVariablePtr *attentionCmds; // sends attention commands to the robot

	BufferedPort<Vector> face_left_in;
	BufferedPort<Vector> face_right_in;
	BufferedPort<Vector> torso_left_in;
	BufferedPort<Vector> torso_right_in;
	BufferedPort<Vector> move_left_in;
	BufferedPort<Vector> move_right_in;

	BufferedPort<Vector> attention_out;

	

	
        
    };
}
#endif

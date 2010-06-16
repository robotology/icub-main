#ifndef MODULEHAND_HPP
#define MODULEHAND_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>



    class moduleHand {
        public:
            moduleHand();
            ~moduleHand();

            void init();
      //void update();
         void cleanup();

      void grab();
      void release();
      bool okToThrowPosition();
      bool checkBlockHand(); // returns false if release is finished
        private:
        
            
            yarp::dev::PolyDriver       *drvHand;
            yarp::dev::IControlLimits   *limHand;
            yarp::dev::IEncoders        *encHand;
            yarp::dev::IPositionControl *posHand;
 
            int nAxes;

            yarp::sig::Vector min;
            yarp::sig::Vector max;

            yarp::sig::Vector scaleFromRobot(const yarp::sig::Vector& v);
            yarp::sig::Vector scaleToRobot(const yarp::sig::Vector& v);
    };

#endif

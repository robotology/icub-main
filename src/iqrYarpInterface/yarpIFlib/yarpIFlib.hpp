/**
 * @class iCubLimbIF
 *
 *
 * @Proxy class to be used by all IQR iCub modules for YARP communication
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


#ifndef YARPIFLIB_HPP
#define YARPIFLIB_HPP

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

namespace yarpIF {

    void openYarpNetwork();
    void closeYarpNetwork();

    class iCubLimbIF {
        public:
           iCubLimbIF();
           ~iCubLimbIF();
          
            bool configure(yarp::os::Property& options);
	    
            bool readEncs(yarp::sig::Vector& encs);
            
            bool setRefSpeeds(const yarp::sig::Vector& vels);
	    bool writePosCmds(const yarp::sig::Vector& cmds);
            
            bool writeVelCmds(const yarp::sig::Vector& vels);

            int  getAxes()                     { return nAxes;   }
            yarp::sig::Vector getMin()         { return min;     }
            yarp::sig::Vector getMax()         { return max;     }
            bool getVerbosity()                { return verbose; }
            void setVerbosity(const bool verb) { verbose=verb;   }

        private:            
            yarp::dev::PolyDriver       *drv;
            yarp::dev::IControlLimits   *lim;
            yarp::dev::IEncoders        *enc;
            yarp::dev::IPositionControl *pos;
	    yarp::dev::IVelocityControl *vel;	
            
            bool configured;
            bool verbose;

            int nAxes;
	    double maxValueCell;
	
            yarp::sig::Vector min;
            yarp::sig::Vector max;

            yarp::sig::Vector scaleFromRobot(const yarp::sig::Vector& v);
            yarp::sig::Vector scaleToRobot(const yarp::sig::Vector& v);
    };
}
#endif

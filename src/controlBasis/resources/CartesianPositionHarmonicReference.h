// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_POSITION_HARMONIC_REFERENCE__H_
#define _CARTESIAN_POSITION_HARMONIC_REFERENCE__H_

#include "CartesianPosition.h"
#include <CartesianPositionHarmonicFunction.h>

#include <string>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

using namespace yarp::math;

namespace CB {

    /**
     * This class implements a "carrot" along the direction of the negative 
     * gradient of a  harmonic function that can be used as a temporary goal 
     * to another Cartesian Controller (i.e., one that is a navigation function, 
     * and doesnt have the undesirable properties of Harmonic functions).
     **/    
    class CartesianPositionHarmonicReference : public CartesianPosition {
        
    protected:

        CartesianPositionHarmonicFunction *harmonicFunction;

    public:
        
        CartesianPositionHarmonicReference(std::string name) {
            
            // set configuration info
            deviceName = name + "/harmref";
            std::string tmpName = "/cb/cartesianposition" + name;

            initPorts(); // mandatory init function            
            std::cout << "Creating new CartesianPositionHarmonicReference, name=" << deviceName.c_str() << std::endl;        

            harmonicFunction = new CartesianPositionHarmonicFunction(tmpName);
            yarp::os::Time::delay(1);
            harmonicFunction->startPotentialFunction();

        }
        
        ~CartesianPositionHarmonicReference() { 
            if(harmonicFunction->isRunning()) 
                harmonicFunction->stopPotentialFunction();
            delete harmonicFunction;
        }

        
        // functions from ControlBasisResource
        bool updateResource() { 

            double gain = -0.02;
            double potential = harmonicFunction->getPotential();
            yarp::sig::Vector Vgrad = harmonicFunction->getPotentialGradient();
            yarp::sig::Vector Vpos = harmonicFunction->getCurrentPosition();

            //printf("Harmonic Goal got potential=%f, gradient=\n%f\t%f\n%f\t%f\n%f\t%f\n",
            //   potential,Vgrad[0],Vpos[0],Vgrad[1],Vpos[1],Vgrad[2],Vpos[2]);

            double mag = sqrt(pow(Vgrad[0],2)+pow(Vgrad[1],2)+pow(Vgrad[2],2));
            yarp::sig::Vector Vstep(3);
            Vstep.zero();

            if( (mag>0) && (potential>0)) 
                Vstep = Vgrad*(1.0/mag)*(1.0/potential)*gain;
            values = Vpos+Vstep;
            return true;
        }

        void startResource() { 
            start(); 
        }
        
        void stopResource() { 
            stop(); 
        }
        
        void setGoal(const yarp::sig::Vector & ref){        
            harmonicFunction->setGoal(ref);
            return;
        }

        void setObstacle(const yarp::sig::Vector & ref, double r){
            harmonicFunction->setObstacle(ref ,r);
            return;
        }

    };
    
}

#endif

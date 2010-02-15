// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_POSITION_HARMONIC_FUNCTION__H_
#define _CARTESIAN_POSITION_HARMONIC_FUNCTION__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {
    
    class CartesianPositionHarmonicFunction : public ControlBasisPotentialFunction {
        
    protected:
        
        // HF vars 
        int ***worldMap;
        double ***potentialMap;
        yarp::sig::Vector noOfBins;

        double threshold;
        double omega;
        int iterationCount;

        bool initialized;

        yarp::sig::Vector rangeMin;
        yarp::sig::Vector rangeMax;
        yarp::sig::Vector resolution;
        yarp::sig::Vector potentialVals;
        yarp::sig::Vector curPos;

        const static int FREESPACE = 0;
        const static int OBSTACLE = 1;
        const static int DIRICHLET = 2;
        const static int GOAL = 3;

        const static double FREESPACE_POTENTIAL = 0.5;
        const static double OBSTACLE_POTENTIAL = 0.0;//1.0
        const static double DIRICHLET_POTENTIAL = 0.0;//1.0
        const static double GOAL_POTENTIAL = 1.0; //-0.3

        const static int MAX_ITERATION = 500; //Max iteration of the Gauss-Seidel iteration
        
    public:
        

        /**
         * Empty Constructor, needs to set configuration info
         **/
        CartesianPositionHarmonicFunction() :
            ControlBasisPotentialFunction("harmonic_pf", "cartesianposition", true)        
        {
            size = 3;
            gradient.resize(size);
            curPos.resize(size);
            initHarmonicFunction();
            std::cout << "Created new CartesianPositionHarmonicFunction..." << std::endl;
        }
        
        ~CartesianPositionHarmonicFunction() { }

        // functions from ControlBasisPotentialFunction
        virtual bool updatePotentialFunction();
        virtual void startPotentialFunction();
        virtual void stopPotentialFunction();
        virtual bool connectToInputs();

        // HF functions
        bool initHarmonicFunction();
        void initializeMap();
        void setGoal(const yarp::sig::Vector &goal);
        void setObstacle(const yarp::sig::Vector &pos, double rad);

        yarp::sig::Vector getCurrentPosition() { return curPos; }

    private:

        void SOR();
        void gaussSeidel();
        double sorOnce();
        double gaussSeidelOnce();
        yarp::sig::Vector computeGradient(const yarp::sig::Vector &pos);

    };
    
}

#endif

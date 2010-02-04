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
        
        CartesianPositionHarmonicFunction(std::string inName) {
            
            size = 3;
            inputName[0] = inName;
            input[0].resize(size);

            potential = 0;
            gradient.resize(3);
            curPos.resize(3);

            inputSpace = "cartesianposition";
            pfTypeName = "harmonic_pf";

            hasReference = true;           
            connectedToInputs = false;
            running = false;

            std::cout << "Creating new CartesianPositionHarmonicFunction PotentialFunction (cur=" << inputName[0].c_str() << "), size=" << gradient.size() << std::endl;

            initHarmonicFunction();

        }
        
        ~CartesianPositionHarmonicFunction() { }

        // functions from ControlBasisPotentialFunction
        bool updatePotentialFunction();
        void startPotentialFunction();
        void stopPotentialFunction();
        bool connectToInputs();

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

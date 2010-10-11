// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _HARMONIC_FUNCTION__H_
#define _HARMONIC_FUNCTION__H_

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cv.h>

#include <map>

namespace iCub {

    class HarmonicFunction {
        
    protected:
        
        double convergenceThreshold;
        double omega;
        double plotScaleFactor;
        int maxIterations;
        int iterationCount;
        int dims;
        bool displayMode;
        bool pathComputed;
        bool leaveRobotTrace;
        std::string mapName;

        yarp::sig::Vector rangeMin;
        yarp::sig::Vector rangeMax;
        yarp::sig::Vector resolution;
        yarp::sig::Vector potential;
        yarp::sig::Vector noOfBins;
        
        yarp::sig::Vector grad;
        yarp::sig::Vector currentPosition;
        
        std::map<std::pair<int,int>,double> potentialMap;
        std::map<std::pair<int,int>,int> worldMap;

        std::vector<std::pair<int,int> > pathPoints;

        IplImage *potentialImage;
        IplImage *worldImage;
        
        const static int FREESPACE = 0;
        const static int OBSTACLE = 1;
        const static int DIRICHLET = 2;
        const static int GOAL = 3;

        CvScalar freespaceColor;
        CvScalar obstacleColor;
        CvScalar goalColor;
        CvScalar dirichletColor;

        const static double FREESPACE_POTENTIAL = 0.5;
        const static double OBSTACLE_POTENTIAL = 0.0;//1.0
        const static double DIRICHLET_POTENTIAL = 0.0;//1.0
        const static double GOAL_POTENTIAL = 1.0; //-0.3
        
    public:
        HarmonicFunction(yarp::sig::Vector min, yarp::sig::Vector max, yarp::sig::Vector res, double om, int maxIter, double thresh, double plotFac, bool dispMode);
        ~HarmonicFunction();
        
        void init();
        
        void setGoal(const yarp::sig::Vector &goal);
        void setObstacle(const yarp::sig::Vector &pos, double rad);
    
        IplImage *getPotentialMap() { return potentialImage; }
        IplImage *getWorldMap() { return worldImage; }

        void printWorldMap();
        void printPotentialMap();

        double getWidth() { return noOfBins[1]; }
        double getHeight() { return noOfBins[0]; }

        void SOR();
        void gaussSeidel();

        void loadMap(std::string map);
        void saveMap(std::string map, int width);

        yarp::sig::Vector computeGradient(const yarp::sig::Vector &pos);
        void computePath(const yarp::sig::Vector &pos, bool drawTrace);
        void clearPath();

    private:

        double sorOnce();
        double gaussSeidelOnce();       

        std::pair<int,int> getMapCoordinate(const yarp::sig::Vector &pos) {            
            // need to reverse the position to correspond to internal map data structure
            std::pair<int,int> p((int)( (double)(pos[1]-rangeMin[1])/(double)resolution[1] + 0.5),
                                 (int)( (double)(pos[0]-rangeMin[0])/(double)resolution[0] + 0.5));        
            return p;
        }
    };

}

#endif

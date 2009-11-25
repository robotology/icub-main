

// Code to be implemented here (removed from Module class)

 //   _mapObjects.setSalienceDecay(0.9995); ///This value needs to be tuned or related to the refresh rate (the regular visual map decay rate is 0.95


#ifndef OBJECT_MAP_H
#define OBJECT_MAP_H

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// opencv
#include <cv.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/spherical_projection.h>
#include <iCub/EgoSphereModule.h>
#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/IModalityMap.h>

namespace iCub {
    namespace contrib {
        class ObjectMap;
    }
}

/**
 *
 * Object map 
 *
 *
 */
class iCub::contrib::ObjectMap : public iCub::contrib::IModalityMap, public yarp::os::IConfig {

    private:

        int _resXObject; // resolution of Object maps
        int _resYObject;

        BufferedPort<Bottle>                _prtObjects; //added Dario 2008
        Bottle *_objectPositions;
        
        yarp::sig::VectorOf<double> *_vctSound;
	
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgCart; // cartesian map
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgSpher; // spherical map
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgRemapX; // cartesian to spherical remap map X
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgRemapY; // cartesian to spherical remap map Y
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgMapResA; // final map, Object resolution
        yarp::sig::ImageOf<yarp::sig::PixelFloat> _imgMapResB; // final map, final resolution
        yarp::sig::ImageOf<yarp::sig::PixelRgb> _imgMapRgb; // final visualization map (not used so far)
        double *_matRotS2W; // sound to world coordinates transformation
        double *_matRotW2S; // world to sound coordinates transformation (3x3 as size 9 array)

        float _weightObject;

        double _salienceDecayRate;

        iCub::contrib::iCubHeadKinematics _headKin;
        RobMatrix _robMatRot;

        void renderGaussianDistribution(yarp::sig::ImageOf<yarp::sig::PixelFloat> &img,
                                        int posX, int posY, 
                                        int sizex, int sizey, 
                                        float sigmaX, float sigmaY, 
                                        float height);
    // convert from RobMatrix to 1 dimensional 3x3 -> 1x9 array
        void convertRobMatrix(RobMatrix &robMatrix, double *matrix);
        inline void transpose(double *src, double *dst){
            dst[0] = src[0]; dst[1] = src[3]; dst[2] = src[6];
            dst[3] = src[1]; dst[4] = src[4]; dst[5] = src[7];
            dst[6] = src[2]; dst[7] = src[5]; dst[8] = src[8];
        }
        
        bool addObservation(VectorOf<double> &vctSound, double *encoders);

    public:
   
        ObjectMap();
        virtual ~ObjectMap();

        bool read(); 
        bool updateDecay();
        bool process(	double azREye, double elREye, int xREye, int yREye, double *rotREye,
                        double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
                        double azHead, double elHead, int xHead, int yHead, double *rotHead,
                        bool blnSaccadicSuppression);
        bool write(bool saccadicSuppression);
        yarp::sig::ImageOf<yarp::sig::PixelFloat>& getSalienceMap(int width, int height);
        yarp::sig::ImageOf<yarp::sig::PixelRgb>& getVisualizationMap(int width, int height);
	
        bool reset() ;

        /** Passes config on to iCub::contrib::CalibTool */
        bool open(yarp::os::Searchable& config);
        bool interrupt();
        bool close();

    // some egosphere controls
        bool setSalienceDecay(double rate);
        double getSalienceDecay();

        bool setWeightObject(float w);
        float getWeightObject();

};


#endif //OBJECT_MAP_H

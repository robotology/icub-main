// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef __UZH_OPTFLOWEMD__
#define __UZH_OPTFLOWEMD__

// opencv
#include <cv.h>
//#include <cxcore.h>
//#include <cxtypes.h>
//#include <cxerror.h>
//#include <cvtypes.h>
#include <cxmisc.h>

//#include <string>
//#include <ostream>
//#include <iostream>
//#include <vector>
//#include <math.h>
    
  
// yarp
//#include <yarp/sig/Image.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/IConfig.h>

// iCub
#include <iCub/IOpticalFlow.h>

namespace iCub {
    namespace contrib {
        class  OptFlowEMD;
    }
}

//using namespace std;
//using namespace yarp::os;
//using namespace yarp::sig;
using namespace iCub::contrib;

/**
 * Class to calculate optical flow using the Elementary Motion detection method.\n
 */

class iCub::contrib::OptFlowEMD : public yarp::os::IConfig,
                                         IOpticalFlow
{
    public:

        /** Constructor */
        OptFlowEMD();

        /** Destructor */
        virtual ~OptFlowEMD();

        // IConfig
        virtual bool open(yarp::os::Searchable& config);
        virtual bool configure (yarp::os::Searchable &config);
        virtual bool close();
        
        // this corresponds to IOptFlow (not inherited in iCub context)
        virtual void calculate_flow(IplImage* imageT, IplImage* imageTMinus1, IplImage* velx, IplImage* vely, IplImage* abs=0);
        
        /* Caution, no check is done on image format. See uzh::imgProc::IOpticalFlow::draw_flow */
        virtual void draw_flow(IplImage *image, IplImage *rgbX, IplImage *rgbY);
        
		virtual float getTotalFlowX();
        virtual float getTotalFlowY();
        virtual float getTotalFlowAbs();
        virtual float getMaxFlowAbsPerPixel();
		
    protected:

        // configuration
        float                           _alpha;       // emd parameter
        float                           _threshold; // motion threshold
        float							_thresholdSquared;
        bool                            _constrain;
        float                           _constrainValue;
        float                           _constrainValueSquared;
        float                           _scale;

        // old LPF's
        IplImage                        *_oldLPFx0;
        IplImage                        *_oldLPFx1;
        IplImage                        *_oldLPFy0;
        IplImage                        *_oldLPFy1;

        CvSize                          _oldImgSize;

		float							_totFlowX;
		float							_totFlowY;
		float							_totFlowAbs;
		float							_maxFlowPixelAbs;
		float							_maxFlowPixelX;
		float							_maxFlowPixelY;
		

        // opt flow calculation
        CvStatus CV_STDCALL             calcOptFlowEMD    ( uchar*  imgA,
                                                            uchar*  imgB,
                                                            int     imgStep,
                                                            CvSize  imgSize,
                                                            float*  velocityX,
                                                            float*  velocityY,
                                                            int     velStep,
                                                            float*  absolute);
        void                            initImages(CvSize size);
        void                            releaseImages();
        
        
};

#endif

 
 
 

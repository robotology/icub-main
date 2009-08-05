// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#ifndef __CLUSTERBLOB__
#define __CLUSTERBLOB__

// std
#include <time.h>

// opencv
#include <cv.h>

// cvBlobsLib
#include <Blob.h>
#include <BlobResult.h>

// yarp
//#include <yarp/os/Property.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/IConfig.h>
#include <yarp/os/Value.h>

// iCub
#include <iCub/BlobFunctions.h>

namespace iCub {
	namespace contrib {
		class ClusterBlob;
	}	
}

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

#define FILTER_ONE_NO      0
#define FILTER_ONE_BIGGEST 1
#define FILTER_ONE_MIDDLE  2
#define FILTER_ONE_RANDOM  3

/**
 * Class for calculating and drawing blobs on grayscale IplImages. 
 *
 * The calculation and drawing is done taking into account the applied configuration.
 *
 * \see icub_visual_blobs
 */
class iCub::contrib::ClusterBlob : public IConfig
{
	public:
		ClusterBlob();
		virtual ~ClusterBlob();
		
		/**
		 * Calculates blobs in grayscale (floating point?) image and adds them to *blobs.
		 * @param inputImage grayscale image.
		 * @param blobs CBlobsResult to which blobs in inputImage are added.
		 */
		virtual void calculate_clusters(IplImage* inputImage, CBlobResult* blobs);
		
		/**
		 * Draws the passed blobs to the given image taking into account
		 * the to the ClusterBlob applied configuration. 
		 * @param image a IPL_DEPTH_8U, 3 image
		 * @param the blobs calculated by calculate_clusters()
		 */
		 virtual void draw_clusters(IplImage* image, CBlobResult* blobs);
		 
		/**
		 * Returns number of blobs calculated by calculate_clusters
		 */
		int							getNumBlobs();
	
        
        // IConfig
        virtual bool open (Searchable &config);
        
	protected:
	
		int							_numBlobs;
		
		// configuration
		double						_dblFilterGreater;
		double						_dblFilterLess;
		int							_intThreshold;
	    bool						_blnDrawBBox;
	    bool						_blnDrawArea;
	    int							_intFilterOne;
	    
	    CvScalar					_clrBBox;
	    CvScalar					_clrArea;
    
};



#endif /*CLUSTERBLOB_H_*/

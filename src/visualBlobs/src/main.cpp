// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// opencv
#include <cv.h>

// cvBlobs
#include <Blob.h>
#include <BlobResult.h>

// yarp
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/ClusterBlob.h>
#include <iCub/BlobFunctions.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class VisualBlobsModule;
    }
}

using namespace iCub::contrib;

#define PIXEL_FLOAT 0
#define PIXEL_RGB   1
#define VCTBLOB_EMPTY     100
#define VCTBLOB_POSX      101
#define VCTBLOB_POSY      102
#define VCTBLOB_SIZE      103
#define VCTBLOB_PERIMETER 104
#define VCTBLOB_TIMESTAMP 105

/**
 * @ingroup icub_module
 *
 * \defgroup icub_visual_blobs visual_blobs
 *
 * Module to extract 8-connected blobs from grayscale or floating point images.
 *
 *  \dot
 * digraph module_visual_blobs_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_visual_blobs {
 *      color = "black"; style = "solid";
 *      label = "visual_blobs module";
 *       "/visual_blobs/i:img";
 *       "/visual_blobs/o:img";
 *       "/visual_blobs/o:vct";
 *     }
 *     "/visual_blobs/o:vct" -> "/some_module/i:vct"
 *     "/visual_blobs/o:img" -> "/image/viewer"
 *     "/salience/map" -> "/visual_blobs/i:img"
 * \enddot
 * 
 * The input port reads ImageOf<PixelRgb> or ImageOf<PixelFloat> depending on
 * the --formatInputImage option passed. 
 * Internally the input image is converted to a grayscale image regardless of the input
 * format. In case of a floating point input the conversion results in absolute
 * values in the range of 0 to 255. Absolute floating point values > 255.0 are bound
 * to 255.
 *
 * A visualization of the calculated blobs is available at the port /visual_blobs/o:img.\n
 * Blob data is written to a port of type VectorOf<double>. The format and entries
 * of the blob data vector can be specified using the option --blobVectorFormat 
 * (See commandline output for available blob properties).\n
 *
 * \see iCub::contrib::VisualBlobsModule
 *
 * \author Jonas Ruesch
 *
 */


/**
 *
 * Visual Blobs Module class
 *
 * \see icub_visual_blobs
 *
 */
class iCub::contrib::VisualBlobsModule : public Module {

private:

    BufferedPort<VectorOf<double> >     _prtVctBlob;
    BufferedPort<ImageOf<PixelFloat> >  _prtImgInFloat;
    BufferedPort<ImageOf<PixelRgb> >    _prtImgInRgb;
    BufferedPort<ImageOf<PixelRgb> >    _prtImgOut;

    ClusterBlob                         _clusterBlob;
    int                                 _blobVectorSize;
    int                                *_blobVectorFormat;

    int                                 _flagImgIn;
    
    IplImage                            *_ocvImgGray;
    IplImage                            *_ocvImgBgr;
    IplImage                            *_ocvImgReturn;
    CvSize                              _oldImgSize;
    double                              _scale;

    double                              _refreshTime;
    double                              _startTime;
    double                              _endTime;
    double                              _execTime;

    void initImages(int width, int height){

        if (_ocvImgGray != NULL)
            cvReleaseImage(&_ocvImgGray);
        _ocvImgGray = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1 );

        if (_ocvImgBgr != NULL)
            cvReleaseImage(&_ocvImgBgr);
        _ocvImgBgr = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3 );

        if (_ocvImgReturn != NULL)
            cvReleaseImage(&_ocvImgReturn);
        _ocvImgReturn = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3 );
        
        //trick the yarp image wrapper
        _ocvImgReturn->colorModel[0] = 'R';
        _ocvImgReturn->colorModel[1] = 'G';
        _ocvImgReturn->colorModel[2] = 'B';
        _ocvImgReturn->channelSeq[0] = 'R';
        _ocvImgReturn->channelSeq[1] = 'G';
        _ocvImgReturn->channelSeq[2] = 'B'; 

    }
    
public:

    VisualBlobsModule(){
    
        _ocvImgGray = NULL;
        _ocvImgReturn = NULL;
        _ocvImgBgr = NULL;
        _oldImgSize.width = 0;
        _oldImgSize.height = 0;
        _blobVectorFormat = NULL;
    }
    
    ~VisualBlobsModule(){
    
        cvReleaseImage(&_ocvImgBgr);
        cvReleaseImage(&_ocvImgGray);
        cvReleaseImage(&_ocvImgReturn);
        delete [] _blobVectorFormat;
    }
    
    virtual bool open(Searchable& config) {
    
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name /prefix\n");
            return false;
        }

        _refreshTime = (double)(config.check("refreshTime", 
                                Value(20),  
                                "Update module every specified miliseconds (int)").asInt()/1000.0);
    
        _scale = config.check("scale",
                              Value(1.0),
                              "scaling of input image (if floating point input image)").asDouble();
                
        ConstString format = config.check("formatInputImage",
                                  Value("PixelRgb"),
                                  "Pixel format of input image [PixelFloat|PixelRgb]").asString();

        // check requested output vector format
        if (config.check("blobVectorFormat", "specify vector entries using codewords [empty|timestamp|posX|posY|size|perimeter]. "
                         "(Default = timestamp posX posY size)")){
            yarp::os::Bottle &bot = config.findGroup("blobVectorFormat");
            if (bot != NULL){
                if (bot.size() > 1){
                    _blobVectorFormat = new int[bot.size()-1];
                    _blobVectorSize = bot.size()-1;
                    for (int i = 1; i < bot.size(); i++){
                        if (strcmp(bot.get(i).asString().c_str(), "empty") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_EMPTY;
                        else if(strcmp(bot.get(i).asString().c_str(), "timestamp") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_TIMESTAMP;
                        else if(strcmp(bot.get(i).asString().c_str(), "posX") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_POSX;
                        else if(strcmp(bot.get(i).asString().c_str(), "posY") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_POSY;
                        else if(strcmp(bot.get(i).asString().c_str(), "size") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_SIZE;
                        else if(strcmp(bot.get(i).asString().c_str(), "perimeter") == 0)
                            _blobVectorFormat[i-1] = VCTBLOB_PERIMETER;
                        else
                            cout << "Unknown vector format: " << bot.get(i).asString() << endl;
                    }
                }
            }
        }

        // apply rest of config to the ClusterBlob object
        _clusterBlob.open(config);

        if (format == "PixelRgb"){
            _flagImgIn = PIXEL_RGB;
            _prtImgInRgb.open(getName("i:img"));
        }
        else if (format == "PixelFloat"){
            _flagImgIn = PIXEL_FLOAT;
            _prtImgInFloat.open(getName("i:img"));
        }

        _prtImgOut.open(getName("o:img"));
        _prtVctBlob.open(getName("o:vct"));

        return true;
    }

    virtual bool close() {
        if (_flagImgIn == PIXEL_RGB)
            _prtImgInRgb.close();
        else if (_flagImgIn == PIXEL_FLOAT)
            _prtImgInFloat.close();
        _prtImgOut.close();
        _prtVctBlob.close();
        return true;
    }

    virtual bool interruptModule() {
        if (_flagImgIn == PIXEL_RGB)
            _prtImgInRgb.interrupt();
        else if (_flagImgIn == PIXEL_FLOAT)
            _prtImgInFloat.interrupt();
        _prtImgOut.interrupt();
        _prtVctBlob.interrupt();
        return true;
    }

    virtual bool updateModule() {

        // time stamp
        _startTime = yarp::os::Time::now();

        yarp::sig::Image *yrpImgIn;
        if (_flagImgIn == PIXEL_RGB)
            yrpImgIn = _prtImgInRgb.read();
        else if (_flagImgIn == PIXEL_FLOAT)
            yrpImgIn = _prtImgInFloat.read();
        
        // image grabbed?
        if (yrpImgIn == NULL) 
            return true;

        // image size > 0?
        if (yrpImgIn->width() == 0 || yrpImgIn->height() == 0)
           return true;

        // size changed?
        if (_oldImgSize.width  != yrpImgIn->width() || 
            _oldImgSize.height != yrpImgIn->height())
            initImages(yrpImgIn->width(), yrpImgIn->height());
        
        // convert input image to grayscale
        if (_flagImgIn == PIXEL_RGB)
            cvCvtColor((IplImage*)yrpImgIn->getIplImage(), _ocvImgGray, CV_RGB2GRAY);
        else if (_flagImgIn == PIXEL_FLOAT)
            BlobFunctions::grayFromFloat((IplImage*)yrpImgIn->getIplImage(), _ocvImgGray, (float)_scale);

        // make black border on grayscale image
		// (prevents blobs from spanning whole image if touching image borders 
        // (cvBlobsLib bug?)
        BlobFunctions::makeBlackBorder(_ocvImgGray, 1);

        // copy gray image to output image (convert to RGB)
        cvCvtColor(_ocvImgGray, _ocvImgReturn, CV_GRAY2RGB);
        
        CBlobResult blobs;
		CBlob *blobTarget;

        // calculate and filter blobs on gray input image
        _clusterBlob.calculate_clusters(_ocvImgGray, &blobs);
        //cout << "Number of detected blobs: " << _clusterBlob.getNumBlobs() << endl;

        // draw blobs on gray output image
        _clusterBlob.draw_clusters(_ocvImgReturn, &blobs);

        // write output vectors (timeStamp, x,y) and (timeStamp, size)
        if (blobs.GetNumBlobs() > 0){
            VectorOf<double> &vct = _prtVctBlob.prepare();
            vct.clear();
            // for each blob
            for (int i = 0; i < blobs.GetNumBlobs(); i++){
                blobTarget = blobs.GetBlob(i);
                // construct vector according to specified blobVectorFormat
                int format;
                for (int j = 0; j < _blobVectorSize; j++){
                    format = _blobVectorFormat[j];
                    switch (format){
                        case VCTBLOB_EMPTY:
                            vct.push_back(0.0);
                            break;
                        case VCTBLOB_TIMESTAMP:
                            vct.push_back(_startTime);
                            break;
                        case VCTBLOB_POSX:
                            vct.push_back(floor(blobTarget->MinX() + (( blobTarget->MaxX() - blobTarget->MinX() ) / 2.0) - _ocvImgGray->width/2.0 + 0.5));
                            break;
                        case VCTBLOB_POSY:
                            vct.push_back(floor(blobTarget->MinY() + (( blobTarget->MaxY() - blobTarget->MinY() ) / 2.0) - _ocvImgGray->height/2.0 +0.5));
                            break;
                        case VCTBLOB_SIZE:
                            vct.push_back(blobTarget->Area());
                            break;
                        case VCTBLOB_PERIMETER:
                            vct.push_back(blobTarget->Perimeter());
                            break;
                        default:
                            break;
                    }
                }
	        } 
            _prtVctBlob.write();
        }

        // wrap and write images back to port
        ImageOf<PixelRgb>& yrpImgOut = _prtImgOut.prepare();
        yrpImgOut.wrapIplImage(_ocvImgReturn);
        _prtImgOut.write();
    
        // buffering old image size
        _oldImgSize.width  = _ocvImgReturn->width;
        _oldImgSize.height = _ocvImgReturn->height;
    
        // calculate timing
        _endTime = yarp::os::Time::now();
        _execTime = _endTime - _startTime;
        if (_execTime < _refreshTime)
            yarp::os::Time::delay(_refreshTime - _execTime);
        //else
            //cout << "refreshTime error: " << _execTime - _refreshTime << " seconds." << endl;

        return true;
    }

};


int main(int argc, char *argv[]) {
    
    Network yarp;
    VisualBlobsModule module;
    module.setName("/visual_blobs"); // set default name of module
    return module.runModule(argc,argv);
}


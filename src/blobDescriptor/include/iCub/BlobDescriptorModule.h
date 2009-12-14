/**
 * @ingroup icub_module
 *
 * \defgroup icub_blobDescriptor blobDescriptor
 *
 * Receive a raw image and a labelled image from an Object Segmentation
 * module, compute a list of affordance descriptors.
 *
 * \section lib_sec Libraries
 *
 * OpenCV, YARP.
 *
 * \section parameters_sec Parameters
 *
 * <b>Command-line Parameters</b>
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key
 * (e.g. \c --from file.ini). The value part can be changed to suit your needs; the default values are shown below.
 *
 * - \c from \c blobDescriptor.ini \n
 *   The configuration file
 *
 * - \c context \c blobDescriptor/conf \n
 *   The sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c blobDescriptor \n
 *   The name of the module (used to form the stem of module port names)
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file
 * (they can also be specified as command-line parameters if you so wish).
 * The value part can be changed to suit your needs; the default values are shown below.
 *
 * - \c conf_port \c /blobDescriptor/conf
 *   Complete configuration, message handling port name (optional)
 *
 * - \c raw_image_input_port \c /blobDescriptor/rawImg:i
 *   Complete raw image input port name
 *
 * - \c labeled_image_input_port \c /blobDescriptor/labeledImg:i
 *   Complete labeled image input port name
 *
 * - \c raw_image_output_port \c /blobDescriptor/rawImg:o
 *   Complete raw image output port name
 *
 * - \c view_image_output_port \c /blobDescriptor/viewImg:o
 *   Complete port name of output image, including overlay edges
 *
 * - \c aff_descriptor_output_port \c /blobDescriptor/affDescriptor:o
 *   Complete affordance object descriptor output port name
 *
 * - \c tracker_init_output_port \c /blobDescriptor/trackerInit:o
 *   Complete tracker initialization parameter output port name
 *
 * \section portsa_sec Ports Accessed
 *
 * - \c /edisonSegm/rawimg:o
 *   Raw image port previously created by a segmentation module.
 *
 * - \c /edisonSegm/labelimg:o
 *   Labeled image port previously created by a segmentation module.
 *
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /blobDescriptor/rawImg:i \n
 *    Raw image input port
 *
 *  - \c /blobDescriptor/labeledImg:i \n
 *    Labeled image input port
 *
 * <b>Output ports</b>
 *
 * - /blobDescriptor/rawImg:o
 *   Raw image output port
 *
 * - /blobDescriptor/viewImg:o
 *   Port to display output image, including overlay edges
 *
 * - /blobDescriptor/affDescriptor:o
 *   Affordance object descriptor output port
 *
 * - /blobDescriptor/trackerInit:o
 *   Tracker initialization parameter output port
 *
 * <b>Port Types<b>
 *
 * - BufferedPort<ImageOf<PixelRgb> >  _rawImgInputPort;
 * - BufferedPort<ImageOf<PixelInt> >  _labeledImgInputPort;
 * - BufferedPort<ImageOf<PixelRgb> >  _rawImgOutputPort;
 * - BufferedPort<ImageOf<PixelRgb> >  _viewImgOutputPort;
 * - BufferedPort<Bottle>              _affDescriptorOutputPort;
 * - BufferedPort<Bottle>              _trackerInitOutputPort;
 *
 * \section out_data_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c blobDescriptor.ini  in \c $ICUB_ROOT/app/blobDescriptor/conf
 *
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>blobDescriptor --name blobDescriptor --context blobDescriptor/conf --from blobDescriptor.ini </tt>
 *
 * \author Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>, Ivana Cingovska, Alexandre Bernardino
 *
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * This file can be edited at src/blobDescriptor/src/blobDescriptor.h
 *
 */

#ifndef __ICUB_BLOB_DESC_MODULE_H__
#define __ICUB_BLOB_DESC_MODULE_H__

/* YARP */
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
using namespace yarp::os;
#include <yarp/sig/all.h> // FIXME: only load necessary headers
using namespace yarp::sig;

/* iCub */
#include <iCub/BlobDescriptorSupport.h>

/* OpenCV */
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

/* system */
//#include <fstream>
//#include <iostream>
//#include <sstream>
#include <string>
#include <stdio.h>
//#include <vector>
using namespace std;

//#define MAX_ELEMS 100 /* maximum number of segmented elements (blobs) to process in a cycle */

class BlobDescriptorModule : public RFModule
{
	/* private class variables and module parameters */
	string                            _moduleName;
	string                            _robotName;
	string                            _robotPortName;
	string                            _rawImgInputPortName;
	string                            _labeledImgInputPortName;
    string                            _rawImgOutputPortName;
    string                            _viewImgOutputPortName;
	string                            _affDescriptorOutputPortName;
	string                            _trackerInitOutputPortName;
	string                            _handlerPortName;
	Port                              _handlerPort; /* a port to handle messages */
	BufferedPort<ImageOf<PixelRgb> >  _rawImgInputPort;
	BufferedPort<ImageOf<PixelInt> >  _labeledImgInputPort;
	BufferedPort<ImageOf<PixelRgb> >  _rawImgOutputPort;
	BufferedPort<ImageOf<PixelRgb> >  _viewImgOutputPort;
	BufferedPort<Bottle>              _affDescriptorOutputPort;
	BufferedPort<Bottle>              _trackerInitOutputPort;
	/* yarp image pointers to access image ports */
	ImageOf<PixelRgb>                *_yarpRawInputPtr;
	ImageOf<PixelInt>                *_yarpLabeledInputPtr;
	/* yarp internal image buffers */
	ImageOf<PixelRgb>				  _yarpRawImg;
	ImageOf<PixelRgb>                 _yarpViewImg;
	ImageOf<PixelRgb>				  _yarpHSVImg;
	ImageOf<PixelMono>				  _yarpHueImg;
	ImageOf<PixelInt>				  _yarpLabeledImg;
	ImageOf<PixelMono>                _yarpTempImg;

	Bottle                            _affDescriptor;
	Bottle                            _trackerInit;
	/* OpenCV images - not needed anymore */
	//IplImage                       *_opencvRawImg;
	//IplImage                       *_opencvLabeledImg32;
    //IplImage                       *_opencvLabeledImg8;
	int                               _w, _h;
	CvSize                            _sz;

	ObjectDescriptor                 *_objDescTable;
	int                               _numObjects;
    int                               _hist_size[2];
	float                             _h_ranges[2], _s_ranges[2], _v_ranges[2];
	
	/* other parameters that can be user-specified (besides port names) */
	int                               _minAreaThreshold; /* min. number of pixels allowed for foreground objects */
	int                               _maxObjects; /* maximum number of object to process */

public:
	virtual bool configure(ResourceFinder &rf); /* configure module parameters, return true if successful */
	virtual bool interruptModule();             /* interrupt, e.g., ports */
	virtual bool close();                       /* close and shut down module */
	virtual bool respond(const Bottle &command, Bottle &reply);
	virtual bool updateModule();
};

#endif // __ICUB_BLOB_DESC_MODULE_H__

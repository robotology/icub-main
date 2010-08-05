/** 
 * @ingroup icub_module
 *
 * \defgroup icub_zdfModule zdfMod
 *
 * Receives the left and right images from the robot and segments objects that are located in the fovea. Performs marker-less pixel-wise segmentation of an object located in the fovea.The output is an image of the segmented object in grayscale and a difference of gausian segmentation.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * IPP
 * OPENCV
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters <\b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c zdfMod.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c zdfMod/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c zdfMod \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * <b>Configuration File Parameters </b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c inPortLeft \c /imageLeft:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inPortRight \c /imageRight:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortProb \c /imageProb:o \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortSeg \c /imageSeg:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortDog \c /imageDog:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter

 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /zdfMod \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /zdfMod
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /zdfMod/imageLeft:i \n
 *  - \c /zdfMod/imageRight:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /zdfMod \n
 *    see above
 *
 *  - \c /zdfMod/imageProb:o \n
 *  - \c /zdfMod/imageSeg:o \n
 *  - \c /zdfMod/imageDog:o \n
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortLeft; \n 
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortRight; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortProb; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortSeg;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortDog;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c zdfMod.ini  in \c $ICUB_ROOT/app/zdfMod/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>zdfMod --name zdfMod --context zdfMod/conf --from zdfMod.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff, Andrew Dankers
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src/yuvProc/include/iCub/yuvProc.h
 * 
 */

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __ICUB_ZDFMODULE_H__
#define __ICUB_ZDFMODULE_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
//ipp includes
#include <ipp.h>
#include <stdlib.h>
#include <string>

#include "iCub/coord.h"
#include "iCub/dog.h"
#include "iCub/multiclass.h"

#include "yarp/os/Time.h"

//use NDT or RANK comparision?
#define RANK0_NDT1 1 //0 (NDT FASTER and RANK a little too sensitive)
//NDT:
#define NDTX     1
#define NDTY     1
#define NDTSIZE  4 //4 or 8: 4
//RANK:
#define RANKY    1 //1 or 2: 1
#define RANKX    1  //1 or 2: 1
#define RANKSIZE 9  //9 or 25: 9

#define NDTEQ    0 //0

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
  
class ZDFThread : public Thread
{
private:

    /*port name strings*/
    string inputNameLeft;
	string inputNameRight;
    string outputNameProb;
	string outputNameSeg;  
	string outputNameDog;      

    /* class variables */
    ImageOf<PixelBgr>  *img_in_left;
	ImageOf<PixelBgr>  *img_in_right;
    ImageOf<PixelMono> *img_out_prob;    
	ImageOf<PixelMono> *img_out_seg;
	ImageOf<PixelMono> *img_out_dog;

    /* thread parameters: they are pointers so that they refer to the original variables */

    BufferedPort<ImageOf<PixelBgr> >  imageInLeft;
	BufferedPort<ImageOf<PixelBgr> >  imageInRight;
    BufferedPort<ImageOf<PixelMono> >  imageOutProb;
	BufferedPort<ImageOf<PixelMono> >  imageOutSeg;
	BufferedPort<ImageOf<PixelMono> >  imageOutDog;

	bool leftPort, rightPort; 
	bool init;
	int psb_in, t_lock_lr, t_lock_ud;
	//Sizes:
	IppiSize srcsize, msize, tsize, tisize, trsize;
	//Vars:
	int sx,sy;
  	Ipp32f max_v, max_t;
  	int mid_x, mid_y, mid_x_m, mid_y_m, area;
  	double cog_x, cog_y, cog_x_send, cog_y_send, spread, cmp_res;
  	int koffsetx, koffsety;
    int ndt1[NDTSIZE];
  	int ndt2[NDTSIZE];
  	int rank1[RANKSIZE];
  	int rank2[RANKSIZE];

	Coord c;

	int psb_m,psb_t,psb_rest,psb_resv;
	int nclasses, dpix_y;

	Ipp32f *res_t; 
  	Ipp8u *out, *seg_im, *seg_dog, *fov_l, *fov_r, *zd_prob_8u, *o_prob_8u;
	Ipp8u **p_prob;
  	//templates:
  	Ipp8u *temp_l, *temp_r;

  	//input:
  	Ipp8u *rec_im_ly;
  	Ipp8u *rec_im_ry;

	//DoG:
	DoG * dl;
  	DoG * dr;
	//Multiclass

    struct MultiClass::Parameters *params;
	MultiClass * m;

	int tl_x, tl_y;
  	int tr_x, tr_y;
  	int waiting;
  	bool update, acquire;
  	int rad_pen,max_rad_pen;
  	double r, rmax, r_deg, l_deg, t_deg, posx, posy, posz, z_;
	
	int width, height;
	double scale;

	//test
	Ipp8u *l_orig, *r_orig;
	int psb;
	IppiRect inroi;
	IppiSize insize;
    int BufferSize;
    
    //string containing module name
    string moduleName;

public:

    /* class methods */
    ZDFThread( MultiClass::Parameters *parameters );
    
    void initAll();
    bool threadInit();     
    void threadRelease();
    void run(); 
	
	void get_rank(Coord c,Ipp8u *im, int w, int*list);
	double cmp_rank(int*l1, int*l2);
	void get_ndt(Coord c,Ipp8u *im, int w, int*list);
	double cmp_ndt(int*l1, int*l2);
	void getAreaCoGSpread(Ipp8u*im, int p,IppiSize s, int*parea,double*pdx,double*pdy,double*pspread);
    void setName(string module);
};

class zdfMod:public RFModule
{
    /* module parameters */
    string moduleName; 
    string handlerName;
	
    Port handlerPort;      //a port to handle messages 
    
    struct MultiClass::Parameters parameters;
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    ZDFThread *zdfThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy

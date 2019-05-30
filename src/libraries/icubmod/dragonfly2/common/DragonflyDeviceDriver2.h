// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __DragonflyDeviceDriver2h__
#define __DragonflyDeviceDriver2h__

//=============================================================================
// YARP Includes
//=============================================================================

// May 06, readapted for YARP2 by nat

#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/IVisualParams.h>

namespace yarp {
    namespace dev {
//         class Dragonfly2OpenParameters;
        class DragonflyDeviceDriver2;
        class DragonflyDeviceDriver2Rgb;
        class DragonflyDeviceDriver2Raw;
    }
}

/**
* \file FirewireCamera.h device driver for managing the 
* IEEE-1394-DR2 Camera
*/

/**
*
@ingroup icub_hardware_modules
\defgroup icub_dragonfly2 dragonfly2
The dragonfly2 framegrabber device driver can acquire RGB color images in 320x240 or 640x480 resolutions.
The dragonfly2raw framegrabber device driver can acquire raw format images in 640x480 resolution.

\section intro_sec Description
The dragonfly framegrabber device driver is based on libdc1394-2.

It can acquire RGB color images in 320x240 or 640x480 resolutions.
In 640x480 there are two options: Bayer decoding performed on board by the camera or Bayer pattern decoding performed by the driver. In the second mode the bandwidth required to the Firewire bus is lower, and thus the framerate can be up to 60 fps. In the first mode the framerate is limited to 15 fps with two cameras on the same channel. Moreover, once the resolution is chosen,
the image can be cropped on board by the camera before the transfer on the Firewire bus.

These functionalities are made available thought two YARP devices: dragonfly2 (for RGB images) and dragondly2raw (for raw images with Bayer encoding).

Runtime parameters can be changed using the graphical interface: \ref icub_framegrabbergui2.

\section lib_sec Libraries
libdc1394-2 (Linux)
PGRFlyCapture (Windows)

\section parameters_sec Parameters

--name             // image output port

--video_type       // 1: RGB 320x240, 2: RGB 640x480, 3: RGB 640x480 (default) software Bayer decoding (higher fps)

--width|size_x     // width image cropping (limited to image size)

--height|size_y    // height image cropping (limited to image size)

--port_number      // Firewire channel (need to specify only if you have more than one firewire card)

--guid             // camera global unique 64 bit identifier <B>WARNING: replaces --d</B>

--d                // camera unit number <B>DEPRECATED</B>

--white_balance    // red and blue balance, values are normalized betwen 0.0 and 1.0

--feature          // camera feature setting, normalized between 0.0 and 1.0 (features listed below)


\subsection video_type The video_type parameter

The video_type parameter determines how images are acquired by the dragonfly chip.

-video_type 1: the image is acquired by the Dragonfly2 camera in 320x240 resolution as RGB color image, and transferred to the framegrabber driver memory buffer in this format. The Firewire bandwidth allows maximum framerate (60 fps) with two cameras. If specified, the --width and --height parameters will '''crop''' the image to the specified dimension.

-video_type 2: the image is acquired by the Dragonfly2 camera in 640x480 resolution as RGB color image, and transferred to the framegrabber driver memory buffer in this format. The Firewire bandwidth allows 15 fps with two cameras. If specified, the --width and --height parameters will '''crop''' the image to the specified dimension.

-video_type 3: the image is acquired as a raw Bayer pattern 640x480 image, and transferred in this format to the framegrabber driver memory buffer. In this way, the bandwidth required to the Firewire bus is lower than in the previous format, allowing 60 fps. The Bayer decoding to the usual RGB format provided by the DragonflyDeviceDriver2 framegrabber is performed at software level by the driver itself. If specified, the --width and --height parameters will '''crop''' the original 640 x 480 image to the specified dimension.

\subsection port_units Port, Unit number and 64 bit Global Unique Identifier

Many cameras can coexist on the same Firewire bus (port), sharing the available bandwidth. Each camera connected to the same Firewire bus is associated to a unit number. Port numbers, as well as unit numbers, are assigned increasingly starting from 0.
The iCub robot has one only Firewire card, and thus its port number is always 0 (can be omitted). The two left and right cameras will be unit 0 and 1, but unfortunately this association is not deterministic, and thus the Global Unique Identifier (guid) must be used in order to univokely identify a camera.
The yarpdev device driver supports configuration files, thus the most convenient way to deal with GUIDs is putting them in .ini files that will be passed to yarpdev together with all the other parameters as follows:

yarpdev --from camera/dragonfly2_config_left.ini
yarpdev --from camera/dragonfly2_config_right.ini

In the latest iCub software releases the .ini files are already supplied in <B>app/robots/iCubXXXnn/camera</B> folders. The --d option must be replaced in them by assigning the guid of the corresponding camera to the guid parameter.
The following configuration file dragonfly2_config_left.ini

device grabber \n
subdevice dragonfly2 \n
width 320 \n
height 240 \n
video_type 1 \n
white_balance 0.506 0.494 \n
gain 0.312 \n
shutter 0.913 \n
name /icub/cam/left \n
brightness 0 \n
DR2 \n
stamp \n
sharpness 0.5 \n
hue 0.48 \n
gamma 0.4 \n
saturation 0.271 \n
framerate 30 \n
d 0 \n
#guid <64bit global identifier, without the leading 0x> then remove the d option \n

will become after configuration, if (for example) A0BB98F34560AAB5 is the guid of the left camera:

device grabber \n
subdevice dragonfly2 \n
width 320 \n
height 240 \n
video_type 1 \n
white_balance 0.506 0.494 \n
gain 0.312 \n
shutter 0.913 \n
name /icub/cam/left \n
brightness 0 \n
DR2 \n
stamp \n
sharpness 0.5 \n
hue 0.48 \n
gamma 0.4 \n
saturation 0.271 \n
framerate 30 \n
guid  A0BB98F34560AAB5 \n

The configuration files already present in the /app/robots/iCubXXXnn/camera folder are:

dragonfly_config_left.ini \n
dragonfly_config_right.ini \n
dragonfly_config_left_bayer_320_240.ini \n
dragonfly_config_right_bayer_320_240.ini \n
dragonfly_config_left_bayer_640_480.ini \n
dragonfly_config_right_bayer_640_480.ini \n

\subsection how_to_guid How to obtain GUID from a camera

Execute from terminal:

yarpdev --device grabber --subdevice dragonfly 2 --name /foocam0 --d 0

The driver will answer something like:

------ %Camera information ------ \n
Vendor                            :     %Point Grey Research \n
%Model                             :     Dragonfly2 DR2-03S2C-EX \n
Unit                              :     0 \n
Specifications ID                 :     0xa02d \n
Software revision                 :     0x102 \n
IIDC version code                 :     548 \n
Unit directory offset             :     0x428 \n
Unit dependent directory offset   :     0x440 \n
Commands registers base           :     0xf00000 \n
Unique ID                         :     0x00b09d01006fb1fb  <B><== THIS IS THE INFORMATION THAT YOU NEED!!!</B> \n
Vendor ID                         :     0xb09d \n
%Model ID                          :     0x1 \n
Advanced features found at offset :     0xf01000 \n
1394b mode capable (>=800Mbit/s)  :     Yes \n
Platform backend                  :     juju \n
------ %Camera platform-specific information ------ \n
Device filename                   :     /dev/fw2 \n

Now, from another terminal, execute

yarpview --name /fooview0 \n
yarp connect /foocam0 /fooview0 \n

and check in the viewer if the camera is the left or right iCub eye. Supposing that it is the left one, write the guid parameter in the dragonfly2_config_left*.ini files as shown in the former section.
Do the same for the other camera, using unit number 1:

yarpdev --device grabber --subdevice dragonfly 2 --name /foocam1 --d 1 \n
yarpview --name /fooview1 \n
yarp connect /foocam1 /fooview1 \n

and set the corresponding guid in the other camera's .ini files, <B>without leading 0x</B>.


\subsection features Features

- brightness
- exposure
- sharpness
- hue
- saturation
- gamma
- shutter
- gain
- iris

Please notice that ''exposure'' is an auto mode, if set it controls ''gain'' and ''shutter'' parameters in order to match the desired ''exposure''.

\section portsa_sec Ports Accessed
None.

\section portsc_sec Ports Created

The dragonfly device driver is usually executed combined with a network wrapper grabber (called \a grabber, see examples below) which reads images from the driver and streams them on the network. The grabber opens the followig ports:

Output ports:
- <as specified by --name> streams out a yarp::sig::ImageOf<yarp::sig::PixelRgb> which contains the image grabbed by the Dragonfly camera.

Input ports:
- <as specified by --name> input port to control the camera features and acquisition modes, accept a yarp::os::Bottle. The FrameGrabberGui module connects to this port.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
Usage syntax:

yarpdev --device grabber --subdevice dragonfly2 --name <yarp port name>
[--video_type <type>] [--width|size_x <W> --height|size_y <H>] [--port_number <pn>]
[--guid <64_bit_camera_unique_identifier>] [--white_balance <red_value> <blue_value>] [--feature <parameter>] [...]

yarpdev --device grabber --subdevice dragonfly2raw --name <yarp port name>
[--width|size_x <W> --height|size_y <H>] [--port_number <pn>]
[--guid <64_bit_global_unique_identifier>] [--white_balance <red_value> <blue_value>] [--feature <parameter>] [...]

<B>WARNING: the old --d <unit_number> parameter is still working but deprecated, please use --guid <64_bit_global_unique_identifier> instead,
because --d <unit_number> camera has become non deterministic in left/right assignment in latest linux releases.</B>

Example:

yarpdev --device grabber --subdevice dragonfly2 --name /icub/cam/left  --guid AB10980D6656E455 [...]

yarpdev --device grabber --subdevice dragonfly2 --name /icub/cam/right --guid 98FF0666E478A001 [...]

yarpdev --device grabber --subdevice dragonfly2raw --name /icub/cam/left  --guid AB10980D6656E455 [...]

yarpdev --device grabber --subdevice dragonfly2raw --name /icub/cam/right --guid 98FF0666E478A001 [...]

<B>DEPRECATED:</B>

yarpdev --device grabber --subdevice dragonfly2 --name /icub/cam/right --d 1|0 [...]

yarpdev --device grabber --subdevice dragonfly2 --name /icub/cam/left  --d 0|1 [...]

yarpdev --device grabber --subdevice dragonfly2raw --name /icub/cam/left  --d 0|1 [...]

yarpdev --device grabber --subdevice dragonfly2raw --name /icub/cam/right --d 1|0 [...]




\author Paul Fitzpatrick, Lorenzo Natale, Giorgio Metta, Alessandro Scalzo

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at /src/libraries/icubmod/dragonfly2/common/DragonflyDeviceDriver2.h
**/

/**
* dragonfly2 and dragonfly2raw device driver implementation.
*/
class yarp::dev::DragonflyDeviceDriver2 :
    public DeviceDriver,
    public IPreciselyTimed,
    public IFrameGrabber,
    public IFrameGrabberRgb,
    public IFrameGrabberControls2,
    public IFrameGrabberControlsDC1394,
    public IRgbVisualParams
{
private:
    DragonflyDeviceDriver2(const DragonflyDeviceDriver2&);
    void operator=(const DragonflyDeviceDriver2&);

public:
    /**
    * Constructor.
    */
    DragonflyDeviceDriver2(bool raw = false);

    /**
    * Destructor.
    */
    virtual ~DragonflyDeviceDriver2();

    /**
    * Open the device driver.
    * @param config configuration for the device driver
    * @return returns true on success, false on failure.
    */
    virtual bool open(yarp::os::Searchable& config);

    /**
    * Closes the device driver.
    * @return returns true/false on success/failure.
    */
    virtual bool close(void);

    /**
    * Implements FrameGrabber basic interface.
    * @param buffer the pointer to the array to store the last frame.
    * @return returns true/false on success/failure.
    */
    virtual bool getRawBuffer(unsigned char *buffer);

    /**
    * Implements the Frame grabber basic interface.
    * @return the size of the raw buffer (for the Dragonfly
    * camera this is 1x640x480).
    */
    virtual int getRawBufferSize();

    /**
    * Implements FrameGrabber basic interface.
    */
    virtual int height() const;

    /**
    * Implements FrameGrabber basic interface.
    */
    virtual int width() const;

    /** 
    * FrameGrabber bgr interface, returns the last acquired frame as
    * a buffer of bgr triplets. A demosaicking method is applied to 
    * reconstuct the color from the Bayer pattern of the sensor.
    * @param buffer pointer to the array that will contain the last frame.
    * @return true/false upon success/failure
    */
    virtual bool getRgbBuffer(unsigned char *buffer);

    /** 
    * Implements the IPreciselyTimed interface.
    * @return the yarp::os::Stamp of the last image acquired
    */
    virtual yarp::os::Stamp getLastInputStamp();

    /**
    * Set Brightness.
    * @param v normalized image brightness [0.0 : 1.0]. 
    * @return true/false upon success/failure
    */
    virtual bool setBrightness(double v);
    /**
    * Set Exposure.
    * @param v normalized image exposure [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setExposure(double v);
    /**
    * Set Sharpness. 
    * @param v normalized image sharpness [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setSharpness(double v);
    /**
    * Set White Balance.
    * @param blue normalized image blue balance [0.0 : 1.0].
    * @param red normalized image red balance [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setWhiteBalance(double blue, double red);
    /**
    * Set Hue.
    * @param v normalized image hue [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setHue(double v);
    /**
    * Set Saturation.
    * @param v normalized image saturation [0.0 : 1.0]. 
    * @return true/false upon success/failure
    */
    virtual bool setSaturation(double v);
    /**
    * Set Gamma.
    * @param v normalized image gamma [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setGamma(double v);		
    /**
    * Set Shutter.
    * @param v normalized camera shutter time [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setShutter(double v);
    /**
    * Set Gain. 
    * @param v normalized camera gain [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setGain(double v);
    /**
    * Set Iris. 
    * @param v normalized camera iris [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setIris(double v);
    //virtual bool setTemperature(double v);
    //virtual bool setWhiteShading(double r,double g,double b);
    //virtual bool setOpticalFilter(double v);
    //virtual bool setCaptureQuality(double v);

    /**
    * Get Brightness.
    * @return normalized image brightness [0.0 : 1.0].
    */
    virtual double getBrightness();
    /**
    * Get Exposure.
    * @return normalized image exposure [0.0 : 1.0].
    */
    virtual double getExposure();    
    /**
    * Get Sharpness.
    * @return normalized image sharpness [0.0 : 1.0].
    */
    virtual double getSharpness();    
    /**
    * Get White Balance.
    * @param blue normalized blue balance [0.0 : 1.0].
    * @param red normalized red balance [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool getWhiteBalance(double &blue, double &red);
    /**
    * Get Hue.
    * @return normalized hue [0.0 : 1.0].
    */
    virtual double getHue();
    /**
    * Get Saturation.
    * @return normalized saturation [0.0 : 1.0].
    */
    virtual double getSaturation();
    /**
    * Get Gamma.
    * @return normalized gamma [0.0 : 1.0].
    */
    virtual double getGamma();
    /**
    * Get Shutter.
    * @return normalized shutter time [0.0 : 1.0].
    */
    virtual double getShutter();
    /**
    * Get Gain.
    * @return normalized gain [0.0 : 1.0].
    */
    virtual double getGain();
    /**
    * Get Iris.
    * @return normalized iris [0.0 : 1.0].
    */
    virtual double getIris();


    //virtual double getTemperature() const;
    //virtual double getWhiteShading() const;
    //virtual double getOpticalFilter() const;
    //virtual double getCaptureQuality() const;

    //virtual bool setAutoBrightness(bool bAuto=true);

    //virtual bool setAutoGain(bool bAuto=true);

    //virtual bool setAutoShutter(bool bAuto=true);

    //virtual bool setAutoWhiteBalance(bool bAuto=true);

    //virtual bool setAuto(bool bAuto=true);

    //virtual void PrintSettings();

    /////////
    // DC1394
    /////////

    /**
    * Is feature present?
    * @param feature feature ID.
    * @return true=present, false=not present.
    */
    virtual bool hasFeatureDC1394(int feature);
    /**
    * Set feature value.
    * @param feature feature ID
    * @param value normalized feature value [0.0 : 1.0].
    * @return true/false upon success/failure.
    */
    virtual bool setFeatureDC1394(int feature, double value);
    /**
    * Get feature value.
    * @param feature feature ID
    * @return normalized feature value [0.0 : 1.0].
    */	
    virtual double getFeatureDC1394(int feature);

    /**
    * Can be feature turned on/off?
    * @param feature feature ID.
    * @return true=YES, false=NO.
    */
    virtual bool hasOnOffDC1394(int feature);
    /**
    * Switch feature on/off.
    * @param feature feature ID.
    * @param onoff true=ON false=OFF.
    * @return true/false upon success/failure.
    */
    virtual bool setActiveDC1394(int feature, bool onoff);
    /**
    * Is feature ON or OFF?
    * @param feature feature ID.
    * @return true=ON false=OFF.
    */
    virtual bool getActiveDC1394(int feature);

    /**
    * Has feature Auto mode?
    * @param feature feature ID.
    * @return true=YES, false=NO.
    */
    virtual bool hasAutoDC1394(int feature);
    /**
    * Has feature Manual mode?
    * @param feature feature ID.
    * @return true=YES, false=NO.
    */	
    virtual bool hasManualDC1394(int feature);
    /**
    * Has feature Manual mode?
    * @param feature feature ID.
    * @return true=YES, false=NO.
    */
    virtual bool hasOnePushDC1394(int feature);
    /**
    * Switch feature Auto/Manual.
    * @param feature feature ID.
    * @param onoff true=Auto false=Manual.
    * @return true/false upon success/failure.
    */
    virtual bool setModeDC1394(int feature, bool auto_onoff);
    /**
    * Is feature mode Auto or Manual?
    * @param feature feature ID.
    * @return true=Auto false=Manual.
    */
    virtual bool getModeDC1394(int feature);
    /**
    * Trigger feature One Push adjust.
    * @param feature feature ID.
    * @return true/false upon success/failure.
    */
    virtual bool setOnePushDC1394(int feature);

    /**
    * Get supported video mode mask.
    * The video mode bitmask is obtained as follows:
    *
    * 	unsigned int mask=0;
    *
    *	for (unsigned int m=0; m<modes.num; ++m)
    *
    *			mask|=1<<(modes.modes[m]-DC1394_VIDEO_MODE_MIN);
    *
    * 0=160x120 YUV444, 1=320x240 YUV422, 2=640x480 YUV411, 3=640x480 YUV422, 4=640x480 RGB8, 5=640x480 MONO8,
    * 6=640x480 MONO16,7=800x600 YUV422, 8=800x600 RGB8, 9=800x600_MONO8, 10=1024x768 YUV422, 11=1024x768 RGB8, 12=1024x768 MONO8, 
    * 13=800x600 MONO16, 14=1024x768 MONO16, 15=1280x960 YUV422, 16=1280x960 RGB8, 17=1280x960_MONO8, 18=1600x1200 YUV422, 19=1600x1200 RGB8,
    * 20=1600x1200 MONO8, 21=1280x960 MONO16, 22=1600x1200_MONO16, 23=EXIF, 24=FORMAT7 0, 25=FORMAT7 1, 26=FORMAT7 2, 27=FORMAT7 3,
    * 28=FORMAT7 4, 29=FORMAT7 5, 30=FORMAT7 6, 31=FORMAT7 7
    * @return video mode bitmask.
    */
    virtual unsigned int getVideoModeMaskDC1394();
    /**
    * Get camera acquisition mode.
    * @return video mode ID: 0=160x120 YUV444, 1=320x240 YUV422, 2=640x480 YUV411, 3=640x480 YUV422, 4=640x480 RGB8, 5=640x480 MONO8,
    * 6=640x480 MONO16,7=800x600 YUV422, 8=800x600 RGB8, 9=800x600_MONO8, 10=1024x768 YUV422, 11=1024x768 RGB8, 12=1024x768 MONO8, 
    * 13=800x600 MONO16, 14=1024x768 MONO16, 15=1280x960 YUV422, 16=1280x960 RGB8, 17=1280x960_MONO8, 18=1600x1200 YUV422, 19=1600x1200 RGB8,
    * 20=1600x1200 MONO8, 21=1280x960 MONO16, 22=1600x1200_MONO16, 23=EXIF, 24=FORMAT7 0, 25=FORMAT7 1, 26=FORMAT7 2, 27=FORMAT7 3,
    * 28=FORMAT7 4, 29=FORMAT7 5, 30=FORMAT7 6, 31=FORMAT7 7
    */
    virtual unsigned int getVideoModeDC1394();
    /**
    * Set camera acquisition mode.
    * @param video_mode ID: 0=160x120 YUV444, 1=320x240 YUV422, 2=640x480 YUV411, 3=640x480 YUV422, 4=640x480 RGB8, 5=640x480 MONO8,
    * 6=640x480 MONO16,7=800x600 YUV422, 8=800x600 RGB8, 9=800x600_MONO8, 10=1024x768 YUV422, 11=1024x768 RGB8, 12=1024x768 MONO8, 
    * 13=800x600 MONO16, 14=1024x768 MONO16, 15=1280x960 YUV422, 16=1280x960 RGB8, 17=1280x960_MONO8, 18=1600x1200 YUV422, 19=1600x1200 RGB8,
    * 20=1600x1200 MONO8, 21=1280x960 MONO16, 22=1600x1200_MONO16, 23=EXIF, 24=FORMAT7 0, 25=FORMAT7 1, 26=FORMAT7 2, 27=FORMAT7 3,
    * 28=FORMAT7 4, 29=FORMAT7 5, 30=FORMAT7 6, 31=FORMAT7 7
    * @return true/false upon success/failure.
    */
    virtual bool setVideoModeDC1394(int video_mode);

    /**
    * Get supported framerates mask.
    * The framerates bitmask is obtained as follows:
    *
    *	unsigned int mask=0;
    *
    *	for (unsigned int f=0; f<fps.num; ++f)
    *
    *		mask|=1<<(fps.framerates[f]-DC1394_FRAMERATE_MIN);
    *
    * 0=1.875 fps, 1=3.75 fps, 2=7.5 fps, 3=15 fps, 4=30 fps, 5=60 fps, 6=120 fps, 7=240 fps.
    * @return framerates bitmask.
    */
    virtual unsigned int getFPSMaskDC1394();

    /**
    * Get camera framerate.
    * @return framerate mode ID: 0=1.875 fps, 1=3.75 fps, 2=7.5 fps, 3=15 fps, 4=30 fps, 5=60 fps, 6=120 fps, 7=240 fps.
    */
    virtual unsigned int getFPSDC1394();
    /**
    * Set camera framerate.
    * @param fps framerate ID: 0=1.875 fps, 1=3.75 fps, 2=7.5 fps, 3=15 fps, 4=30 fps, 5=60 fps, 6=120 fps, 7=240 fps.
    * @return true/false upon success/failure.
    */
    virtual bool setFPSDC1394(int fps);

    /**
    * Get camera Firewire ISO speed.
    * @return ISO speed ID: 0=100 Mbps, 1=200 Mbps, 2=400 Mbps, 3=800 Mbps, 4=1600 Mbps, 5=3200 Mbps.
    */
    virtual unsigned int getISOSpeedDC1394();
    /**
    * Set camera Firewire ISO speed.
    * @param speed ISO speed ID: 0=100 Mbps, 1=200 Mbps, 2=400 Mbps, 3=800 Mbps, 4=1600 Mbps, 5=3200 Mbps.
    * @return true/false upon success/failure.
    */
    virtual bool setISOSpeedDC1394(int speed);

    /**
    * Get supported color coding mask.
    * The framerates bitmask is obtained as follows:
    *
    *	unsigned int mask=0;
    *
    *	for (unsigned int m=0; m<codings.num; ++m)
    *
    *			mask|=1<<(codings.codings[m]-DC1394_COLOR_CODING_MIN);
    *
    * 0=MONO8, 1=YUV411, 2=YUV422, 3=YUV444, 4=RGB8, 5=MONO16, 6=RGB16, 7=MONO16S, 8=RGB16S, 9=RAW8, 10=RAW16.
    * @return framerates bitmask.
    */
    virtual unsigned int getColorCodingMaskDC1394(unsigned int video_mode);
    /**
    * Get image color coding.
    * @return image color coding ID: 0=MONO8, 1=YUV411, 2=YUV422, 3=YUV444, 4=RGB8, 5=MONO16, 6=RGB16, 7=MONO16S, 8=RGB16S, 9=RAW8, 10=RAW16.
    */
    virtual unsigned int getColorCodingDC1394();
    /**
    * Set image color coding.
    * @param coding image color coding ID: 0=MONO8, 1=YUV411, 2=YUV422, 3=YUV444, 4=RGB8, 5=MONO16, 6=RGB16, 7=MONO16S, 8=RGB16S, 9=RAW8, 10=RAW16.
    * @return true/false upon success/failure.
    */
    virtual bool setColorCodingDC1394(int coding);

    /**
    * Set White Balance.
    * @param blue normalized image blue balance [0.0 : 1.0].
    * @param red normalized image red balance [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool setWhiteBalanceDC1394(double b, double r);
    /**
    * Get White Balance.
    * @param blue normalized blue balance [0.0 : 1.0].
    * @param red normalized red balance [0.0 : 1.0].
    * @return true/false upon success/failure
    */
    virtual bool getWhiteBalanceDC1394(double &b, double &r);

    /** 
    * Get maximum image dimensions in Format 7 mode.
    * @param xdim maximum width
    * @param ydim maximum height
    * @param xstep width granularity
    * @param ystep height granularity
    * @param xoffstep horizontal offset granularity
    * @param yoffstep vertical offset granularity
    * @return true/false upon success/failure
    */
    virtual bool getFormat7MaxWindowDC1394(unsigned int &xdim,unsigned int &ydim,unsigned int &xstep,unsigned int &ystep,unsigned int &xoffstep,unsigned int &yoffstep);

    /** 
    * Get image dimensions in Format 7 mode.
    * @param xdim image width
    * @param ydim image height
    * @param x0 horizontal image offset
    * @param y0 vertical image offset
    * @return true/false upon success/failure
    */
    virtual bool getFormat7WindowDC1394(unsigned int &xdim,unsigned int &ydim,int &x0,int &y0);
    /** 
    * Set image dimensions in Format 7 mode.
    * @param xdim image width
    * @param ydim image height
    * @param x0 horizontal image offset
    * @param y0 vertical image offset
    * @return true/false upon success/failure
    */
    virtual bool setFormat7WindowDC1394(unsigned int xdim,unsigned int ydim,int x0,int y0);

    /**
    * Set Operation Mode.
    * @param b1394b true=1394b false=LEGACY
    * @return true/false upon success/failure
    */
    virtual bool setOperationModeDC1394(bool b1394b);
    /**
    * Get Operation Mode.
    * @return true=1394b false=LEGACY
    */
    virtual bool getOperationModeDC1394(); 

    /**
    * Set image transmission ON/OFF.
    * @param bTxON true=ON false=OFF
    * @return true/false upon success/failure
    */
    virtual bool setTransmissionDC1394(bool bTxON);
    /**
    * Is image transmission ON or OFF?
    * @return true=ON false=OFF
    */
    virtual bool getTransmissionDC1394();
    //virtual bool setBayerDC1394(bool bON);
    //virtual bool getBayerDC1394();

    /**
    * Set feature commands propagation ON/OFF.
    * All the cameras on the same Firewire bus can be adjusted at once setting broadcast ON. In this way, they will share the feature settings. 
    * @param onoff true=ON false=OFF
    * @return true/false upon success/failure
    */
    virtual bool setBroadcastDC1394(bool onoff);
    /**
    * Set camera features to default.
    * @return true/false upon success/failure
    */
    virtual bool setDefaultsDC1394();
    /*
    * Reset camera.
    * @return true/false upon success/failure
    */
    virtual bool setResetDC1394();
    /**
    * Switch camera power ON/OFF.
    * @param onoff true=ON false=OFF
    * @return true/false upon success/failure
    */
    virtual bool setPowerDC1394(bool onoff);
    /**
    * Switch image capture ON/OFF.
    * @param onoff true=ON false=OFF
    * @return true/false upon success/failure
    */
    virtual bool setCaptureDC1394(bool bON);

    /** 
    * Get Firewire communication packet size.
    * In Format7 mode the framerate depends from packet size.
    * @return bytes per packet
    */
    virtual unsigned int getBytesPerPacketDC1394();
    /** 
    * Set Firewire communication packet size.
    * In Format7 mode the framerate depends from packet size.
    * @param bpp bytes per packet
    * @return true/false upon success/failure
    */
    virtual bool setBytesPerPacketDC1394(unsigned int bpp);

    //IVisualParams

    /**
     * Return the height of each frame.
     * @return rgb image height
     */
    virtual int getRgbHeight();

    /**
     * Return the width of each frame.
     * @return rgb image width
     */
    virtual int getRgbWidth();
    /**
     * Get the possible configurations of the camera
     * @param configurations  list of camera supported configurations as CameraConfig type
     * @return true on success
     */

    virtual bool getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig> &configurations);
    /**
     * Get the resolution of the rgb image from the camera
     * @param width  image width
     * @param height image height
     * @return true on success
     */

    virtual bool getRgbResolution(int &width, int &height);
    /**
     * Set the resolution of the rgb image from the camera
     * @param width  image width
     * @param height image height
     * @return true on success
     */

    virtual bool setRgbResolution(int width, int height);

    /**
     * Get the field of view (FOV) of the rgb camera.
     *
     * @param  horizontalFov will return the value of the horizontal fov in degrees
     * @param  verticalFov   will return the value of the vertical fov in degrees
     * @return true on success
     */
    virtual bool getRgbFOV(double &horizontalFov, double &verticalFov);

    /**
     * Set the field of view (FOV) of the rgb camera.
     *
     * @param  horizontalFov will set the value of the horizontal fov in degrees
     * @param  verticalFov   will set the value of the vertical fov in degrees
     * @return true on success
     */
    virtual bool setRgbFOV(double horizontalFov, double verticalFov);

    /**
     * Get the intrinsic parameters of the rgb camera
     * @param  intrinsic  return a Property containing intrinsic parameters
     *       of the optical model of the camera.
     * @return true if success
     *
     * Look at IVisualParams.h for more details
     */
    virtual bool getRgbIntrinsicParam(yarp::os::Property &intrinsic);

    /**
     * Get the mirroring setting of the sensor
     *
     * @param mirror: true if image is mirrored, false otherwise
     * @return true if success
     */
    virtual bool getRgbMirroring(bool &mirror);

    /**
     * Set the mirroring setting of the sensor
     *
     * @param mirror: true if image should be mirrored, false otherwise
     * @return true if success
     */
    virtual bool setRgbMirroring(bool mirror);

    // Control2

    /* Implementation of IFrameGrabberControls2 interface */
    virtual bool getCameraDescription(CameraDescriptor *camera);
    virtual bool hasFeature(int feature, bool *hasFeature);
    virtual bool setFeature(int feature, double values);
    virtual bool getFeature(int feature, double *values);
    virtual bool setFeature(int feature, double  value1, double  value2);
    virtual bool getFeature(int feature, double *value1, double *value2);
    virtual bool hasOnOff(int feature, bool *HasOnOff);
    virtual bool setActive(int feature, bool onoff);
    virtual bool getActive(int feature, bool *isActive);
    virtual bool hasAuto(int feature, bool *hasAuto);
    virtual bool hasManual(int feature, bool *hasManual);
    virtual bool hasOnePush(int feature, bool *hasOnePush);
    virtual bool setMode(int feature, FeatureMode mode);
    virtual bool getMode(int feature, FeatureMode *mode);
    virtual bool setOnePush(int feature);

protected:
    void* system_resources;
    bool raw;
    FeatureMode TRANSL_MODE(bool mode) { return (mode ? MODE_AUTO : MODE_MANUAL); }
    bool TRANSL_MODE(FeatureMode mode) { return (mode == MODE_AUTO? 1 : 0); }
};

/**
* @ingroup icub_hardware_modules
* @brief `dragonfly2` : framegrabber device driver that can acquire RGB color images in 320x240 or 640x480 resolutions.
*
* See \ref dragonfly2 for for more details.
*
* | YARP device name |
* |:-----------------:|
* | `dragonfly2` |
*/
class yarp::dev::DragonflyDeviceDriver2Rgb : 
    public yarp::dev::DragonflyDeviceDriver2,
    public IFrameGrabberImage,
    public IFrameGrabberImageRaw
{
private:
    DragonflyDeviceDriver2Rgb(const DragonflyDeviceDriver2Rgb&);
    void operator=(const DragonflyDeviceDriver2Rgb&);

public:
    /**
    * Constructor.
    */
    DragonflyDeviceDriver2Rgb() : DragonflyDeviceDriver2(false){}

    /**
    * Destructor.
    */
    virtual ~DragonflyDeviceDriver2Rgb(){}

    /** 
    * FrameGrabber image interface, returns the last acquired frame as
    * an rgb image. A demosaicking method is applied to 
    * reconstuct the color from the Bayer pattern of the sensor.
    * @param image that will store the last frame.
    * @return true/false upon success/failure
    */
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    /**
        * FrameGrabber image interface, returns the last acquired frame as
        * a raw image.
        * @param image that will store the last frame.
        * @return true/false upon success/failure
        */
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image);

    /** 
     * Return the height of each frame.
     * @return image height
     */
    virtual int height() const;

    /** 
     * Return the width of each frame.
     * @return image width
     */
    virtual int width() const;
};

/**
* @ingroup icub_hardware_modules
* @brief `dragonfly2raw` : framegrabber device driver that can acquire raw format images in 640x480 resolution.
*
* See \ref dragonfly2 for for more details.
*
* | YARP device name |
* |:-----------------:|
* | `dragonfly2raw` |
*/
class yarp::dev::DragonflyDeviceDriver2Raw :
    public yarp::dev::DragonflyDeviceDriver2,
    public IFrameGrabberImageRaw
{
private:
    DragonflyDeviceDriver2Raw(const DragonflyDeviceDriver2Raw&);
    void operator=(const DragonflyDeviceDriver2Raw&);

public:
    /**
    * Constructor.
    */
    DragonflyDeviceDriver2Raw() : DragonflyDeviceDriver2(true){}

    /**
    * Destructor.
    */
    virtual ~DragonflyDeviceDriver2Raw(){}

    /** 
    * FrameGrabber image interface, returns the last acquired frame as
    * an rgb image. A demosaicking method is applied to 
    * reconstuct the color from the Bayer pattern of the sensor.
    * @param image that will store the last frame.
    * @return true/false upon success/failure
    */
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono>& image);

    /** 
     * Return the height of each frame.
     * @return image height
     */
    virtual int height() const;

    /** 
     * Return the width of each frame.
     * @return image width
     */
    virtual int width() const;
};
#endif

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   eric.sauser@a3.epfl.ch
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

/**
 *
@ingroup icub_lasaImitation_module
\defgroup icub__lasaImitation_MultipleMiceDriver Multiple Mice Driver

A device driver to get input simltaneously information form multiple mices.

\section intro_sec Description

\note Currently work only on Linux...

In order to disable some mice (e.g. touchpads you want to use) to be used by linux as a pointing mouse, and also to be able to read mice data from /dev/input/eventXX, the following should be done.

On Ubuntu >= 8.10 (>= Intrepid Ibex)

    - copy or create a fdi file into "hal" folder (see below for more details on th .fdi file)
          - sudo cp 10-usbglide.fdi /etc/hal/fdi/preprobe/
    - restart hal-deamon
          - sudo /etc/init.d/hal restart (or "sudo service hal restart" on Ubuntu >= Karmic Koala)
    - change the default permissions to /dev/input/eventXX
          - Using <sudo>, add the following line at the end of the file: /etc/udev/rules.d/40-permissions.rules
                + KERNEL=="event[0-9]*", GROUP="input", MODE="0660"
          - Or create this file for (Ubuntu >= Karmic Koala)
    - restart udev
          - sudo /etc/init.d/udev restart (or "sudo service udev restart" on Ubuntu >= Karmic Koala)
    - create the group "input"
          - sudo groupadd -f input
    - put yourself in that group
          - sudo gpasswd -a your_username input
    - You may need to restart your system (at least on Ubuntu >= Karmic Koala)
    - That's it!

Example of a .fdi file to disable a given mouse device 

\verbatim
<?xml version="1.0" encoding="UTF-8"?> <!-- -*- SGML -*- -->

<!-- 
  An example fdi config that disables the use of specific pointing device as is
  This example considers "Cirque Corporation USB GlidePoint" devices. For your device
  plug it in and check its name using for instance lshal | grep "info.product".
-->
<deviceinfo version="0.2">

  <device>
    <match key="info.product" string="Cirque Corporation USB GlidePoint">
      <merge key="info.ignore" type="bool">true</merge>
    </match>
  </device>

</deviceinfo>
\endverbatim

\note Please have a look at src/lasaImitation/MultipleMiceExample/src/MMiceExample.cpp to see how this class can be instantiated.
 
\section dependencies_sec Dependencies

- YARP

\author Eric Sauser

Copyright (C) 2008 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/lasaImitation/MultipleMiceDriver/src/MMiceDeviceDriver.h.
**/

#ifndef __MMiceDeviceDriverh__
#define __MMiceDeviceDriverh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Time.h>
#include <vector>

#include "manymouse.h"

using std::vector;
/*
namespace yarp {
    namespace dev {
        class MMiceDeviceDriver;
    }
}
*/


/**
 * \ingroup icub__lasaImitation_MultipleMiceDriver
 *
 * A device driver class get input simltaneously information form multiple mices
 * They can be standard mices, touchpads, and even 3D mices... However, only 
 * one button is considered.
 * 
 * Please have a look at src/lasaImitation/MultipleMiceExample/src/MMiceExample.cpp to see
 * how this class can be instantiated.
 */
class MMiceDeviceDriver : public yarp::dev::DeviceDriver
{
public:

    enum Mode           {MMM_STANDARD=0, MMM_INTEGRATOR, MMM_RELTOABS}; 
    enum MouseEventType {NONE = 0, RELMOTION, BUTTON, ABSMOTION, SCROLL, DISCONNECT};

    typedef struct{
        int X,Y,Z,RX,RY,RZ;
    } Point6DInt;

    typedef struct{
        int     device;
        int     hasChanged;
        int     lastEvent;
        
        int     btnUp;
        int     btnDown;
        int     btnState;
        int     btnLinkState;
        double  lastTime;
        
        union{
            Point6DInt  mRel;
            int                 mRelArray[6];   
        };
        double  lastRelTime;
        
        union{
            Point6DInt  mAbs;
            int                 mAbsArray[6];   
        };

        union{
            Point6DInt  mInt;
            int                 mIntArray[6];   
        };                
    } MouseEventSummary;

public:
    
            MMiceDeviceDriver();
    virtual ~MMiceDeviceDriver();
    
    virtual bool  open();
    virtual bool  close();
  
    
    int           GetNumDevices();

    const char*   GetDeviceName(int i);

    int           GetNumValidDevices();

    bool          IsDeviceValid(int i);
    
    bool  SetBaseMiceName(const char* baseMiceName);

    void  Update();

    // void  PrintSummary();
    // void  PrintSummary2();

    
    MouseEventSummary *GetMouseData(int id);

    bool               HasMouseChanged(int id);

    

    
    bool  InitMap(int nmice);
    
    bool  ResetMap();    
    
    bool  LoadMap(const char * filename);

    bool  SaveMap(const char * filename);

    bool  TouchAndSetMouseId(int id);

          
    void  SetMode(Mode newMode);
    
    
    int           LinkMiceButtons(int *ids, int len);
    void          UnlinkMiceButtons(int linkId);

private:
    
    static  bool               sIsReady;
    static  int                sNumDevices;
    static  int                sNumDrivers;
    static  MouseEventSummary *sMiceEvents;
    static  vector<int>        sDriverUpdateState;
    static  Mode              *sMode;
    static  int                sLastDeviceTouched;
    
    int                mId;
    int                mValidCount;
    int               *mValidDevices;
    MouseEventSummary *mMice;
    int                mNumMice;
    int               *mMiceMap;  
    int                mMouseToId;
    bool               bIsReady;
    int              **mButtonLinks;
    int                mButtonLinksSize;
    
    double             mCurrentTime;
    double             mIntegratorStopTime;

    bool  ClearMiceData(int id);
};



#endif


/* Copyright (C) 2013  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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



#ifndef __comanDevicesHandler_h__
#define __comanDevicesHandler_h__

#include "Debug.h"
#include <yarp/dev/Drivers.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#include <Boards_iface.h>

// No need to include this class into yarp::dev namespace.
class comanDevicesHandler: public yarp::dev::DeviceDriver
{
public:
    static yarp::os::Semaphore    comanDevicesHandler_mutex;

private:
    // Data for Singleton handling
    bool                          _initted;
    static comanDevicesHandler    *_handle;
    Boards_ctrl                   *_board_crtl;
    static int                    _usedBy;
    int                           *_gravityOffsets;
    int                           _gravityOffsetsVectorSize;

private:
    comanDevicesHandler();                      // Singletons have private constructor
    ~comanDevicesHandler();


public:
    /*! @fn     static  comanDevicesHandler* instance();
     *  @brief  Create the Singleton if it doesn't exists yet and return the pointer.
     *  @return Pointer to comanDevicesHandler singleton
     */
    static  comanDevicesHandler* instance();

    /*! @fn     static  comanDevicesHandler* deInstance();
     *  @brief  Deregister from the singleton usage and destroy it when done
     *  @return true
     */
    static  bool deInstance();

    /*! @fn     open(yarp::os::Searchable& config);
     *  @brief  Open and configure the singleton, verifies internally if already initted
     *  @param  config a searcheable object with path to Yaml file
     *  @return True if succesfully intitialized, false if errors occurred
     */
    bool open(yarp::os::Searchable& config);
    bool close();
    Boards_ctrl *getBoard_ctrl_p();

    void initGravityWorkAround();
    bool setGravityOffset(int bId, int offset);
};

#endif

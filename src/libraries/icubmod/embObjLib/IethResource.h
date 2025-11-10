
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2014  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Marco Accame
 * email: marco.accame@iit.it
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

//
// $Id: TheEthManager.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __IethResource__
#define __IethResource__


#include "EoProtocol.h"
#include <string>


// marco.accame on 20 oct 2014.
// the objects which use ethResource to communicate with the ethernet boards inside the robot
// must be derived from this class IethResource.
// these objects are: embObjMotionControl, embObjSkin, embObjAnalogSensor, and future ones.
// these object must implement the virtual functions initialised() and update() so that:
// - initialised() must return true only if the object is opened (its method open() has returned).
// - update() takes care of filling private data structures with bytes contained in the relevant ROPs coming from remote boards.
//   as an example, see how the skin data is moved from ems board to embObjSkin.
//   when a ROP containing a EOarray of skin data arrives, the callback eoprot_fun_UPDT_sk_skin_status_arrayof10canframes() is called.
//   this function calls feat_manage_skin_data() and passes the boardnumber, the id32, and the pointer to the received EOarray.
//   this function uses TheEthManager::getHandle() to retrieve a pointer to the relevant IethResource which matches (boardnumber, id32)
//   the pointer is used directly or transformed with dynamic_cast to a pointer to embObjSkin and the function update() is called.
//   needless to say, the update(0 function whcich is called is always the embObjSkin::update() which gets the EOarray and copies its
//   items inside embObjSkin::data.
//
// warning: the caller of function embObjSkin::update() is the ethReceiver thread. if it writes something into embObjSkin::data,
//          and the same embObjSkin::data is read by other threads via EmbObjSkin::read(), then ..... the two concurrent operations
//          must be protected with a mutex.
//          the rxdata passed from eoprot_fun_UPDT_sk_skin_status_arrayof10canframes() upto embObjSkin::update() does not need to be
//          protected vs concurrent use because ... the only thread which writes into it is the caller of teh function: the ethReceiver thread
//
// the name of the class is IethResource because this class acts as an interface from ethResource which is the one which
// manages decoding of received UDP packets and calls the callbacks of the EOnv which in turn call IethResource::update().
//


namespace eth {

    typedef enum
    {
        iethres_none            = 255,
        iethres_management      = 0,
        iethres_analogmais      = 1,
        iethres_analogstrain    = 2,
        iethres_motioncontrol   = 3,
        iethres_skin            = 4,
        iethres_analogvirtual   = 5,
        iethres_analogmultienc  = 7,
        iethres_analoginertial3 = 8,
        iethres_temperature     = 9,
        iethres_analogpsc       = 10,
        iethres_analogpos       = 11,
        iethres_analogft        = 12,
        iethres_analogbattery = 13
    } iethresType_t;

    enum { iethresType_numberof = 14 };


    class IethResource
    {

    public:
            virtual ~IethResource() {}

            virtual bool initialised() = 0;
            virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata) = 0;
            virtual iethresType_t type() = 0;

    public:
            const char * stringOfType();
            /* the entityName there will be the name of the entity, or it can be an empty string if the enity hasn't name
            The entity could be:
              - axis in case the ethResource type is iethres_motioncontrol
              - sensor id in case of, skin, mais and strain 
              The defiult value is an empty string 
              return false in case of error, like entityId is not present. Aniway the entity name is initialize to empty string.*/
            virtual bool getEntityName(uint32_t entityId, std::string &entityName);

            /* the encoderTypeName will be the name of the type of the encoder mounted at the joint or at the motor, 
            or it can be the string "ERROR" is a corresponding name is not found
            The entity, i.e. the jomo, can be:
              - motor
              - joint
            The default value is the empty string
            It returns false in case of error and the encoderTypeName string is filled with "ERROR"
            */
            virtual bool getEncoderTypeName(uint32_t jomoId, eOmc_position_t pos, std::string &encoderTypeName);

            virtual bool getEntityControlModeName(uint32_t jomoId, eOmc_controlmode_t control_mode, std::string &controlModeName, eObool_t compact_string = eobool_true);

    private:
            static const char * names[iethresType_numberof+1];
    };

} // namespace eth

#endif

// eof


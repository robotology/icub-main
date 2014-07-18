// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
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


#ifndef __SKIN_MESH_THREAD_H__
#define __SKIN_MESH_THREAD_H__

//#include <stdio.h>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#define EMBSK_SIZE_INFO     128
// embObj includes
#include <ethManager.h>
#include <ethResource.h>
#include "EoUtilities.h"
#include "FeatureInterface_hid.h"       // Interface with embObj world (callback)
//#include "skinParams.h"
#include "SkinConfigReader.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::os::impl;
using namespace yarp::sig;



class SkinPatchInfo
{
public:
    int                     idPatch;
    eOprotIndex_t           indexNv;
    std::vector <int>       cardAddrList;
    bool checkCardAddrIsInList(int cardAddr);
};

class SkinConfig
{
   public:
   int                             totalCardsNum;
   std::vector<SkinPatchInfo>      patchInfoList;
   uint8_t                         numOfPatches;
};

class EmbObjSkin :  public yarp::dev::IAnalogSensor,
                    public DeviceDriver,
                    public IiCubFeature
{
protected:
    TheEthManager   *ethManager;
    PolyDriver      resource;
    ethResources    *res;
    FEAT_ID         _fId;
    bool            initted;
    Semaphore       mutex;
    //int             totalCardsNum;
    //std::vector<SkinPatchInfo> patchInfoList;
    size_t          sensorsNum;
    Vector          data;
    //uint8_t         numOfPatches; //currently one patch is made up by all skin boards connected to one can port of ems.
    SkinBoardCfgParam _brdCfg;
    SkinTriangleCfgParam _triangCfg;
    bool            _newCfg;
    SkinConfigReader *_cfgReader;
    SkinConfig        _skCfg;

    bool            init();
    bool            fromConfig(yarp::os::Searchable& config);
    bool            initWithSpecialConfig(yarp::os::Searchable& config);
    bool            isEpManagedByBoard();
    bool            start();
    bool            configPeriodicMessage(void);
    eOprotIndex_t convertIdPatch2IndexNv(int idPatch)
    {
      /*in xml file idPatch are number of ems canPort identified with numer 1 or 2 on electronic schematics.
      * in ethernet protocol the patch number is the index part of network variable identifier, that starts from 0 */
        if(_skCfg.numOfPatches == 1)
            return(0);
        else
            return(idPatch-1);
    }


public:
    EmbObjSkin();
    ~EmbObjSkin()   { }

    char            info[EMBSK_SIZE_INFO];

    virtual bool    open(yarp::os::Searchable& config);

    virtual bool    close();

    virtual int     read(yarp::sig::Vector &out);
    virtual int     getState(int ch);
    virtual int     getChannels();
    virtual int     calibrateSensor();
    virtual int     calibrateChannel(int ch, double v);

    virtual int     calibrateSensor(const yarp::sig::Vector& v);
    virtual int     calibrateChannel(int ch);

    virtual bool    fillData(void *data, eOprotID32_t id32);
    virtual void    setId(FEAT_ID &id);
    bool            isInitted(void);
};

#endif


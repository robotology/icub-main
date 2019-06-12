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


#ifndef __EMBOBJSKIN_H__
#define __EMBOBJSKIN_H__

#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>


#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>


#include "SkinConfigReader.h"
#include <SkinDiagnostics.h>
#include "serviceParser.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::os::impl;
using namespace yarp::sig;



class SkinPatchInfo
{
public:
    int                     idPatch;
    eOcanport_t             canport; // so far a patch contains addresses of a unique canport
    eOprotIndex_t           indexNv;
    std::vector <int>       cardAddrList;
    int checkCardAddrIsInList(int cardAddr);
};

class SkinConfig
{
   public:
   int                             totalCardsNum;
   std::vector<SkinPatchInfo>      patchInfoList;
   uint8_t                         numOfPatches;
};


// -- class EmbObjSkin

class EmbObjSkin :  public yarp::dev::IAnalogSensor,
                    public DeviceDriver,
                    public eth::IethResource
{

public:

    enum { EMBSK_SIZE_INFO = 128 };
    enum { SPECIAL_TRIANGLE_CFG_MAX_NUM = 20 };

    bool            opened;

protected:

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;

    eth::TheEthManager *ethManager;
    eth::AbstractEthResource *res;

    Semaphore       mutex;
    //int             totalCardsNum;
    //std::vector<SkinPatchInfo> patchInfoList;
    size_t          sensorsNum;
    Vector          skindata;
    //uint8_t         numOfPatches; //currently one patch is made up by all skin boards connected to one can port of ems.
    SkinBoardCfgParam _brdCfg;
    SkinTriangleCfgParam _triangCfg;
    bool            _newCfg;
    SkinConfigReader  _cfgReader;
    SkinConfig        _skCfg;

    bool            init();
    bool            fromConfig(yarp::os::Searchable& config);
    bool            initWithSpecialConfig(yarp::os::Searchable& config);
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

private:

    ServiceParser *parser;
    eOmn_serv_parameter_t ethservice;

    bool verbosewhenok;

    /****************** diagnostic********************************/
    bool _isDiagnosticPresent;       // is the diagnostic available from the firmware
    /*************************************************************/

    /** The detected skin errors. These are used for diagnostics purposes. */
    std::vector<iCub::skin::diagnostics::DetectedError> errors;

public:

    EmbObjSkin();
    ~EmbObjSkin();

    virtual bool    open(yarp::os::Searchable& config);

    virtual bool    close();
    void            cleanup(void);

    virtual int     read(yarp::sig::Vector &out);
    virtual int     getState(int ch);
    virtual int     getChannels();
    virtual int     calibrateSensor();
    virtual int     calibrateChannel(int ch, double v);

    virtual int     calibrateSensor(const yarp::sig::Vector& v);
    virtual int     calibrateChannel(int ch);

    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);

};

#endif


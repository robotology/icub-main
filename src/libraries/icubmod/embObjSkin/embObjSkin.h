// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2026  Mesh Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino, Jacopo Losi
 * email: alberto.cardellino@iit.it, jacopo.losi@iit.it
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
#include <mutex>
#include <optional>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ReturnValue.h>


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
    size_t                  taxelsOffset{0};
    size_t                  taxelsSize{0}; // number of taxels per patch
    size_t                  boardsOffset{0};
    std::optional<size_t>   checkCardAddrIsInList(int cardAddr);
};

class SkinConfig
{
   public:
   int                             totalCardsNum;
   std::vector<SkinPatchInfo>      patchInfoList;
   uint8_t                         numOfPatches;
};


// -- class EmbObjSkin

class EmbObjSkin :  public ISkinPatches,
                    public DeviceDriver,
                    public eth::IethResource
{
protected:

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;

    eth::TheEthManager *ethManager;
    eth::AbstractEthResource *res;

    mutable std::mutex   mtx;
    size_t               sensorsNum;
    size_t               _cumulativeTaxels;
    Vector               skindata;
    SkinBoardCfgParam    _brdCfg;
    SkinTriangleCfgParam _triangCfg;
    bool                 _newCfg;
    SkinConfigReader     _cfgReader;
    SkinConfig           _skCfg;

    void            cleanup(void);
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
    bool opened;
    std::string boardInfo() const;

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

    // DeviceDriver interface
    virtual bool    open(yarp::os::Searchable& config);
    virtual bool    close();

    // ISkinPatches interface    
    /** Get the number of skin patches exposed by THIS device */
    virtual yarp::dev::ReturnValue  getNrOfSkinPatches(size_t &n) const override;
    /** Get the status of the specified patch */
    virtual MAS_status getSkinPatchStatus(size_t sens_index) const override;
    /** Get the name of the specified patch */
    virtual yarp::dev::ReturnValue    getSkinPatchName(size_t sens_index, std::string &name) const override;
    /** Get the last readings of the sensors related to the specified patch */
    virtual yarp::dev::ReturnValue    getSkinPatchMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;
    /** Get the size of the specified skin patch, i.e., the number of taxels (sensors) it contains */
    virtual size_t  getSkinPatchSize(size_t sens_index) const override;

    // IethResource interface
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);

};

#endif


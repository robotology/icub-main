/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


 
#include "FeatureInterface.h"
#include "FeatureInterface_hid.h"

#include <ethManager.h>
#include "embObjMotionControl.h"
#include "embObjAnalogSensor.h"
#include "embObjSkin.h"

#include "EoProtocol.h"

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/LogStream.h>

static TheEthManager *_interface2ethManager = NULL;

void embObjPrintTrace(char *string)
{
    yTrace("%s", string);
}

void embObjPrintDebug(char *string)
{
    yDebug("%s", string);
}

void embObjPrintInfo(char *string)
{
    yInfo("%s", string);
}

void embObjPrintWarning(char *string)
{
    yWarning("%s", string);
}

void embObjPrintError(char *string)
{
    yError("%s", string);
}

void embObjPrintFatal(char *string)
{
    yError("EMS received the following FATAL error: %s", string);
}

void feat_Initialise(void *ethman)
{
    if(_interface2ethManager == NULL )
    {
        _interface2ethManager = (TheEthManager*) ethman;
    }
}

void feat_DeInitialise()
{
    _interface2ethManager = NULL;
}

fakestdbool_t feat_addEncoderTimeStamp(FEAT_boardnumber_t boardnum, eOprotID32_t id32)
{
    IethResource* mc = NULL;
    ethFeatType_t type;

    if (_interface2ethManager == NULL)
        return fakestdbool_false;

    bool ret = _interface2ethManager->getHandle(boardnum, id32, &mc, &type);

    if(!ret)
    {
        yDebug("EMS callback was unable to get access to the embObjMotionControl");
        return fakestdbool_false;
    }

    if(type != ethFeatType_MotionControl)
    {
        return fakestdbool_false;
    }
    else if(false == mc->initialised())
    {
        return fakestdbool_false;
    }
    else
    {
        mc->update(id32, yarp::os::Time::now(), NULL);
    }

    return fakestdbool_true;
}


fakestdbool_t feat_manage_motioncontrol_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void* rxdata)
{
    IethResource* mc = NULL;
    ethFeatType_t type;

    if (_interface2ethManager == NULL)
        return fakestdbool_false;

    bool ret = _interface2ethManager->getHandle(boardnum, id32, &mc, &type);

    if(!ret)
    {
        yDebug("EMS callback was unable to get access to the embObjMotionControl");
        return fakestdbool_false;
    }

    if(type != ethFeatType_MotionControl)
    {
        return fakestdbool_false;
    }
    else if(false == mc->initialised())
    {
        return fakestdbool_false;
    }
    else
    {
        mc->update(id32, yarp::os::Time::now(), rxdata);
    }

    return fakestdbool_true;
}

fakestdbool_t feat_manage_skin_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *arrayofcandata)
{   
    static int error = 0;
    IethResource* skin;
    ethFeatType_t type;

    if (_interface2ethManager == NULL)
        return fakestdbool_false;

    bool ret = _interface2ethManager->getHandle(boardnum, id32, &skin, &type);

    if(!ret)
    {
        yDebug("EMS callback was unable to get access to the embObjSkin");
        return fakestdbool_false;
    }

    if(type != ethFeatType_Skin)
    {   // the ethmanager does not know this object yet or the dynamic cast failed because it is not an embObjSkin
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        if(0 == (error%1000) )
            yWarning() << "feat_manage_skin_data() received a request from BOARD" << boardnum << "with ID:" << nvinfo << "but no class was jet instatiated for it";

        error++;
        return fakestdbool_false;
    }
    else if(false == skin->initialised())
    {   // the ethmanager has the object, but the device was not fully opened yet. cannot use it
        return fakestdbool_false;
    }
    else
    {   // the object exists and is completed: it can be used
        skin->update(id32, yarp::os::Time::now(), (void *)arrayofcandata);
    }

    return fakestdbool_true;
}


fakestdbool_t feat_manage_analogsensors_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *data)
{
    IethResource* sensor;
    ethFeatType_t type;

    if (_interface2ethManager == NULL)
        return fakestdbool_false;

    bool ret = _interface2ethManager->getHandle(boardnum, id32, &sensor, &type);

    if(!ret)
    {
        yDebug("EMS callback was unable to get access to the embObjAnalogSensor");
        return fakestdbool_false;
    }

    if(! ((type == ethFeatType_AnalogMais) || (type == ethFeatType_AnalogStrain) || (type == ethFeatType_AnalogInertial)) )
    {
        yError("EMS analog sensor callback - the ethmanager does not know this object YET or a wrong pointer has been returned because it is not an embObjAnalogSensor");
        return fakestdbool_false;
    }
    else if(false == sensor->initialised())
    {   // the ethmanager has the object, but the obiect was not full initted yet. cannot use it
        return fakestdbool_false;
    }
    else
    {   // the object exists and is completed: it can be used.
        // data is a EOarray* in case of mais or strain but it is a eOas_inertial_status_t* in case of inertial sensor
        sensor->update(id32, yarp::os::Time::now(), data);
    }

    return fakestdbool_true;
}


void* feat_MC_handler_get(FEAT_boardnumber_t boardnum, eOprotID32_t id32)
{
    IethResource* h = NULL;
    ethFeatType_t type;

    if (_interface2ethManager == NULL)
        return NULL;

    _interface2ethManager->getHandle(boardnum, id32, &h, &type);
    return (void*) h;
}

fakestdbool_t feat_MC_mutex_post(void *mchandler, uint32_t prognum)
{
    eoThreadEntry *th = NULL;
    IethResource *ier = static_cast<IethResource*>(mchandler);
    IethResource *mc = ier; //dynamic_cast<embObjMotionControl *>(ier);

    if(NULL == mc)
    {
        return fakestdbool_false;
    }
    else if(false == mc->initialised())
    {   // it can be that the object is already created but its open() not yet completed. it is in open() that we allocate requestQueue ....
        return fakestdbool_false;
    }


    int threadId;
    eoThreadFifo *fuffy = mc->getFifo(prognum);

    if( (threadId = fuffy->pop()) < 0)
    {
        yError() << "Received an answer message nobody is waiting for (MCmutex_post)";
        return fakestdbool_false;
    }
    else
    {
        th = mc->getThreadTable(threadId);
        if(NULL == th)
            yError() << "MCmutex_post error at line " << __LINE__;
        th->push();
        return fakestdbool_true;
    }

    return fakestdbool_false;
}


FEAT_boardnumber_t nvBoardNum2FeatIdBoardNum(eOprotBRD_t nvboardnum)
{
    if(eo_prot_BRDdummy == nvboardnum)
    {
        return(FEAT_boardnumber_dummy);
    }
    
    return(nvboardnum+1);
}

eOprotBRD_t featIdBoardNum2nvBoardNum(FEAT_boardnumber_t fid_boardnum)
{
    if(FEAT_boardnumber_dummy == fid_boardnum)
    {
        return(eo_prot_BRDdummy);
    }

    return(fid_boardnum-1);
}

double feat_yarp_time_now(void)
{
    return(yarp::os::Time::now());
}

fakestdbool_t feat_signal_network_reply(eOprotBRD_t brd, eOprotID32_t id32, uint32_t signature)
{
    if (_interface2ethManager == NULL)
        return(fakestdbool_false);

    ethResources* ethres = _interface2ethManager->GetEthResource(nvBoardNum2FeatIdBoardNum(brd));

    if(NULL == ethres)
    {
        return(fakestdbool_false);
    }

    return(ethres->aNetworkQueryReplyHasArrived(id32, signature));
}

fakestdbool_t feat_embObjCANPrintHandler(eOprotBRD_t brd, eOmn_info_basic_t* infobasic)
{
    if (_interface2ethManager == NULL)
        return(fakestdbool_false);

    ethResources* ethres = _interface2ethManager->GetEthResource(nvBoardNum2FeatIdBoardNum(brd));

    //Call the ethres manager
    bool res = ethres->CANPrintHandler(infobasic);
    return res;
}

#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/Recursive_Thread_Mutex.h>



// returns a void pointer to the allocated ACE_Recursive_Thread_Mutex
void* ace_mutex_new(void)
{
    ACE_Recursive_Thread_Mutex* mtx = new ACE_Recursive_Thread_Mutex();
    return((void*)mtx);
}

// returns 0 on success to take mutex, -3 on failure upon timeout, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(), tout_usec is in microsec (no timeout is 0xffffffff).
int8_t ace_mutex_take(void* m, uint32_t tout_usec)
{
    ACE_Recursive_Thread_Mutex* acemtx = (ACE_Recursive_Thread_Mutex*)m;
    if(NULL == acemtx)
    {
        return(-2);
    }

    acemtx->acquire();

    return(0);
}

// returns 0 on success to take mutex, -1 on genric failure of releasing mutex, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(),
int8_t ace_mutex_release(void* m)
{
    ACE_Recursive_Thread_Mutex* acemtx = (ACE_Recursive_Thread_Mutex*)m;
    if(NULL == acemtx)
    {
        return(-2);
    }

    acemtx->release();

    return(0);
}

void ace_mutex_delete(void* m)
{
    ACE_Recursive_Thread_Mutex* acemtx = (ACE_Recursive_Thread_Mutex*)m;
    if(NULL != acemtx)
    {
        delete acemtx;
    }
}


// eof





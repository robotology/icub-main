/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <ethManager.h>
#include <IethResource.h>
#include <abstractEthResource.h>
#include <theNVmanager.h>


#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>


#include <ace/ACE.h>
#include <ace/config.h>
#include <ace/Recursive_Thread_Mutex.h>
#include "EOYtheSystem.h"
#include "EOYmutex.h"


using namespace eth;



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "FeatureInterface.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface
// --------------------------------------------------------------------------------------------------------------------

#include "FeatureInterface_hid.h"


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables. deprecated: better using _get(), _set() on static variables
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-secction

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-secction


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static eth::TheEthManager *_interface2ethManager = NULL;


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


void feat_Initialise(void *handleOfTheEthManager)
{
    if(NULL == _interface2ethManager)
    {
        _interface2ethManager = reinterpret_cast<eth::TheEthManager*>(handleOfTheEthManager);
    }
    if (yarp_time_now_func_ptr == NULL) {
        yarp_time_now_func_ptr = &feat_yarp_time_now;
    }
    if (ace_mutex_new_func_ptr == NULL ) {
        ace_mutex_new_func_ptr = &ace_mutex_new;
    }
    if (ace_mutex_take_func_ptr == NULL) {
        ace_mutex_take_func_ptr = &ace_mutex_take;
    }
    if (ace_mutex_release_func_ptr == NULL) {
        ace_mutex_release_func_ptr = &ace_mutex_release;
    }
    if(ace_mutex_delete_func_ptr == NULL) {
        ace_mutex_delete_func_ptr = &ace_mutex_delete;
    }
}


void feat_DeInitialise()
{
    _interface2ethManager       = NULL;
    yarp_time_now_func_ptr      = NULL;
    ace_mutex_new_func_ptr      = NULL;
    ace_mutex_take_func_ptr     = NULL;
    ace_mutex_release_func_ptr  = NULL;
    ace_mutex_delete_func_ptr   = NULL;
}


eObool_t feat_manage_motioncontrol_data(eOipv4addr_t ipv4, eOprotID32_t id32, void* rxdata)
{
    IethResource* mc = NULL;

    if(NULL == _interface2ethManager)
    {
        return eobool_false;
    }

    mc = _interface2ethManager->getInterface(ipv4, id32);

    if(NULL == mc)
    {
        char ipinfo[20];
        char nvinfo[128];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yDebug("feat_manage_motioncontrol_data() fails to get a handle of embObjMotionControl for IP = %s and NV = %s", ipinfo, nvinfo);
        return eobool_false;
    }

    if(false == mc->initialised())
    {
        return eobool_false;
    }
    else
    {
        mc->update(id32, yarp::os::Time::now(), rxdata);
    }

    return eobool_true;
}


eObool_t feat_manage_motioncontrol_addinfo_multienc(eOipv4addr_t ipv4, eOprotID32_t id32, void* rxdata)
{
    IethResource* multienc = NULL;

    if(NULL == _interface2ethManager)
    {
        return eobool_false;
    }

    multienc = _interface2ethManager->ethBoards->get_interface(ipv4, iethres_analogmultienc);
    
    if(NULL == multienc)
    {
        char ipinfo[20];
        char nvinfo[128];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yDebug("feat_manage_motioncontrol_addinfo_multienc() fails to get a handle of embObjMotionControl for IP = %s and NV = %s", ipinfo, nvinfo);
        return eobool_false;
    }

    if(false == multienc->initialised())
    {
        return eobool_false;
    }
    else
    {
        multienc->update(id32, yarp::os::Time::now(), rxdata);
    }

    return eobool_true;
}


eObool_t feat_manage_skin_data(eOipv4addr_t ipv4, eOprotID32_t id32, void *arrayofcandata)
{   
    IethResource* skin;

    if(NULL == _interface2ethManager)
    {
        return eobool_false;
    }

    skin = _interface2ethManager->getInterface(ipv4, id32);

    if(NULL == skin)
    {
        char ipinfo[20];
        char nvinfo[128];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yDebug("feat_manage_skin_data() fails to get a handle of embObjSkin for IP = %s and NV = %s", ipinfo, nvinfo);
        return eobool_false;
    }

    if(false == skin->initialised())
    {
        return eobool_false;
    }
    else
    {
        skin->update(id32, yarp::os::Time::now(), (void *)arrayofcandata);
    }

    return eobool_true;
}


eObool_t feat_manage_analogsensors_data(eOipv4addr_t ipv4, eOprotID32_t id32, void *data)
{
    IethResource* sensor;

    if(NULL == _interface2ethManager)
    {
        return eobool_false;
    }

    sensor = _interface2ethManager->getInterface(ipv4, id32);

    if(NULL == sensor)
    {
        char ipinfo[20];
        char nvinfo[128];
        eo_common_ipv4addr_to_string(ipv4, ipinfo, sizeof(ipinfo));
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yDebug("feat_manage_analogsensors_data() fails to get a handle of embObjAnalogSensor for IP = %s and NV = %s", ipinfo, nvinfo);
        return eobool_false;
    }

    if(false == sensor->initialised())
    {
        return eobool_false;
    }
    else
    {   // data is a EOarray* in case of mais or strain but it is a eOas_inertial_status_t* in case of inertial sensor
        sensor->update(id32, yarp::os::Time::now(), data);
    }

    return eobool_true;
}


void* feat_MC_handler_get(eOipv4addr_t ipv4, eOprotID32_t id32)
{
    IethResource* h = NULL;

    if(NULL == _interface2ethManager)
    {
        return NULL;
    }

    h = _interface2ethManager->getInterface(ipv4, id32);

    return (void*) h;
}


double feat_yarp_time_now(void)
{
    return(yarp::os::Time::now());
}

eObool_t feat_signal_network_onsay(eOipv4addr_t ipv4, eOprotID32_t id32, uint32_t signature)
{
    theNVmanager& nvman = theNVmanager::getInstance();
    nvman.onarrival(theNVmanager::ropCode::say, ipv4, id32, signature);
    return eobool_true;
}

eObool_t feat_signal_network_onsig(eOipv4addr_t ipv4, eOprotID32_t id32, uint32_t signature)
{
    theNVmanager& nvman = theNVmanager::getInstance();
    nvman.onarrival(theNVmanager::ropCode::sig, ipv4, id32, signature);
    return eobool_true;
}


eObool_t feat_CANprint(eOipv4addr_t ipv4, eOmn_info_basic_t* infobasic)
{
    if(NULL == _interface2ethManager)
    {
        return(eobool_false);
    }

    eth::AbstractEthResource* ethres = _interface2ethManager->getEthResource(ipv4);

    bool res = ethres->CANPrintHandler(infobasic);
    return res;
}


const char * feat_GetBoardName(eOipv4addr_t ipv4)
{
    static const char * errorstr = "error";

    if(NULL == _interface2ethManager)
    {
        return errorstr;
    }

    return(_interface2ethManager->getName(ipv4).c_str());
}


void feat_PrintTrace(char *string)
{
    yTrace("%s", string);
}


void feat_PrintDebug(char *string)
{
    yDebug("%s", string);
}


void feat_PrintInfo(char *string)
{
    yInfo("%s", string);
}


void feat_PrintWarning(char *string)
{
    yWarning("%s", string);
}


void feat_PrintError(char *string)
{
    yError("%s", string);
}


void feat_PrintFatal(char *string)
{
    yError("EMS received the following FATAL error: %s", string);
}


// returns a void pointer to the allocated ACE_Recursive_Thread_Mutex
void* ace_mutex_new(void)
{
    ACE_Recursive_Thread_Mutex* mtx = new ACE_Recursive_Thread_Mutex();
    return(mtx);
}


// returns 0 on success to take mutex, -3 on failure upon timeout, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(), tout_usec is in microsec (no timeout is 0xffffffff).
int8_t ace_mutex_take(void* m, uint32_t tout_usec)
{
    ACE_Recursive_Thread_Mutex* acemtx = reinterpret_cast<ACE_Recursive_Thread_Mutex*>(m);
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
    ACE_Recursive_Thread_Mutex* acemtx = reinterpret_cast<ACE_Recursive_Thread_Mutex*>(m);
    if(NULL == acemtx)
    {
        return(-2);
    }

    acemtx->release();

    return(0);
}


void ace_mutex_delete(void* m)
{
    ACE_Recursive_Thread_Mutex* acemtx = reinterpret_cast<ACE_Recursive_Thread_Mutex*>(m);
    if(NULL != acemtx)
    {
        delete acemtx;
    }
}


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------







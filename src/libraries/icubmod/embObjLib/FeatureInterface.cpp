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




static TheEthManager *_interface2ethManager = NULL;

void initCallback(void *p)
{
    if(_interface2ethManager == NULL )
    {
        _interface2ethManager = (TheEthManager*) p;
    }
}

fakestdbool_t addEncoderTimeStamp(FEAT_ID *id, int jointNum)
{
    embObjMotionControl *tmp = (embObjMotionControl *)(_interface2ethManager->getHandle(id->boardNum, id->ep));
    if(tmp != NULL)
    {
        tmp->refreshEncoderTimeStamp(jointNum);
        return fakestdbool_true;
    }

    return fakestdbool_false;
}

fakestdbool_t findAndFill(FEAT_ID *id, void *sk_array, eOprotID32_t id32)
{
    // new with table, data stored in eoSkin;
    // specie di view grezza, usare dynamic cast?
    static int error = 0;
    static int notYetInitted = 0;
    IiCubFeature *skin;
    EmbObjSkin *tmp = (EmbObjSkin *)(_interface2ethManager->getHandle(id->boardNum, id->ep));

    if(NULL == tmp)
    {   // the ethmanager does not know this object yet
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        if(0 == (error%1000) )
            yError() << "Got a message from BOARD" << id->boardNum << "with ID:" << nvinfo << "but no class was instatiated for it";

        error++;
        return fakestdbool_false;
    }
    else if(false == tmp->isOpened())
    {   // the ethmanager has the object, but the device was not fully opened yet. cannot use it
        return fakestdbool_false;
    }
    else
    {   // the object exists and is completed: it can be used
        skin = dynamic_cast<IiCubFeature *>(tmp);
        if(NULL != skin)
        {
            skin->fillData((void *)sk_array, id32);
        }
        else
        {
            if(0 == (notYetInitted%1000))
                yWarning() << "Got a SKIN message with EP "<< id->ep << "board number " << id->boardNum << " not yet initialized";

            notYetInitted++;
            return fakestdbool_false;
        }
    }
    return fakestdbool_true;
}

void* get_MChandler_fromEP(uint8_t boardnum, eOprotEndpoint_t ep)
{
    void* h = NULL;
    h = _interface2ethManager->getHandle(boardnum, ep);
    return h;
}

fakestdbool_t handle_AS_data(FEAT_ID *id, void *as_array, eOprotID32_t id32)
{
    IiCubFeature *iAnalog;

    // specie di view grezza, usare dynamic cast?
    embObjAnalogSensor *tmp = (embObjAnalogSensor *)(_interface2ethManager->getHandle(id->boardNum, id->ep));

    if(NULL == tmp)
    {   // the ethmanager does not know this object yet
        return fakestdbool_false;
    }
    else if(false == tmp->isOpened())
    {   // the ethmanager has the object, but the obiect was not full initted yet. cannot use it
        return fakestdbool_false;
    }
    else
    {   // the object exists and is completed: it can be used
        iAnalog = dynamic_cast<IiCubFeature *>(tmp);
        iAnalog->fillData(as_array, id32);
    }

    return fakestdbool_true;
}

fakestdbool_t MCmutex_post(void *p, uint32_t prognum)
{
    eoThreadEntry *th = NULL;
    embObjMotionControl *handler = (embObjMotionControl *) p;

    if(NULL == handler)
    {
        return fakestdbool_false;
    }
    else if(false == handler->isOpened())
    {   // it can be that the object is already created but its open() not yet completed. it is in open() that we allocate requestQueue ....
        return fakestdbool_false;
    }


    int threadId;
    eoThreadFifo *fuffy = handler->requestQueue->getFifo(prognum);

    if( (threadId = fuffy->pop()) < 0)
    {
        yError() << "Received an answer message nobody is waiting for (MCmutex_post)";
        return fakestdbool_false;
    }
    else
    {
        th = handler->requestQueue->threadPool->getThreadTable(threadId);
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
    ethResources* ethres = _interface2ethManager->GetEthResource(nvBoardNum2FeatIdBoardNum(brd));

    if(NULL == ethres)
    {
        return(fakestdbool_false);
    }

    return(ethres->aNetworkQueryReplyHasArrived(id32, signature));
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


// eof





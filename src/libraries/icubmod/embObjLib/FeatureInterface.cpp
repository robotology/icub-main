/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "FeatureInterface.h"
#include "FeatureInterface_hid.h"
#include "IRobotInterface.h"
#include <ethManager.h>
#include "eOcfg_EPs_board.h"
#include "embObjMotionControl.h"


bool findAndFill(FEAT_ID *id, char *sk_array)
{
    // new with table, data stored in eoSkin;
	// specie di view grezza, usare dynamic cast?
    IiCubFeature * skin =  (IiCubFeature*) (ethResCreator::instance()->getHandleFromEP(id->ep));
    if(NULL == skin)
    {
//        printf( "/************************************\\\n"
//                "            Parte non trovata!!!\n"
//                "\\***********************************/\n");
        return false;
    }
    else
        skin->fillData(sk_array);

    return true;
}

void * get_MChandler_fromEP(eOnvEP_t ep)
{
	void *h = NULL;
	h = ethResCreator::instance()->getHandleFromEP(ep);
	FEAT_ID id = ethResCreator::instance()->getFeatInfoFromEP(ep);

	return h;
}

bool MCmutex_post(void * p, uint16_t epindex, uint16_t nvindex)
{
	//epindex in realtà non serve.
	embObjMotionControl * handler = (embObjMotionControl*) p;
	int threadId;
	eoThreadFifo *fuffy = handler->requestQueue->getFifo(nvindex);
	fuffy->pop(threadId);
	eoThreadEntry * th = handler->requestQueue->threadPool->getThreadTable(threadId);
	th->push();
	return true;
}

bool EP_NV_2_index(eOnvEP_t ep, eOnvID_t nvid, uint16_t *epindex, uint16_t *nvindex)
{
	FEAT_ID _fId = ethResCreator::instance()->getFeatInfoFromEP(ep);
	eOuint16_fp_uint16_t p = _fId.EPhash_function;
	*epindex = p(ep);
	eOnvscfg_EP_t  *EPcfg = eo_cfg_nvsEP_board_EPs_cfg_get(_fId.EPvector, *epindex);
	*nvindex = eo_cfg_nvsEP_board_NVs_endpoint_Nvindex_get(EPcfg, nvid);

	// Ora ho gli indici che mi servono per accedere alla tabella dei thread
}

void transceiver_wait(eOnvEP_t ep)
{
	embObjMotionControl * handler = (embObjMotionControl*) get_MChandler_fromEP(ep);
	handler->res->transceiver->_mutex.wait();
}

void transceiver_post(eOnvEP_t ep)
{
	embObjMotionControl * handler = (embObjMotionControl*) get_MChandler_fromEP(ep);
	handler->res->transceiver->_mutex.post();
}


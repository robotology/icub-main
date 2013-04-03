/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

using namespace std;

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hostTransceiver.hpp"
#include "FeatureInterface.h"

#include "EoCommon.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "Debug.h"

#include <yarp/os/Time.h>

#define _DEBUG_ON_FILE_
#undef _DEBUG_ON_FILE_

#ifdef _DEBUG_ON_FILE_
#define SOGLIA                70000
#define MAX_ACQUISITION       10000
uint64_t idx = 0;
uint64_t max_idx = MAX_ACQUISITION*7*16*2 / 8;
uint64_t utime[MAX_ACQUISITION*16*7*2*2] = {0};
uint64_t errors[MAX_ACQUISITION*16*7*2*2][2] = {0};
uint64_t nErr=0;
FILE *outFile = NULL;waiting for new ms";
#endif



hostTransceiver::hostTransceiver() : transMutex(1)
{
    yTrace();
    bytesUsed = 0;
}

hostTransceiver::~hostTransceiver()
{
    yTrace();
}

bool hostTransceiver::init(uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsizerx, uint8_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();
    eOhosttransceiver_cfg_t hosttxrxcfg;
    hosttxrxcfg.remoteboardipv4addr   = _remoteipaddr;
    hosttxrxcfg.remoteboardipv4port   = _ipport;
    hosttxrxcfg.tobedefined           =   0;


    switch(_board_n)
    {
    case 1:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb1;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb1_fptr_hashfunction_ep2index;
        break;
    case 2:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb2;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb2_fptr_hashfunction_ep2index;
        break;
    case 3:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb3;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb3_fptr_hashfunction_ep2index;
        break;
    case 4:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb4;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb4_fptr_hashfunction_ep2index;
        break;
    case 5:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb5;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb5_fptr_hashfunction_ep2index;
        break;
    case 6:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb6;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb6_fptr_hashfunction_ep2index;
        break;
    case 7:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb7;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb7_fptr_hashfunction_ep2index;
        break;
    case 8:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb8;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb8_fptr_hashfunction_ep2index;
        break;
    case 9:
        hosttxrxcfg.vectorof_endpoint_cfg = eo_cfg_EPs_vectorof_eb9;
        hosttxrxcfg.hashfunction_ep2index = eo_cfg_nvsEP_eb9_fptr_hashfunction_ep2index;
        break;
    default:
        yError() << "Got a non existing board number" << _board_n;
        return false;
    }

    localipaddr  = _localipaddr;
    remoteipaddr = _remoteipaddr;
    ipport       = _ipport;

    // initialise the transceiver: it creates a EOtransceiver and its nvsCfg by loading all the endpoints
    hosttxrx     = eo_hosttransceiver_New(&hosttxrxcfg);
    if(hosttxrx == NULL)
        return false;

    // retrieve teh transceiver
    pc104txrx    = eo_hosttransceiver_Transceiver(hosttxrx);
    if(pc104txrx == NULL)
        return false;

    // retrieve the nvscfg
    pc104nvscfg  = eo_hosttransceiver_NVsCfg(hosttxrx);
    if(pc104nvscfg == NULL)
        return false;

//    p_TxPkt = eo_packet_New(_pktsize);
//    if(p_TxPkt == NULL)
//        return false;

    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
        return false;

    // save board specific configs for later use (they may be needed by some callbacks)
    EPvector = hosttxrxcfg.vectorof_endpoint_cfg;
    EPhash_function_ep2index = hosttxrxcfg.hashfunction_ep2index;
    return true;
}

bool hostTransceiver::nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
{
    transMutex.wait();
    bool ret = true;
    if( eores_OK != eo_nv_Set(nv, dat, forceset, upd))
    {
        yError() << "Error while setting NV data\n";
        ret = false;
    }
    transMutex.post();
    return ret;
}

bool hostTransceiver::load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid)
{
    bool ret = false;
    eOropdescriptor_t ropdesc;
    ropdesc.configuration = eok_ropconfiguration_basic;
    ropdesc.configuration.plustime = 1;
    ropdesc.ropcode = opc;
    ropdesc.ep = ep;
    ropdesc.id = nvid;
    ropdesc.size = 0;
    ropdesc.data = NULL;
    ropdesc.signature = 0;


    transMutex.wait();
    eOresult_t res = eo_transceiver_rop_occasional_Load(pc104txrx, &ropdesc);
    transMutex.post();

    if(eores_OK == res)
        ret = true;
    else
    {
        yError() << "Error while loading ROP in ropframe!!\n";
        ret = false;
    }
    return ret;
}

/* not possible (too messy) go have a unique function to get nvids...
eOnvID_t hostTransceiver::getNVid(char* nvName, uint numberOf, eOnvEP_t endPoint, FeatureType type)
{
    eOnvID_t nvid = EOK_uint16dummy;
    _mutex.wait();
    switch(type)
    {
        // Add endpoint management
        case Management:  (2 different functions
            break;
        case MotionControl:
       3 different function: joint, motor and controller
            nvid = eo_cfg_nvsEP_mc_joint_NVID_Get(  (eOcfg_nvsEP_mc_endpoint_t) endPoint, (eOcfg_nvsEP_mc_jointNumber_t) numberOf, (eOcfg_nvsEP_mc_jointNVindex_t) nvName);
            break;
        case Skin:
            nvid = eo_cfg_nvsEP_sk_NVID_Get(        (eOcfg_nvsEP_sk_endpoint_t) endPoint, (eOcfg_nvsEP_sk_skinNumber_t) numberOf, (eOcfg_nvsEP_sk_skinNVindex_t) nvName);
            break;
        case AnalogStrain:
            nvid = eo_cfg_nvsEP_as_strain_NVID_Get( (eOcfg_nvsEP_as_endpoint_t) endPoint, (eOcfg_nvsEP_as_strainNumber_t) numberOf, (eOcfg_nvsEP_as_strainNVindex_t) nvName);
            break;
        case AnalogMais:
            nvid = eo_cfg_nvsEP_as_mais_NVID_Get(   (eOcfg_nvsEP_as_endpoint_t) endPoint, (eOcfg_nvsEP_as_maisNumber_t) numberOf, (eOcfg_nvsEP_as_maisNVindex_t) nvName);
            break;
        default:
            break;
    }
    _mutex.post();

    return nvid;
} */

bool hostTransceiver::addSetMessage(eOnvID_t nvid, eOnvEP_t endPoint, uint8_t* data)
{
    //uint16_t    nvSize;
    eOresult_t    res;
    EOnv          nv;

    EOnv *nvRoot = getNVhandler(endPoint, nvid, &nv);

    if(NULL == nvRoot)
    {
        yError() << "Unable to get pointer to desired NV with id" << nvid;
        return false;
    }

    transMutex.wait();

    //nvSize = eo_nv_Size(&nv, data);   //TODO calcolare dimesione corretta, tenendo conto di tempo e signature


    if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
    {
        // the nv is not writeable
        yError() << "Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
        transMutex.post();
        return false;
    }


    eOropdescriptor_t ropdesc;
    ropdesc.configuration = eok_ropconfiguration_basic;
    ropdesc.configuration.plustime = 1;
    ropdesc.ropcode = eo_ropcode_set;
    ropdesc.ep = endPoint;
    ropdesc.id = nvid;
    ropdesc.size = 0;
    ropdesc.data = NULL;
    ropdesc.signature = 0;

    transMutex.post();
    bool ret = false;

    for(int i=0; ( (i<5) && (!ret) ); i++)
    {
        transMutex.wait();
        if(eores_OK != eo_transceiver_rop_occasional_Load(pc104txrx, &ropdesc))
        {
            yError() << "Error while loading ROP in ropframe\n";
            transMutex.post();
            yarp::os::Time::delay(0.001);

        }
        else
        {
            transMutex.post();
            ret = true;
        }
    }
    return ret;
}


bool hostTransceiver::addGetMessage(eOnvID_t nvid, eOnvEP_t endPoint)
{
    uint16_t    nvSize;
    eOresult_t    res;
    EOnv          nv;

    EOnv *nvRoot = getNVhandler((uint16_t) endPoint, nvid, &nv);

    if(NULL == nvRoot)
        yError() << "Unable to get pointer to desired NV with id" << nvid;


    eOropdescriptor_t ropdesc;
    ropdesc.configuration = eok_ropconfiguration_basic;
    ropdesc.configuration.plustime = 1;
    ropdesc.ropcode = eo_ropcode_ask;
    ropdesc.ep = endPoint;
    ropdesc.id = nvid;
    ropdesc.size = 0;
    ropdesc.data = NULL;
    ropdesc.signature = 0;
    
    bool ret = false;

    for(int i=0; ( (i<5) && (!ret) ); i++)
    {
        transMutex.wait();
        if(eores_OK != eo_transceiver_rop_occasional_Load(pc104txrx, &ropdesc))
        {
            yError() << "Error while loading ROP in ropframe\n";
            transMutex.post();
            yarp::os::Time::delay(0.001);

        }
        else
        {
            transMutex.post();
            ret = true;
        }
    }
    return ret;    
}

bool hostTransceiver::readBufferedValue(eOnvID_t nvid, eOnvEP_t endPoint, uint8_t *data, uint16_t* size)
{
    EOnv nv;
    EOnv *nvRoot = getNVhandler((uint16_t) endPoint, nvid, &nv);

    if(NULL == nvRoot)
    {
        yError() << "Unable to get pointer to desired NV with id" << nvid;
        return false;
    }
    getNVvalue(nvRoot, data, size);
    return true;
}

bool hostTransceiver::readValue(eOnvID_t nvid, eOnvEP_t endPoint, void* outValue, uint16_t *size)
{
    EOnv tmpNV;
    EOnv *nvRoot = getNVhandler((uint16_t) endPoint, nvid, &tmpNV);

    if(NULL == nvRoot)
    {
        yError () << "Nv pointer not found\n";
        return false;
    }

    return getNVvalue(nvRoot, (uint8_t*) outValue, size);
}


// somebody passes the received packet - this is used just as an interface
void hostTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    eo_packet_Capacity_Get(p_RxPkt, &capacityrxpkt);
    if(size > capacityrxpkt)
    {
        yError () << "received packet has size " << size << "which is higher than capacity of rx pkt = " << capacityrxpkt << "\n";
        return;
    } 

    eo_packet_Payload_Set(p_RxPkt, data, size);
    eo_packet_Addressing_Set(p_RxPkt, remoteipaddr, ipport);
    eo_transceiver_Receive(pc104txrx, p_RxPkt, &numofrops, &txtime);
}

// and Processes it
void checkDataForDebug(uint8_t *data, uint16_t size)
{
    //  yTrace();

    static uint32_t prevTime = 0;
    static uint32_t prevNum = 0;
    uint32_t progNum;

/*
//	 debug purpose

    static int print=0;
    int parsed = 0;
    int16_t *datasize;
    print++;
    if(print==1000)
    {
    printf("\nRopFrame Header = ");
    for(int i =0; i<4; i++)
        printf("%0x",data[i]);

    parsed = 16;
    printf("Rop:");
    while(parsed < size)
    {
        for(int i =0; i<6; i++, parsed++)
        printf("%0x-", data[parsed]);

        datasize = (int16_t*)&data[parsed];
        printf(" size = %d (%0X%0X)\n", *datasize, data[parsed], data[parsed+1]);
        parsed+= 2+ *datasize;
    }
    printf("END\n");
    print=0;
    }
////////////////////////
*/



#ifdef _DEBUG_ON_FILE_

    // what is txtime??
    if(idx == 0)
    {
        utime[idx] = 0;
        progNum = (txtime >> 32);
        prevTime = (txtime & 0xFFFFFFFF);
        prevNum = progNum;
        idx++;
    }

    else if(idx < max_idx)
    {
        utime[idx] = (txtime & 0xFFFFFFFF);// - prevTime;
        progNum = (txtime >> 32);

        //        if(utime[idx] >= SOGLIA)
        //        	printf("utime - prevTime = %d\n ",utime[idx] - prevTime);
        if( (progNum - prevNum) > 1)
        {
        errors[nErr][0] =  prevNum;
        errors[nErr][1] =  progNum;
        printf("missing packet - old = %d, new = %d\n ",prevNum, progNum);
        printf("missing packet - old = %d, new = %d\n ",errors[nErr][0], errors[nErr][1]);
        nErr++;
        }

        prevTime = (txtime & 0xFFFFFFFF);
        prevNum = progNum;
        idx++;
    }

    if(idx == max_idx)
    {
        outFile = fopen("/usr/local/src/robot/pc104-logs/transceiverTimestamp.txt", "w+");
        printf("Trans fopen: %s\n",strerror(errno));
        if(NULL == outFile)
        {
        outFile = stdout;
        }
        //    	for(uint64_t i=0; i<idx; i++)
        //    		fprintf(outFile, "%d\n", utime[i]);

        fprintf(outFile, "\n\nMissing packets: %d over %d\n\n", nErr, max_idx);
        uint64_t diff;
        for(uint64_t i=0; i< nErr; i++)
        {
        diff = errors[i][1] - errors[i][0];
        fprintf(outFile, "%d -> %d (%d)\n", errors[i][0] , errors[i][1], (uint32_t) diff);
        }
        printf("Trans fprintf: %s\n",strerror(errno));

        if(outFile != stdout)
        fclose(outFile);
        idx++;
    }

#endif
}

/* This function just modify the pointer 'data', in order to point to transceiver's memory where a copy of the ropframe
 * to be sent is stored.
 * The memory holding this ropframe will be written ONLY in case of a new call of eo_transceiver_Transmit function,
 * therefore it is safe to use it. No concurrency is involved here.
 */
void hostTransceiver::getTransmit(uint8_t **data, uint16_t *size)
{
    uint16_t numofrops;
    bytesUsed = 0;
    EOpacket* ptrpkt = NULL;
    eo_transceiver_Transmit(pc104txrx, &ptrpkt, &numofrops);
    // now ptrpkt points to internal tx packet of the transceiver.
    eo_packet_Payload_Get(ptrpkt, data, size);
}

/*
// somebody retrieves what must be transmitted
void hostTransceiver::getTransmit(uint8_t *_data, uint16_t *size) //**data
{
    uint16_t numofrops;
    uint8_t *data; //
    _mutex.wait();
    eo_transceiver_Transmit(pc104txrx, &pktTx, &numofrops);
    eo_packet_Payload_Get(pktTx, &data, size);
    memcpy(data, _data, size);
    _mutex.post();
}
*/

EOnv* hostTransceiver::getNVhandler(uint16_t endpoint, uint16_t id, EOnv *nvRoot)
{
    //	yTrace();
    uint16_t      ondevindex = 0, onendpointindex = 0 , onidindex = 0;
    EOtreenode    *nvTreenodeRoot= NULL;
    eOresult_t    res;

    // convetire come parametro, oppure mettere direttam l'indirizzo ip come parametro...
    eOnvOwnership_t	nvownership = eo_nv_ownership_remote;

//    _mutex.wait();
    res = eo_nvscfg_GetIndices(	this->pc104nvscfg, remoteipaddr, endpoint, id, &ondevindex,	&onendpointindex, &onidindex);

    // if the nvscfg does not have the triple (ip, ep, id) then we return an error
    if(eores_OK != res)
    {
        // Do something about this case?
        nvRoot = NULL;
    }
    else
    {
        // nvRoot MUST be a pointer to an already existing object, created by the caller!!
        nvRoot = eo_nvscfg_GetNV(this->pc104nvscfg, ondevindex, onendpointindex, onidindex, nvTreenodeRoot, nvRoot);
    }
//    _mutex.post();
    return(nvRoot);
}


bool hostTransceiver::getNVvalue(EOnv *nvRoot, uint8_t* data, uint16_t* size)
{
    bool ret;
    if (NULL == nvRoot)
    {   // should return false as error
        return false;
    }
//     transMutex.wait();
    (eores_OK == eo_nv_remoteGet(nvRoot, data, size)) ? ret = true : ret = false;
//     _mutex.post();
    return ret;
}


void hostTransceiver::getHostData(const EOconstvector **pEPvector, eOuint16_fp_uint16_t *pEPhash_function)
{
    *pEPvector = EPvector;
    *pEPhash_function = EPhash_function_ep2index;
}

uint16_t hostTransceiver::getNVnumber(int boardNum, eOnvEP_t ep)
{
//    yTrace();
    const EOconstvector* EPvector = eo_cfg_nvsEP_board_EPs_nvscfgep_get(boardNum);
    uint16_t epindex = eo_cfg_nvsEP_board_EPs_epindex_get(boardNum, ep);
    return eo_cfg_nvsEP_board_NVs_endpoint_numberof_get(EPvector, epindex);
}

uint16_t hostTransceiver::translate_NVid2index(uint8_t boardNum, eOnvEP_t ep, eOnvID_t nvid)
{
    //  yTrace();
    EOconstvector* pEPvector = eo_cfg_nvsEP_board_EPs_nvscfgep_get(boardNum);
    uint16_t epindex = eo_cfg_nvsEP_board_EPs_epindex_get(boardNum, ep);
    eOnvscfg_EP_t* EPcfg = eo_cfg_nvsEP_board_EPs_cfg_get(pEPvector,  epindex);
    return eo_cfg_nvsEP_board_NVs_endpoint_Nvindex_get(EPcfg, nvid);
}


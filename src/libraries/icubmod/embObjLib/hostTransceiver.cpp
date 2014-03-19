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
#include "EOYtheSystem.h"

#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "EoProtocol.h"
#include "Debug.h"

#include <yarp/os/Time.h>

#include "EoProtocol.h"

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
FILE *outFile = NULL;
#endif



#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ //if this macro is defined then ethMenager sends pkts to ems even if they are empty
                                              //ATTENTION: is important to define also the same macro in ethManager.cpp

hostTransceiver::hostTransceiver() : transMutex(1)
{
    yTrace();

    ipport 		 = 0;
    localipaddr  = 0;
    remoteipaddr = 0;

    p_RxPkt     = NULL;
    hosttxrx    = NULL;
    pc104txrx   = NULL;
    nvset       = NULL;
}

hostTransceiver::~hostTransceiver()
{
    yTrace();
}

bool hostTransceiver::init(uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsizerx, uint8_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();

    // 02-may13: so that the newly introduced field sizes contains the EOK_HOSTTRANSCEIVER_* values
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
    hosttxrxcfg.remoteboardipv4addr   = _remoteipaddr;
    hosttxrxcfg.remoteboardipv4port   = _ipport;
 
    eoy_sys_Initialise(NULL, NULL, NULL);

    switch(_board_n)
    {
    case 1:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b01_nvsetDEVcfg;
        break;
    case 2:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b02_nvsetDEVcfg;
        break;
    case 3:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b03_nvsetDEVcfg;
        break;
    case 4:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b04_nvsetDEVcfg;
        break;
    case 5:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b05_nvsetDEVcfg;
        break;
    case 6:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b06_nvsetDEVcfg;
        break;
    case 7:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b07_nvsetDEVcfg;
        break;
    case 8:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b08_nvsetDEVcfg;
        break;
    case 9:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b09_nvsetDEVcfg;
        break;
    case 10:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b10_nvsetDEVcfg;
        break;
    case 11:
        hosttxrxcfg.nvsetdevcfg = &eoprot_b11_nvsetDEVcfg;
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
    pc104txrx    = eo_hosttransceiver_GetTransceiver(hosttxrx);
    if(pc104txrx == NULL)
        return false;

    // retrieve the nvscfg
//    pc104nvscfg  = eo_hosttransceiver_NVsCfg(hosttxrx);
//    if(pc104nvscfg == NULL)
//        return false;
    nvset = eo_hosttransceiver_GetNVset(hosttxrx);
    if(NULL == nvset)
    {
        return false;
    }

//    p_TxPkt = eo_packet_New(_pktsize);
//    if(p_TxPkt == NULL)
//        return false;

    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
        return false;

//    // save board specific configs for later use (they may be needed by some callbacks)
//    EPvector = hosttxrxcfg.vectorof_endpoint_cfg;
//    EPhash_function_ep2index = hosttxrxcfg.hashfunction_ep2index;
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


bool hostTransceiver::addSetMessage__( eOprotID32_t protid, uint8_t* data, uint32_t signature)
{
    EOnv          nv;

    if(protid == EOK_uint16dummy)
    {
        yError() << "eo HostTransceiver: called addSetMessage with invalid nvid ";
        return false;
    }

    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "addSetMessage__: Unable to get pointer to desired NV with protid" << protid;
        return false;
    }

    transMutex.wait();

    if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
    {
        // the nv is not writeable
        yError() << "Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
        transMutex.post();
        return false;
    }

    eOropdescriptor_t ropdesc = {0};

    //ropdesc.control = {0};
    ropdesc.control.plustime = 1;
    ropdesc.ropcode = eo_ropcode_set;
    ropdesc.id32 = protid;
    ropdesc.size = eo_nv_Size(&nv);
    ropdesc.data = data;
    ropdesc.signature = signature;

    transMutex.post();
    bool ret = false;

    for(int i=0; ( (i<5) && (!ret) ); i++)
    {
        transMutex.wait();
        if(eores_OK != eo_transceiver_OccasionalROP_Load(pc104txrx, &ropdesc))
        {
            yWarning() << "addSetMessage: attempt num " << i+1 << ": Error while loading SET ROP with ep "<< eoprot_ID2endpoint(protid) << " entity " << eoprot_ID2entity(protid)  << " index " << eoprot_ID2index(protid)  << " tag " << eoprot_ID2tag(protid) << "in ropframe";
            transMutex.post();
            yarp::os::Time::delay(0.001);

        }
        else
        {
            if(i!=0)
                yWarning() << "addSetMessage: SUCCESS at attempt num " << i+1 << "for loading SET ROP with ep "<< eoprot_ID2endpoint(protid) << " entity " << eoprot_ID2entity(protid)  << " index " << eoprot_ID2index(protid)  << " tag " << eoprot_ID2tag(protid) << "in ropframe";
            transMutex.post();
            ret = true;
        }
    }
    if(!ret)
    {
        yError() << "addSetMessage: Finished attempts!! Error while loading SET ROP with ep "<<eoprot_ID2endpoint(protid) << " entity " << eoprot_ID2entity(protid)  << " index " << eoprot_ID2index(protid)  << " tag " << eoprot_ID2tag(protid) << "in ropframe";
    }
    return ret;
}

bool hostTransceiver::addSetMessage( eOprotID32_t protid, uint8_t* data)
{
   return(hostTransceiver::addSetMessage__(protid, data, 0));
}

bool hostTransceiver::addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig)
{
    return(hostTransceiver::addSetMessage__(protid, data, sig));
}


bool hostTransceiver::addGetMessage( eOprotID32_t protid)
{
    EOnv          nv;

    if(protid == EOK_uint16dummy)
    {
        yError() << "eo HostTransceiver: called addGetMessage with invalid nvid";
        return false;
    }

    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "addGetMessage: Unable to get pointer to desired NV with id" << protid;
        return false;
    }

    eOropdescriptor_t ropdesc = {0};

    //ropdesc.control = {0};
    ropdesc.control.plustime = 1;
    ropdesc.ropcode = eo_ropcode_ask;
    ropdesc.id32 = protid;
    ropdesc.size = 0;
    ropdesc.data = NULL;
    ropdesc.signature = 0;
    

    bool ret = false;

    for(int i=0; ( (i<5) && (!ret) ); i++)
    {
        transMutex.wait();
        if(eores_OK != eo_transceiver_OccasionalROP_Load(pc104txrx, &ropdesc))
        {
            yWarning() << "addGetMessage: attempt num " << i+1 << ": Error while loading GET ROP with ep "<< eoprot_ID2endpoint(protid) << " and nvid " << protid << "in ropframe";
            transMutex.post();
            yarp::os::Time::delay(0.001);
        }
        else
        {
            transMutex.post();
            ret = true;
        }
    }
    if(!ret)
    {
    	yError() << "addGetMessage: Finished attempts!!Error while loading GET ROP with ep "<< eoprot_ID2endpoint(protid)<< " and nvid " << protid << "in ropframe";
    }
    return ret;
}

bool hostTransceiver::readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size)
{
    EOnv nv;
    if(protid == EOK_uint16dummy)
    {
        yError() << "eo HostTransceiver: called readValue with invalid nvid";
        return false;
    }

    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "readBufferedValue: Unable to get pointer to desired NV with id" << protid;
        return false;
    }
    //protetion on reading data by yarp
    transMutex.wait();
    getNVvalue(nv_ptr, data, size);
    transMutex.post();
    return true;
}

bool hostTransceiver::readSentValue(eOprotID32_t protid, uint8_t *data, uint16_t* size)
{
    EOnv nv;
    bool ret;
    if(protid == EOK_uint16dummy)
    {
        yError() << "eo HostTransceiver: called readValue with invalid nvid";
        return false;
    }

    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "readSentValue: Unable to get pointer to desired NV with id" << protid;
        return false;
    }
    //protetion on reading data by yarp
    transMutex.wait();
    (eores_OK == eo_nv_Get(nv_ptr, eo_nv_strg_volatile, data, size)) ? ret = true : ret = false;
    transMutex.post();
    return true;
}

#if 0 // not used, e in ogni caso identica a quella sopra!! (a parte il mutex che cmq ho aggiunto ora ed il tipo del puntatore ai dati)
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
#endif

// somebody passes the received packet - this is used just as an interface
void hostTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    // protezione per la scrittura dei dati all'interno della memoria del transceiver, su ricezione di un rop.
    // il mutex è unico per tutto il transceiver (quindi più endpoint)
    //transMutex.wait();
    eo_packet_Capacity_Get(p_RxPkt, &capacityrxpkt);
    if(size > capacityrxpkt)
    {
        yError () << "received packet has size " << size << "which is higher than capacity of rx pkt = " << capacityrxpkt << "\n";
        return;
    } 

    eo_packet_Payload_Set(p_RxPkt, data, size);
    eo_packet_Addressing_Set(p_RxPkt, remoteipaddr, ipport);
    eo_transceiver_Receive(pc104txrx, p_RxPkt, &numofrops, &txtime);
    //transMutex.post();
}

// and Processes it
void checkDataForDebug(uint8_t *data, uint16_t size)
{
    //  yTrace();

    static uint32_t prevTime = 0;
    static uint32_t prevNum = 0;
    //uint32_t progNum;

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
    EOpacket* ptrpkt = NULL;
    eOresult_t res;
    //is important set size to 0 because if size is 0 pc104 no trasmit data
    *size = 0;
    *data = NULL;



    //RMEOVE
    //eo_transceiver_Transmit(pc104txrx, &ptrpkt, &numofrops);

    //ADD
    res = eo_transceiver_outpacket_Prepare(pc104txrx, &numofrops);
#ifdef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_
    if((eores_OK != res))
#else
    if((eores_OK != res) || (0 == numofrops)) //transmit only if res is ok and there is at least one rop to send
#endif
    {
    	//if I have no rop to send don't send any pkt
    	return;
    }
    res = eo_transceiver_outpacket_Get(pc104txrx, &ptrpkt);
    if(eores_OK != res)
    {
    	return;
    }

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

EOnv* hostTransceiver::getNVhandler(eOprotID32_t protid, EOnv* nv)
{
    eOresult_t    res;
    res = eo_nvset_NV_Get(nvset, remoteipaddr, protid, nv);
    if(eores_OK != res)
    {
        return NULL;
    }

    return(nv);
}


bool hostTransceiver::getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size)
{
    bool ret;
    if (NULL == nv)
    {   // should return false as error
        return false;
    }
//     transMutex.wait();
    (eores_OK == eo_nv_Get(nv, eo_nv_strg_volatile, data, size)) ? ret = true : ret = false;
//     _mutex.post();
    return ret;
}


//void hostTransceiver::getHostData(const EOconstvector **pEPvector, eOuint16_fp_uint16_t *pEPhash_function)
//{
//    *pEPvector = EPvector;
//    *pEPhash_function = EPhash_function_ep2index;
//}

uint16_t hostTransceiver::getNVnumber(int boardNum, eOnvEP8_t ep)
{
    yTrace() << "board num=" << boardNum << " ep=" << ep;
    if(ep < hosttxrxcfg.nvsetdevcfg->vectorof_epcfg->size)
    {
        eOnvset_EPcfg_t *setcfg = (eOnvset_EPcfg_t *)hosttxrxcfg.nvsetdevcfg->vectorof_epcfg->item_array_data;
        return(setcfg[ep].protif->getvarsnumberof(boardNum-1, ep));
    }

    yError() << "cfg size=" <<  hosttxrxcfg.nvsetdevcfg->vectorof_epcfg->size << " ep=" << ep << " board num=" << boardNum;
    return 0;
}

uint32_t hostTransceiver::translate_NVid2index(int boardNum, eOprotID32_t protoid)
{
    return(eoprot_endpoint_id2prognum(boardNum-1, protoid));
}


// eof



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


#undef USE_EOPROT_ROBOT

#if     defined(USE_EOPROT_ROBOT)

#include "eOprot_robot.h"

#else

#include "eOprot_b01.h"
#include "eOprot_b02.h"
#include "eOprot_b03.h"
#include "eOprot_b04.h"
#include "eOprot_b05.h"
#include "eOprot_b06.h"
#include "eOprot_b07.h"
#include "eOprot_b08.h"
#include "eOprot_b09.h"
#include "eOprot_b10.h"
#include "eOprot_b11.h"

#endif

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
FILE *outFile = NULL;
#endif



#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ //if this macro is defined then ethMenager sends pkts to ems even if they are empty
                                              //ATTENTION: is important to define also the same macro in ethManager.cpp

hostTransceiver::hostTransceiver() : transMutex(1)
{
    yTrace();

    ipport 		        = 0;
    localipaddr         = 0;
    remoteipaddr        = 0;

    protboardnumber     = 0;    // however, 0 is a valid board number. thus it must be changed in runtime.
    p_RxPkt             = NULL;
    hosttxrx            = NULL;
    pc104txrx           = NULL;
    nvset               = NULL;
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
}

hostTransceiver::~hostTransceiver()
{
    yTrace();
}


bool hostTransceiver::init(uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsizerx, FEAT_boardnumber_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();

    // marco.accame on 10 apr 2014:
    // eo_hosttransceiver_cfg_default contains the EOK_HOSTTRANSCEIVER_* values which are good for reception of a suitable EOframe
    // hovever, in future it would be fine to be able loading the fields inside eOhosttransceiver_cfg_t from a common eOprot_robot.h
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
    hosttxrxcfg.remoteboardipv4addr     = _remoteipaddr;
    hosttxrxcfg.remoteboardipv4port     = _ipport;
 
    eoy_sys_Initialise(NULL, NULL, NULL);

#if     defined(USE_EOPROT_ROBOT)

    // initialise the protocol for the robot. if already initted by another board it just returns ok
    eoprot_robot_Initialise();
    uint8_t protboardindex = featIdBoardNum2nvBoardNum(_board_n);
    uint8_t numboards = eoprot_robot_DEVcfg_numberof();
    if(protboardindex >= numboards)
    {   // the robot does not have sucha a board
        yError() << "hostTransceiver::init() called w/ invalid _board_n";
        return false;
    }
    hosttxrxcfg.nvsetdevcfg = eoprot_robot_DEVcfg_get(protboardindex);

#else

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
#endif

    localipaddr  = _localipaddr;
    remoteipaddr = _remoteipaddr;
    ipport       = _ipport;

    // initialise the transceiver: it creates a EOtransceiver and its EOnvSet
    hosttxrx     = eo_hosttransceiver_New(&hosttxrxcfg);            // never returns NULL. it calls its error manager
    if(hosttxrx == NULL)
        return false;

    // retrieve the transceiver
    pc104txrx    = eo_hosttransceiver_GetTransceiver(hosttxrx);     
    if(pc104txrx == NULL)
        return false;

    // retrieve the nvset
    nvset = eo_hosttransceiver_GetNVset(hosttxrx);
    if(NULL == nvset)
    {
        return false;
    }

    // retrieve the protboardnumber. the type eOprotBRD_t manages board numbers from 0 upwards.
    protboardnumber = eo_hosttransceiver_GetBoardNumber(hosttxrx);

    // build the packet used for reception.
    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
        return false;


    return true;
}

bool hostTransceiver::nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
{
    if((NULL == nv) || (NULL == dat))
    {
        yError() << "eo HostTransceiver: called nvSetData() with NULL nv or dat";
        return false;
    }  
    
    transMutex.wait();
    bool ret = true;
    if(eores_OK != eo_nv_Set(nv, dat, forceset, upd))
    {
        yError() << "Error while setting NV data\n";
        ret = false;
    }
    transMutex.post();
    return ret;
}

// if signature is eo_rop_SIGNATUREdummy (0xffffffff) we dont send the signature. if writelocalcache is true we copy data into local ram of the EOnv 
bool hostTransceiver::addSetMessage__(eOprotID32_t protid, uint8_t* data, uint32_t signature, bool writelocalrxcache)
{

    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo HostTransceiver: called addSetMessage__() with invalid protid";
        return false;
    }

    if(NULL == data)
    {
        yError() << "eo HostTransceiver: called addSetMessage__() with NULL data";
        return false;
    }
    
    if(true == writelocalrxcache)
    {
        EOnv    nv;
        EOnv*   nv_ptr = NULL;

        nv_ptr = getNVhandler(protid, &nv);

        if(NULL == nv_ptr)
        {
            yError() << "addSetMessage__: Unable to get pointer to desired NV with protid" << protid;
            return false;
        }

        transMutex.wait();

        // marco.accame on 09 apr 2014:
        // we write data into 
        if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
        {
            // the nv is not writeable
            yError() << "Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
            transMutex.post();
            return false;
        }
        
        transMutex.post();
    }

    eOropdescriptor_t ropdesc = {0};
    
    // marco.accame: recommend to use the following to have a basic valid descriptor which is modified later
    memcpy(&ropdesc, &eok_ropdesc_basic, sizeof(eOropdescriptor_t));

    ropdesc.control.plustime    = 1;
    ropdesc.control.plussign    = (eo_rop_SIGNATUREdummy == signature) ? 0 : 1;
    ropdesc.ropcode             = eo_ropcode_set;
    ropdesc.id32                = protid;
    ropdesc.size                = 0;        // marco.accame: the size is internally computed from the id32
    ropdesc.data                = data;
    ropdesc.signature           = signature;

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

bool hostTransceiver::addSetMessage(eOprotID32_t protid, uint8_t* data)
{
   return(hostTransceiver::addSetMessage__(protid, data, eo_rop_SIGNATUREdummy, false));
}

bool hostTransceiver::addSetMessageAndCacheLocally(eOprotID32_t protid, uint8_t* data)
{
   return(hostTransceiver::addSetMessage__(protid, data, eo_rop_SIGNATUREdummy, true));
}

bool hostTransceiver::addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig)
{
    return(hostTransceiver::addSetMessage__(protid, data, sig, false));
}


bool hostTransceiver::addGetMessage(eOprotID32_t protid)
{
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo HostTransceiver: called addGetMessage() with invalid protid";
        return false;
    }


    eOropdescriptor_t ropdesc = {0};
    // marco.accame: recommend to use the following
    memcpy(&ropdesc, &eok_ropdesc_basic, sizeof(eOropdescriptor_t));
    ropdesc.control.plustime    = 1;
    ropdesc.control.plussign    = 0;
    ropdesc.ropcode             = eo_ropcode_ask;
    ropdesc.id32                = protid;
    ropdesc.size                = 0;
    ropdesc.data                = NULL;
    ropdesc.signature           = 0;
    

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
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo HostTransceiver: called readBufferedValue() with invalid protid";
        return false;
    }
    
    if((NULL == data) || (NULL == size))
    {
        yError() << "eo HostTransceiver: called readBufferedValue() with NULL data or size";
        return false;
    }       
    
    EOnv nv;
    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "readBufferedValue: Unable to get pointer to desired NV with id " << protid;
        return false;
    }
    // protection on reading data by yarp
    transMutex.wait();
    getNVvalue(nv_ptr, data, size);
    transMutex.post();
    return true;
}


// use the readSentValue() to retrieve a value previously set into a EOnv with method ::addSetMessage__(protid, data, signature, bool writelocalcache = true).
// take in mind however, that the opration is not clean.
// the ram of EOnv is done to accept values coming from the network. if robot-interface writes data into a EOnv, then a received rop of type say<> or sig<> will
// overwrite the same memory area. we need to re-think the mode with which someone wants to retrieve the last sent value of a EOnv.

bool hostTransceiver::readSentValue(eOprotID32_t protid, uint8_t *data, uint16_t* size)
{
    bool ret = false;
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo HostTransceiver: called readSentValue() with invalid protid";
        return false;
    }

    if((NULL == data) || (NULL == size))
    {
        yError() << "eo HostTransceiver: called readSentValue() with NULL data or size";
        return false;
    }    
    
    EOnv nv;
    EOnv *nv_ptr = getNVhandler(protid, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "readSentValue: Unable to get pointer to desired NV with id " << protid;
        return false;
    }
    // protection on reading data by yarp
    transMutex.wait();
    ret = (eores_OK == eo_nv_Get(nv_ptr, eo_nv_strg_volatile, data, size)) ? true : false;
    transMutex.post();
    return true;
}


// somebody passes the received packet - this is used just as an interface
void hostTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
    if(NULL == data)
    {
        yError() << "eo HostTransceiver::onMsgReception() called with NULL data";
        return;
    } 
    
    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    // protezione per la scrittura dei dati all'interno della memoria del transceiver, su ricezione di un rop.
    // il mutex e' unico per tutto il transceiver
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
    if((NULL == data) || (NULL == size))
    {
        yError() << "eo HostTransceiver::getTransmit() called with NULL data or size";
        return;
    }  

    uint16_t numofrops;
    EOpacket* ptrpkt = NULL;
    eOresult_t res;
    //is important set size to 0 because if size is 0 pc104 no trasmit data
    *size = 0;
    *data = NULL;


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


#define OLDMODE

#ifdef OLDMODE
#include "EOconstvector_hid.h"
#endif

uint16_t hostTransceiver::getNVnumber(eOnvEP8_t ep)
{
/*  marco.accame on 10 apr 2014:
    i had to modify it be cause it would not work in some cases (boards w/ only skin), because the ordering inside hosttxrxcfg.nvsetdevcfg->vectorof_epcfg 
    is not guaranteed to be according to the ep value.
    in boards of blue icub from 1 to 9 it works because the ep value also gives the index of the array.
    but not in eb11,  where the eoprot_endpoint_skin (of value 3) is in position 1. we must use hosttxrxcfg.nvsetdevcfg->fptr_ep2indexofepcfg()
    to compute the index ...  
    much better using the eoprot_endpoint_numberofvariables_get() function instead.
 */

#ifdef OLDMODE    
    uint16_t epi = hosttxrxcfg.nvsetdevcfg->fptr_ep2indexofepcfg(ep);
    if(EOK_uint16dummy == epi)
    {
        yError() << "invalid ep = " <<  ep << " for prot board num = " << protboardnumber;
        return 0;    
    }
    

    eOnvset_EPcfg_t *setcfg = (eOnvset_EPcfg_t *)hosttxrxcfg.nvsetdevcfg->vectorof_epcfg->item_array_data;
    return(setcfg[epi].protif->getvarsnumberof(protboardnumber, ep));

#else
    return(eoprot_endpoint_numberofvariables_get(protboardnumber, ep));
#endif
}

uint32_t hostTransceiver::translate_NVid2index(eOprotID32_t protid)
{
    return(eoprot_endpoint_id2prognum(protboardnumber, protid));
}

eOprotBRD_t hostTransceiver::get_protBRDnumber(void)
{
    return(protboardnumber);
}


// eof




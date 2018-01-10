/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - macros
// --------------------------------------------------------------------------------------------------------------------

#define HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES

// if this macro is defined then ethManager sends pkts even if they dont have ROPs inside
#undef HOSTTRANSCEIVER_EmptyROPframesAreTransmitted

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

#include "EOYmutex.h"
#include "EOYtheSystem.h"
#include "EOtheErrorManager.h"
#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "EoProtocol.h"

#include "ethManager.h"


#include "FeatureInterface.h"



#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"


#include <yarp/os/LogStream.h>

#include <yarp/os/Time.h>



bool HostTransceiver::lock_transceiver(bool on)
{
#if !defined(HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES)
    if(on)
        htmtx->wait();
    else
        htmtx->post();
#endif
    return true;
}




bool HostTransceiver::lock_nvs(bool on)
{
#if !defined(HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES)
    if(on)
        nvmtx->wait();
    else
        nvmtx->post();
#endif
    return true;
}



HostTransceiver::HostTransceiver():delayAfterROPloadingFailure(0.001) // 1ms
{
    yTrace();

//    delayAfterROPloadingFailure = 0.001 * TheEthManager::instance()->getEthSender()->getRate();

    ipport              = 0;
    localipaddr         = 0;
    remoteipaddr        = 0;
    eo_common_ipv4addr_to_string(remoteipaddr, remoteipstring, sizeof(remoteipstring));
    pktsizerx           = 0;

    protboardnumber     = eo_prot_BRDdummy;
    p_RxPkt             = NULL;
    hosttxrx            = NULL;
    pc104txrx           = NULL;
    nvset               = NULL;
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));

    TXrateOfRegularROPs = defTXrateOfRegularROPs;
    cycletime = defcycletime;
    maxtimeRX = defmaxtimeRX;
    maxtimeDO = defmaxtimeDO;
    maxtimeTX = defmaxtimeTX;


    capacityofTXpacket = defMaxSizeOfTXpacket;
    maxSizeOfROP = defMaxSizeOfROP;

#if !defined(HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES)
    htmtx = new Semaphore(1);
    nvmtx = new Semaphore(1);
#endif
}


HostTransceiver::~HostTransceiver()
{
#if !defined(HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES)
    delete htmtx;
    delete nvmtx;
#endif

    if(NULL != p_RxPkt)
    {
        eo_packet_Delete(p_RxPkt);
        p_RxPkt = NULL;
    }
    if(NULL != hosttxrx)
    {
        eo_hosttransceiver_Delete(hosttxrx);
        hosttxrx = NULL;
    }

    // pointers nvset and pc104txrx are just handles retrieved (and deallocated) by object EOhostTransceiver.
    // thus no _Delete() method is required for them
    nvset = NULL;
    pc104txrx = NULL;

    yTrace();
}


bool HostTransceiver::init2(yarp::os::Searchable &cfgEthBoard, eOipv4addressing_t& localIPaddressing, eOipv4addr_t remoteIP, uint16_t rxpktsize)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();


    if(NULL != hosttxrx)
    {
        yError() << "HostTransceiver::init(): called but ... its EOhostTransceiver is already created";
        return false;
    }

    // ok. we can go on. assign values of some member variables

    uint8_t ip4 = 0;
    eo_common_ipv4addr_to_decimal(remoteIP, NULL, NULL, NULL, &ip4);
    protboardnumber = ip4;
    localipaddr     = localIPaddressing.addr;
    remoteipaddr    = remoteIP;
    eo_common_ipv4addr_to_string(remoteipaddr, remoteipstring, sizeof(remoteipstring));
    ipport          = localIPaddressing.port;
    pktsizerx       = rxpktsize;


    if(false == initProtocol())
    {
        yError() << "HostTransceiver::init() -> HostTransceiver::initProtocol() fails";
        return false;
    }


    if(eobool_false == eoprot_board_can_be_managed(protboardnumber))
    {
        yError() << "HostTransceiver::init() -> the BOARD w/ IP " << remoteipstring << "cannot be managed by EOprotocol";
        return false;
    }



    if(!prepareTransceiverConfig2(cfgEthBoard))
    {
        yError() << "HostTransceiver::init() -> HostTransceiver::prepareTransceiverConfig2() fails";
        return false;
    }


    // now hosttxrxcfg is ready, thus ...
    // initialise the transceiver: it creates a EOhostTransceiver and its EOnvSet
    hosttxrx = eo_hosttransceiver_New(&hosttxrxcfg);
    if(hosttxrx == NULL)
    {   // it never returns NULL. on allocation failure it calls its error manager. however ...
        yError() << "HostTransceiver::init(): .... eo_hosttransceiver_New() failed";
        return false;
    }

    // retrieve the transceiver
    pc104txrx = eo_hosttransceiver_GetTransceiver(hosttxrx);
    if(pc104txrx == NULL)
    {
        return false;
    }

    // retrieve the nvset
    nvset = eo_hosttransceiver_GetNVset(hosttxrx);
    if(NULL == nvset)
    {
        return false;
    }



    // build the packet used for reception.
    p_RxPkt = eo_packet_New(rxpktsize);
    if(p_RxPkt == NULL)
    {
        return false;
    }


    return true;
}



//bool HostTransceiver::nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
//{
//    if((NULL == nv) || (NULL == dat))
//    {
//        yError() << "eo HostTransceiver: called nvSetData() with NULL nv or dat";
//        return false;
//    }
    
//    lock_nvs(true);
//    eOresult_t eores = eo_nv_Set(nv, dat, forceset, upd);
//    lock_nvs(false);

//    bool ret = true;
//    if(eores_OK != eores)
//    {
//        yError() << "HostTransceiver::nvSetData(): error while setting NV data w/ eo_nv_Set()\n";
//        ret = false;
//    }

//    return ret;
//}


// if signature is eo_rop_SIGNATUREdummy (0xffffffff) we dont send the signature. if writelocalcache is true we copy data into local ram of the EOnv 
bool HostTransceiver::addSetMessage__(eOprotID32_t id32, uint8_t* data, uint32_t signature, bool writelocalrxcache)
{
#ifdef    ETHRES_DEBUG_DONTREADBACK   // in test beds in which no EMS are connected, just skip this and go on
return true;
#endif
    eOresult_t eores = eores_NOK_generic;
    int32_t err = -1;
    int32_t info0 = -1;
    int32_t info1 = -1;
    int32_t info2 = -1;

    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addSetMessage__() called w/ invalid id on BOARD /w IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }

    if(NULL == data)
    {
        yError() << "HostTransceiver::addSetMessage__() called w/ with NULL data";
        return false;
    }
    
    if(true == writelocalrxcache)
    {
        EOnv    nv;
        EOnv*   nv_ptr = NULL;

        nv_ptr = getNVhandler(id32, &nv);

        if(NULL == nv_ptr)
        {
            yError() << "HostTransceiver::addSetMessage__(): Unable to get pointer to desired NV with id32" << id32;
            return false;
        }

        lock_nvs(true);
        eores = eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo);
        lock_nvs(false);

        // marco.accame on 09 apr 2014:
        // we write data into 
        if(eores_OK != eores)
        {
            // the nv is not writeable
            yError() << "HostTransceiver::addSetMessage__(): Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
            return false;
        }
        
    }

    eOropdescriptor_t ropdesc = {0};
    
    // marco.accame: use eok_ropdesc_basic to have a basic valid descriptor which is modified later
    memcpy(&ropdesc, &eok_ropdesc_basic, sizeof(eOropdescriptor_t));

    ropdesc.control.plustime    = 1;
    ropdesc.control.plussign    = (eo_rop_SIGNATUREdummy == signature) ? 0 : 1;
    ropdesc.ropcode             = eo_ropcode_set;
    ropdesc.id32                = id32;
    ropdesc.size                = 0;        // marco.accame: the size is internally computed from the id32
    ropdesc.data                = data;
    ropdesc.signature           = signature;

    bool ret = false;

    for(int i=0; ( (i<maxNumberOfROPloadingAttempts) && (!ret) ); i++)
    {
        lock_transceiver(true);
        eores = eo_transceiver_OccasionalROP_Load(pc104txrx, &ropdesc);
        lock_transceiver(false);

        if(eores_OK != eores)
        {
            char nvinfo[128];
            eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
            yWarning() << "HostTransceiver::addSetMessage__(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "unsuccessful at attempt num " << i+1 <<
                          "with id: " << nvinfo;

            eo_transceiver_lasterror_tx_Get(pc104txrx, &err, &info0, &info1, &info2);
            yWarning() << "HostTransceiver::addSetMessage__(): eo_transceiver_lasterror_tx_Get() detected: err=" << err << "infos = " << info0 << info1 << info2;

            yarp::os::Time::delay(delayAfterROPloadingFailure);
        }
        else
        {
            if(i!=0)
            {
                char nvinfo[128];
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "HostTransceiver::addSetMessage__(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "successful ONLY at attempt num " << i+1 <<
                              "with id: " << nvinfo;                
            }

            ret = true;
        }
    }
    if(!ret)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addSetMessage__(): ERROR in eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring+1 << "after all attempts" <<
                    "with id: " << nvinfo;
    }

    return ret;
}


bool HostTransceiver::appendSetMessage(eOprotID32_t id32, uint8_t* data)
{
   return(HostTransceiver::addSetMessage__(id32, data, eo_rop_SIGNATUREdummy, false));
}


bool HostTransceiver::addSetMessageAndCacheLocally(eOprotID32_t id32, uint8_t* data)
{
   return(HostTransceiver::addSetMessage__(id32, data, eo_rop_SIGNATUREdummy, true));
}


bool HostTransceiver::addSetMessageWithSignature(eOprotID32_t id32, uint8_t* data, uint32_t sig)
{
    return(HostTransceiver::addSetMessage__(id32, data, sig, false));
}


bool HostTransceiver::addGetMessage__(eOprotID32_t id32, uint32_t signature)
{
    eOresult_t eores = eores_NOK_generic;
    int32_t err = -1;
    int32_t info0 = -1;
    int32_t info1 = -1;
    int32_t info2 = -1;

    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addGetMessage__() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }

    eOropdescriptor_t ropdesc = {0};
    // marco.accame: recommend to use eok_ropdesc_basic
    memcpy(&ropdesc, &eok_ropdesc_basic, sizeof(eOropdescriptor_t));
    ropdesc.control.plustime    = 1;
    ropdesc.control.plussign    = (eo_rop_SIGNATUREdummy == signature) ? 0 : 1;
    ropdesc.ropcode             = eo_ropcode_ask;
    ropdesc.id32                = id32;
    ropdesc.size                = 0;
    ropdesc.data                = NULL;
    ropdesc.signature           = signature;


    bool ret = false;

    for(int i=0; ( (i<maxNumberOfROPloadingAttempts) && (!ret) ); i++)
    {
        lock_transceiver(true);
        eores = eo_transceiver_OccasionalROP_Load(pc104txrx, &ropdesc);
        lock_transceiver(false);

        if(eores_OK != eores)
        {
            char nvinfo[128];
            eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
            yWarning() << "HostTransceiver::addGetMessage__(): eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring<< "unsuccessfull at attempt num " << i+1 <<
                          "with id: " << nvinfo;

            eo_transceiver_lasterror_tx_Get(pc104txrx, &err, &info0, &info1, &info2);
            yWarning() << "HostTransceiver::addGetMessage__(): eo_transceiver_lasterror_tx_Get() detected: err=" << err << "infos = " << info0 << info1 << info2;

            yarp::os::Time::delay(delayAfterROPloadingFailure);
        }
        else
        {
            if(i!=0)
            {
                char nvinfo[128];
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "HostTransceiver::addGetMessage__(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "succesful ONLY at attempt num " << i+1 <<
                              "with id: " << nvinfo;

            }
            ret = true;
        }
    }
    if(!ret)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addGetMessage__(): ERROR in eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring << "after all attempts " <<
                    "with id: " << nvinfo;
    }
    return ret;
}


bool HostTransceiver::appendGetMessage(eOprotID32_t id32)
{
    return(HostTransceiver::addGetMessage__(id32, eo_rop_SIGNATUREdummy));
}


bool HostTransceiver::addGetMessageWithSignature(eOprotID32_t id32, uint32_t signature)
{
    return(HostTransceiver::addGetMessage__(id32, signature));
}


bool HostTransceiver::getBufferedValue(eOprotID32_t id32,  uint8_t *data, uint16_t* size)
{      
    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::readBufferedValue() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }
    
    if((NULL == data) || (NULL == size))
    {
        yError() << "eo HostTransceiver: called readBufferedValue() with NULL data or size";
        return false;
    }       
    
    EOnv nv;
    EOnv *nv_ptr = getNVhandler(id32, &nv);

    if(NULL == nv_ptr)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "readBufferedValue: Unable to get pointer to desired NV with: " << nvinfo;
        return false;
    }

    // marco.accame: protection is inside getNVvalue() member function  
    bool ret = getNVvalue(nv_ptr, data, size);
 

    return ret;
}


// use the readSentValue() to retrieve a value previously set into a EOnv with method ::addSetMessage__(id32, data, signature, bool writelocalcache = true).
// take in mind however, that the opration is not clean.
// the ram of EOnv is done to accept values coming from the network. if robot-interface writes data into a EOnv, then a received rop of type say<> or sig<> will
// overwrite the same memory area. we need to re-think the mode with which someone wants to retrieve the last sent value of a EOnv.

bool HostTransceiver::readSentValue(eOprotID32_t id32, uint8_t *data, uint16_t* size)
{
    bool ret = false;
    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::readSentValue() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }

    if((NULL == data) || (NULL == size))
    {
        yError() << "eo HostTransceiver: called readSentValue() with NULL data or size";
        return false;
    }    
    
    EOnv nv;
    EOnv *nv_ptr = getNVhandler(id32, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "readSentValue: Unable to get pointer to desired NV with id = " << id32;
        return false;
    }
    // protection on reading data by yarp
    lock_nvs(true);
    ret = (eores_OK == eo_nv_Get(nv_ptr, eo_nv_strg_volatile, data, size)) ? true : false;
    lock_nvs(false);
    return true;
}


int HostTransceiver::getCapacityOfRXpacket(void)
{
    return pktsizerx;
}


// somebody passes the received packet - this is used just as an interface
void HostTransceiver::onMsgReception(uint64_t *data, uint16_t size)
{
    if(NULL == data)
    {
        yError() << "eo HostTransceiver::onMsgReception() called with NULL data";
        return;
    } 
    
    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    eo_packet_Capacity_Get(p_RxPkt, &capacityrxpkt);
    if(size > capacityrxpkt)
    {
        yError () << "received packet has size " << size << "which is higher than capacity of rx pkt = " << capacityrxpkt << "\n";
        return;
    } 

    eo_packet_Payload_Set(p_RxPkt, (uint8_t*)data, size);
    eo_packet_Addressing_Set(p_RxPkt, remoteipaddr, ipport);

    // the transceiver can receive and transmit in parallel because reception manipulates memory passed externally
    // and the two operations do not use internal memory shared between the two
    // that is true unless there is a say<> required upon a received ask<>. in such a case the receiver must put the answer inside a data
    // structure read by the transmitter. but the host transceiver does not accept ask<>, thus it is not our case.

    // for the above reason, we could avoid protection.
    // HOWEVER: it is a good thing to protect the nvs as the receiver writes them and someone else reads them to retrieve values for yarp ports
    // for this reason, we use eo_trans_protection_enabled and eo_nvset_protection_one_per_endpoint when we initialise the transceiver.
    // that solves concurrency problems for the transceiver
    eo_transceiver_Receive(pc104txrx, p_RxPkt, &numofrops, &txtime);
}


bool HostTransceiver::isSupported(eOprot_endpoint_t ep)
{
    if(eobool_true == eoprot_endpoint_configured_is(get_protBRDnumber(), ep))
    {
        return true;
    }

    return false;
}


/* This function just modify the pointer 'data', in order to point to transceiver's memory where a copy of the ropframe
 * to be sent is stored.
 * The memory holding this ropframe will be written ONLY in case of a new call of eo_transceiver_Transmit function,
 * therefore it is safe to use it. No concurrency is involved here.
 */
bool HostTransceiver::getTransmit(uint8_t **data, uint16_t *size, uint16_t* numofrops)
{
    // marco.accame on 14oct14: as long as this function is called by one thread only, it is possible to limit the protection to
    //                          only one function: eo_transceiver_outpacket_Prepare().

    if((NULL == data) || (NULL == size) || (NULL == numofrops))
    {
        yError() << "eo HostTransceiver::getTransmit() called with NULL data or zero size or zero numofrops";
        return false;
    }  

    EOpacket* ptrpkt = NULL;
    eOresult_t res;


    // it is important set all these values to NULL and zero because they are used by the caller to decide if it will tx any data or not
    *data = NULL;
    *size = 0;
    *numofrops = 0;

#if !defined(HOSTTRANSCEIVER_EmptyROPframesAreTransmitted)
    // marco.accame: robotInterface uses only occasionals, thus we dont need to pass arguments for replies and regulars
    // moreover: if we passed those pointers with non-NULL values,  we would lock/unlock internal mutex for them, thus we would spend more time
    uint16_t numofoccasionals = 0;
    lock_transceiver(true);
    eo_transceiver_NumberofOutROPs(pc104txrx, NULL, &numofoccasionals, NULL);
    lock_transceiver(false);
    if(0 == numofoccasionals)
    {
        return false;
    }
#endif


    uint16_t tmpnumofrops = 0;

    // it must be protected vs concurrent use of other threads attempting to put rops inside the transceiver.
    lock_transceiver(true);
    res = eo_transceiver_outpacket_Prepare(pc104txrx, &tmpnumofrops, NULL);
    lock_transceiver(false);

#if defined(HOSTTRANSCEIVER_EmptyROPframesAreTransmitted)
    if((eores_OK != res))
#else
    if((eores_OK != res) || (0 == tmpnumofrops)) // transmit only if res is ok and there is at least one rop to send
#endif
    {
        return false;
    }

    // this function does not use data used by concurrent threads, thus it can be left un-protected.
    res = eo_transceiver_outpacket_Get(pc104txrx, &ptrpkt);

    if(eores_OK != res)
    {
        return false;
    }

    // after these two lines, in data, size and numofrops we have what we need
    eo_packet_Payload_Get(ptrpkt, data, size);
    *numofrops = tmpnumofrops;

    if(tmpnumofrops > 0)
        return true;
    else
        return false;
}


EOnv* HostTransceiver::getNVhandler(eOprotID32_t id32, EOnv* nv)
{
    eOresult_t    res;
    res = eo_nvset_NV_Get(nvset, id32, nv);
    if(eores_OK != res)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::getNVhandler() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return NULL;
    }

    return(nv);
}


bool HostTransceiver::getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size)
{
    bool ret;
    if (NULL == nv)
    {   
        yError() << "HostTransceiver::getNVvalue() called w/ NULL nv value: BOARD w/ IP" << remoteipstring;
        return false;
    }
    lock_nvs(true);
    (eores_OK == eo_nv_Get(nv, eo_nv_strg_volatile, data, size)) ? ret = true : ret = false;
    lock_nvs(false);

    if(false == ret)
    {
        yError() << "HostTransceiver::getNVvalue() fails in eo_nv_Get(): BOARD w/ IP" << remoteipstring;
    }
    return ret;
}


uint16_t HostTransceiver::getNVnumber(eOnvEP8_t ep)
{
    return(eoprot_endpoint_numberofvariables_get(protboardnumber, ep));
}

uint32_t HostTransceiver::translate_NVid2index(eOprotID32_t id32)
{
    return(eoprot_endpoint_id2prognum(protboardnumber, id32));
}


eOprotBRD_t HostTransceiver::get_protBRDnumber(void)
{
    return(protboardnumber);
}


eOipv4addr_t HostTransceiver::get_remoteIPaddress(void)
{
    return(remoteipaddr);
}


bool HostTransceiver::initProtocol()
{
    static bool alreadyinitted = false;

//    if(false == alreadyinitted)
//    {
//        // before using embOBJ we need initialing its system. it is better to init it again in case someone did not do it
//        // we use the TheEthManager function ... however that was already called
//        TheEthManager::instance()->initEOYsystem();
//    }

    // reserve resources for board # protboardnumber
    if(eores_OK != eoprot_config_board_reserve(protboardnumber))
    {
        yError() << "HostTransceiver::initProtocol(): call to eoprot_config_board_reserve() fails.";
        return(false);
    }

    if(false == alreadyinitted)
    {
	    // configure all the callbacks of all endpoints.
	
	    eoprot_override_mn();
	    eoprot_override_mc();
	    eoprot_override_as();
	    eoprot_override_sk();

        // ok. all is done correctly
	    alreadyinitted = true;

    }
    else
    {

    }

    return(true);
}



void HostTransceiver::eoprot_override_mn(void)
{
    // marco.accame on 22 mar 2016:
    // i want to keep a minimum of callbacks, thus i have cleaned and put comments about what is needed and why

    static const eOprot_callbacks_endpoint_descriptor_t mn_callbacks_descriptor_endp =
    {
        EO_INIT(.endpoint)          eoprot_endpoint_management,
        EO_INIT(.raminitialise)     NULL
    };

    static const eOprot_callbacks_variable_descriptor_t mn_callbacks_descriptors_vars[] =
    {   // management
        {   // mn_info_status: used for printing sig<> diagnostics ROPs which have strings
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_info,
            EO_INIT(.tag)           eoprot_tag_mn_info_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mn_info_status
        },
        {   // mn_info_status_basic: used for printing sig<> diagnostics ROPs which have compact form (the vast majority)
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_info,
            EO_INIT(.tag)           eoprot_tag_mn_info_status_basic,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mn_info_status_basic
        },
        {   // mn_comm_cmmnds_command_replynumof: used for receiving reply of a command in the form of sig<> ROP
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_comm,
            EO_INIT(.tag)           eoprot_tag_mn_comm_cmmnds_command_replynumof,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mn_comm_cmmnds_command_replynumof
        },
        {   // mn_comm_cmmnds_command_replyarray: used for receiving reply of a command in the form of sig<> ROP
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_comm,
            EO_INIT(.tag)           eoprot_tag_mn_comm_cmmnds_command_replyarray,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mn_comm_cmmnds_command_replyarray
        },
        {   // mn_service_status_commandresult: used for receiving reply of a command in the form of sig<> ROP
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_service,
            EO_INIT(.tag)           eoprot_tag_mn_service_status_commandresult,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mn_service_status_commandresult
        }
    };


    // -----------------------------------------------------------------------------------------------------------------------------------
    // -- initialisation of onsay() function common in MN endpoint for every board

    // function eoprot_fun_ONSAY_mn() is called by any say<> ROP which is received related to the MN endpoint
    // it is used to unlock a waiting mutex
    eoprot_config_onsay_endpoint_set(eoprot_endpoint_management, eoprot_fun_ONSAY_mn);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of MN. common to every board. they perform an action or reception of a specific sig<> ROP.

    int number = sizeof(mn_callbacks_descriptors_vars)/sizeof(mn_callbacks_descriptors_vars[0]);
    for(int i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&mn_callbacks_descriptors_vars[i]);
    }

}


void HostTransceiver::eoprot_override_mc(void)
{
    // marco.accame on 22 mar 2016:
    // i want to keep a minimum of callbacks, thus i have cleaned and put comments about what is needed and why

    static const eOprot_callbacks_endpoint_descriptor_t mc_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_motioncontrol, 
        EO_INIT(.raminitialise)     NULL
    };
    
    static const eOprot_callbacks_variable_descriptor_t mc_callbacks_descriptors_vars[] = 
    { 
        // joint
        {   // joint_status: used to inform the motioncontrol device that a sig<> ROP about joint status has arrived. for .. updating timestamps
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status
        },
        {   // joint_status_basic: used to inform the motioncontrol device that a sig<> ROP about joint status has arrived. for .. updating the same timestamps as above
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status_core,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status_core
        },
        {   // joint_status_basic: used to inform the motioncontrol device that a sig<> ROP about joint status has arrived. for .. updating the same timestamps as above
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status_addinfo_multienc,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status_addinfo_multienc
        }
        // motor
        // nothing so far

        // comment: the functions eoprot_fun_UPDT_mc_joint_status_core() and eoprot_fun_UPDT_mc_joint_status() update the same _encodersStamp[]
        // variables. only one of those two id32 variables are regularly sig<>-led.
        // on the other hand, the same _encodersStamp[] is used for motor encoders ... thus maybe we miss something.
    };



    // -----------------------------------------------------------------------------------------------------------------------------------
    // -- initialisation of onsay() function common in MC endpoint for every board

    // function eoprot_fun_ONSAY_mc() is called by any say<> ROP which is received related to the MC endpoint
    // it is used to unlock a waiting mutex

    eoprot_config_onsay_endpoint_set(eoprot_endpoint_motioncontrol, eoprot_fun_ONSAY_mc);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of MC. common to every board. they perform an action or reception of a specific sig<> ROP.
    
    int number = sizeof(mc_callbacks_descriptors_vars)/sizeof(mc_callbacks_descriptors_vars[0]);
    for(int i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&mc_callbacks_descriptors_vars[i]);
    }
}


void HostTransceiver::eoprot_override_as(void)
{
    // marco.accame on 22 mar 2016:
    // i want to keep a minimum of callbacks, thus i have cleaned and put comments about what is needed and why

    static const eOprot_callbacks_endpoint_descriptor_t as_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_analogsensors, 
        EO_INIT(.raminitialise)     NULL
    };
    
    static const eOprot_callbacks_variable_descriptor_t as_callbacks_descriptors_vars[] = 
    { 
        // strain
        {   // strain_status_calibratedvalues: it gives data from strain to the device, so that it writes it in the relevant yarp port
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_strain,
            EO_INIT(.tag)           eoprot_tag_as_strain_status_calibratedvalues,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_calibratedvalues
        },
        {   // strain_status_uncalibratedvalues: it gives data from strain to the device, so that it writes it in the relevant yarp port
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_strain,
            EO_INIT(.tag)           eoprot_tag_as_strain_status_uncalibratedvalues,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_uncalibratedvalues
        },
        // mais        
        {   // mais_status_the15values: it gives data from mais to the device, so that it writes it in the relevant yarp port
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_mais,
            EO_INIT(.tag)           eoprot_tag_as_mais_status_the15values,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_mais_status_the15values
        },
        // inertial
        {   // eoprot_tag_as_inertial_status: it gives data from mtb to the device, so that it writes it in the relevant yarp port
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_inertial,
            EO_INIT(.tag)           eoprot_tag_as_inertial_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_inertial_status
        }
#if 0   // marco.accame: i keep the code just for the debug phase.       
        ,{   // eoprot_tag_as_inertial_status_accelerometer
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_inertial,
            EO_INIT(.tag)           eoprot_tag_as_inertial_status_accelerometer,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_inertial_status_accelerometer
        },
        {   // eoprot_tag_as_inertial_status_gyroscope
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_inertial,
            EO_INIT(.tag)           eoprot_tag_as_inertial_status_gyroscope,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_inertial_status_gyroscope
        }
#endif        
    };


    // -----------------------------------------------------------------------------------------------------------------------------------
    // -- initialisation of onsay() function common in AS endpoint for every board

    // function eoprot_fun_ONSAY_mc() is called by any say<> ROP which is received related to the AS endpoint
    // it is used to unlock a waiting mutex

    eoprot_config_onsay_endpoint_set(eoprot_endpoint_analogsensors, eoprot_fun_ONSAY_as);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of AS. common to every board. they perform an action or reception of a specific sig<> ROP.
    
    int number = sizeof(as_callbacks_descriptors_vars)/sizeof(as_callbacks_descriptors_vars[0]);
    for(int i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&as_callbacks_descriptors_vars[i]);
    }

}


void HostTransceiver::eoprot_override_sk(void)
{
    // marco.accame on 22 mar 2016:
    // i want to keep a minimum of callbacks, thus i have cleaned and put comments about what is needed and why

    static const eOprot_callbacks_endpoint_descriptor_t sk_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_skin, 
        EO_INIT(.raminitialise)     NULL
    };
    
    static const eOprot_callbacks_variable_descriptor_t sk_callbacks_descriptors_vars[] = 
    { 
        // skin
        {   // skin_status_arrayof10canframes: it gives data from mtb to the device, so that it writes it in the relevant yarp port
            EO_INIT(.endpoint)      eoprot_endpoint_skin,
            EO_INIT(.entity)        eoprot_entity_sk_skin,
            EO_INIT(.tag)           eoprot_tag_sk_skin_status_arrayofcandata,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_sk_skin_status_arrayofcandata
        }    
    };


    // -----------------------------------------------------------------------------------------------------------------------------------
    // -- initialisation of onsay() function common in SK endpoint for every board

    // function eoprot_fun_ONSAY_mc() is called by any say<> ROP which is received related to the SK endpoint
    // it is used to unlock a waiting mutex

    eoprot_config_onsay_endpoint_set(eoprot_endpoint_skin, eoprot_fun_ONSAY_sk);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of SK. common to every board. they perform an action or reception of a specific sig<> ROP.
    
    int number = sizeof(sk_callbacks_descriptors_vars)/sizeof(sk_callbacks_descriptors_vars[0]);
    for(int i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&sk_callbacks_descriptors_vars[i]);
    }

}


void cpp_protocol_callback_incaseoferror_in_sequencenumberReceived(EOreceiver *r)
{  
    const eOreceiver_seqnum_error_t * err = eo_receiver_GetSequenceNumberError(r);
    long long unsigned int exp = err->exp_seqnum;
    long long unsigned int rec = err->rec_seqnum;
    long long unsigned int timeoftxofcurrent = err->timeoftxofcurrent;
    long long unsigned int timeoftxofprevious = err->timeoftxofprevious;
    char *ipaddr = (char*)&err->remipv4addr;
    //printf("\nERROR in sequence number from IP = %d.%d.%d.%d\t Expected: \t%llu,\t received: \t%llu\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3], exp, rec);
    char errmsg[256] = {0};
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::onMsgReception() detected an ERROR in sequence number from IP = %d.%d.%d.%d. Expected: %llu, Received: %llu, Missing: %llu, Prev Frame TX at %llu us, This Frame TX at %llu us",
                                    ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3],
                                    exp, rec, rec-exp,
                                    timeoftxofprevious, timeoftxofcurrent);
    yError() << errmsg;
}


typedef struct
{
    uint32_t            startofframe;       /**< it is the start of the frame: it is EOFRAME_START */
    uint16_t            ropssizeof;         /**< tells how many bytes are reserved for the rops: its value can be 0 to ... */
    uint16_t            ropsnumberof;       /**< tells how many rops are inside: its value can be 0 to ... */
    uint64_t            ageofframe;         /**< tells the time (in usec) of creation of the frame */
    uint64_t            sequencenumber;     /**< contains a sequence number */
} tmpStructROPframeHeader_t;


void cpp_protocol_callback_incaseoferror_invalidFrame(EOreceiver *r)
{
    const eOreceiver_invalidframe_error_t * err = eo_receiver_GetInvalidFrameError(r);
    char errmsg[256] = {0};
    char *ipaddr = (char*)&err->remipv4addr;
    tmpStructROPframeHeader_t *header = (tmpStructROPframeHeader_t*)err->ropframe;
    long long unsigned int ageofframe = header->ageofframe;
    long long unsigned int sequencenumber = header->sequencenumber;
    uint16_t ropframesize = 0;
    eo_ropframe_Size_Get(err->ropframe, &ropframesize);
    //snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::onMsgReception() detected an ERROR of type INVALID FRAME from IP = TBD");
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::onMsgReception() detected an ERROR of type INVALID FRAME from IP = %d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
    yError() << errmsg;
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::onMsgReception() detected: ropframesize = %d, ropsizeof = %d, ropsnumberof = %d, ageoframe = %llu, sequencenumber = %llu", ropframesize, header->ropssizeof, header->ropsnumberof, ageofframe, sequencenumber);
    yDebug() << errmsg;

}


bool HostTransceiver::prepareTransceiverConfig2(yarp::os::Searchable &cfgEthBoard)
{
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
    hosttxrxcfg.remoteboardipv4addr     = remoteipaddr;
    hosttxrxcfg.remoteboardipv4port     = ipport;


    Bottle groupEthBoardProperties = Bottle(cfgEthBoard.findGroup("ETH_BOARD_PROPERTIES"));
    if(groupEthBoardProperties.isNull())
    {
        yError() << "HostTransceiver::method() cannot find ETH_BOARD_PROPERTIES group in config files for BOARD w/ IP" << remoteipstring;
        return NULL;
    }


    Bottle groupEthBoardSettings = Bottle(cfgEthBoard.findGroup("ETH_BOARD_SETTINGS"));
    if(groupEthBoardSettings.isNull())
    {
        yError() << "HostTransceiver::method() cannot find ETH_BOARD_SETTINGS group in config files for BOARD w/ IP" << remoteipstring;
        return NULL;
    }
    else
    {
        TXrateOfRegularROPs = defTXrateOfRegularROPs;
        cycletime = defcycletime;
        maxtimeRX = defmaxtimeRX;
        maxtimeDO = defmaxtimeDO;
        maxtimeTX = defmaxtimeTX;

        Bottle groupEthBoardSettings_RunningMode = Bottle(groupEthBoardSettings.findGroup("RUNNINGMODE"));
        if(groupEthBoardSettings_RunningMode.isNull())
        {
            yWarning() << "HostTransceiver::method(): cannot find ETH_BOARD_PROPERTIES/RUNNINGMODE group in config files for BOARD w/ IP" << remoteipstring << " and will use default values";
            yWarning() << "Default values for ETH_BOARD_PROPERTIES/RUNNINGMODE group: (period, maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity, TXrateOfRegularROPs) = " <<
                          cycletime << maxtimeRX << maxtimeDO << maxtimeTX << TXrateOfRegularROPs;
        }
        else
        {

            if(true == groupEthBoardSettings_RunningMode.check("period"))
            {
                int tmp = groupEthBoardSettings_RunningMode.find("period").asInt();

                if(1000 != tmp)
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::period can be only 1000 (so far) and it was:" << tmp;
                    tmp = 1000;
                }
                cycletime = tmp;
            }

            if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfRXactivity"))
            {
                int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfRXactivity").asInt();

                if((tmp < 5) || (tmp > 990))
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxTimeOfRXactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                    tmp = defmaxtimeRX;
                }

                maxtimeRX = tmp;
            }

            if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfDOactivity"))
            {
                int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfDOactivity").asInt();

                if((tmp < 5) || (tmp > 990))
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxtimeOfDOactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                    tmp = defmaxtimeDO;
                }

                maxtimeDO = tmp;
            }

            if(true == groupEthBoardSettings_RunningMode.check("maxTimeOfTXactivity"))
            {
                int tmp = groupEthBoardSettings_RunningMode.find("maxTimeOfTXactivity").asInt();

                if((tmp < 5) || (tmp > 990))
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::maxTimeOfTXactivity must be in [5, 990] (so far) and it was:" << tmp << "Using default value";
                    tmp = defmaxtimeTX;
                }

                maxtimeTX = tmp;
            }


            if(true == groupEthBoardSettings_RunningMode.check("TXrateOfRegularROPs"))
            {
                int tmp = groupEthBoardSettings_RunningMode.find("TXrateOfRegularROPs").asInt();

                if(tmp <=0)
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::TXrateOfRegularROPs must be in [1, 20] and it was:" << tmp << "Using value = 1";
                    tmp = 1;
                }
                if(tmp >200)
                {
                    yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "ETH_BOARD_SETTINGS::RUNNINGMODE::TXrateOfRegularROPs must be in [1, 20] and it was:" << tmp << "Using value = 20";
                    tmp = 20;
                }
                TXrateOfRegularROPs = tmp;
            }

            // consistency check
            if((maxtimeRX+maxtimeDO+maxtimeTX) != cycletime)
            {
                yWarning() << "HostTransceiver::method() for BOARD" << remoteipstring << "In ETH_BOARD_SETTINGS::RUNNINGMODE sum(maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity) != period !!! Using default values";

                yError() << maxtimeRX+maxtimeDO+maxtimeTX;
                cycletime = defcycletime;
                maxtimeRX = defmaxtimeRX;
                maxtimeDO = defmaxtimeDO;
                maxtimeTX = defmaxtimeTX;
            }

            yInfo() << "Found group ETH_BOARD_PROPERTIES/RUNNINGMODE with values: (period, maxTimeOfRXactivity, maxTimeOfDOactivity, maxTimeOfTXactivity, TXrateOfRegularROPs) = " <<
                        cycletime << maxtimeRX << maxtimeDO << maxtimeTX << TXrateOfRegularROPs;

        }
    }


    if(true == groupEthBoardProperties.check("maxSizeRXpacket"))
    {
        capacityofTXpacket = groupEthBoardProperties.find("maxSizeRXpacket").asInt();
        yDebug() << "HostTransceiver::prepareTransceiverConfig2() has detected capacityofTXpacket =" << capacityofTXpacket << "for BOARD w/ IP" << remoteipstring;
    }

    if(true == groupEthBoardProperties.check("maxSizeROP"))
    {
        maxSizeOfROP = groupEthBoardProperties.find("maxSizeROP").asInt();
        yDebug() << "HostTransceiver::prepareTransceiverConfig2() has detected maxSizeOfROP =" << maxSizeOfROP << "for BOARD w/ IP" << remoteipstring;
    }



    // we build the hosttransceiver so that:
    // 1. it can send a packet which can always be received by the board (max tx size = max rx size of remote board)
    // 2. it has the same maxsize of rop as remote board
    // 3. it has no regulars, no space for replies, and maximum space for occasionals.
    // the properties of remote board are inside remoteTransceiverProperties and are taken from the xml file.
    // after the transceiver is built and communication can happen, we shall verify if remoteTransceiverProperties has the same values
    // of what is read from remote board. see funtion ethResources::verifyBoardTransceiver().

    hosttxrxcfg.sizes.capacityoftxpacket            = capacityofTXpacket;
    hosttxrxcfg.sizes.capacityofrop                 = maxSizeOfROP;
    hosttxrxcfg.sizes.capacityofropframeregulars    = eo_ropframe_sizeforZEROrops;
    hosttxrxcfg.sizes.capacityofropframereplies     = eo_ropframe_sizeforZEROrops;
    hosttxrxcfg.sizes.capacityofropframeoccasionals = (hosttxrxcfg.sizes.capacityoftxpacket - eo_ropframe_sizeforZEROrops) - hosttxrxcfg.sizes.capacityofropframeregulars - hosttxrxcfg.sizes.capacityofropframereplies;
    hosttxrxcfg.sizes.maxnumberofregularrops        = 0;


    // ok, now the nvset ... we use maximum capabilities so that we can manage communication of up to 12 jomos
    const eOnvset_BRDcfg_t* brdcf2use = &eonvset_BRDcfgMax;


    memcpy(&nvsetbrdconfig, brdcf2use, sizeof(eOnvset_BRDcfg_t));
    nvsetbrdconfig.boardnum = get_protBRDnumber();

    hosttxrxcfg.nvsetbrdcfg = &nvsetbrdconfig;
    hosttxrxcfg.extfn.onerrorseqnumber = cpp_protocol_callback_incaseoferror_in_sequencenumberReceived;
    hosttxrxcfg.extfn.onerrorinvalidframe = cpp_protocol_callback_incaseoferror_invalidFrame;


#if !defined(HOSTTRANSCEIVER_USE_INTERNAL_MUTEXES)
    hosttxrxcfg.mutex_fn_new = NULL;
    hosttxrxcfg.transprotection = eo_trans_protection_none;
    hosttxrxcfg.nvsetprotection = eo_nvset_protection_none;
#else
    // mutex protection inside transceiver
    hosttxrxcfg.mutex_fn_new = (eov_mutex_fn_mutexderived_new) eoy_mutex_New;
    hosttxrxcfg.transprotection = eo_trans_protection_enabled; // eo_trans_protection_none
    hosttxrxcfg.nvsetprotection = eo_nvset_protection_one_per_endpoint; // eo_nvset_protection_one_per_object // eo_nvset_protection_none
#endif

    return(true);
}



// eof




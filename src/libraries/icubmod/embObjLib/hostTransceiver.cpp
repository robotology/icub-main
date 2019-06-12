/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
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
#include "ethParser.h"

#include "FeatureInterface.h"



#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"


#include <yarp/os/LogStream.h>

#include <yarp/os/Time.h>

using namespace eth;

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

//    delayAfterROPloadingFailure = TheEthManager::instance()->getEthSender()->getPeriod();

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


bool HostTransceiver::init2(AbstractEthResource *owner, yarp::os::Searchable &cfgtotal, eOipv4addressing_t& localIPaddressing, eOipv4addr_t remoteIP, uint16_t rxpktsize)
{
    if(NULL != hosttxrx)
    {
        yError() << "HostTransceiver::init(): called but ... its EOhostTransceiver is already created";
        return false;
    }

    if(nullptr == owner)
    {
        yError() << "HostTransceiver::init2(): called w/ nullptr";
    }


    _owner = owner;

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



    if(!prepareTransceiverConfig2(cfgtotal))
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

eOipv4addr_t HostTransceiver::getIPv4()
{
    return remoteipaddr;
}

AbstractEthResource * HostTransceiver::getResource()
{
    return _owner;
}


bool HostTransceiver::write(const eOprotID32_t id32, const void* data, bool forcewriteOfReadOnly)
{
    eOresult_t eores = eores_NOK_generic;

    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::write() called w/ invalid id on BOARD /w IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }

    if(NULL == data)
    {
        yError() << "HostTransceiver::write() called w/ with NULL data";
        return false;
    }


    EOnv nv;
    EOnv* nv_ptr = NULL;

    nv_ptr = getnvhandler(id32, &nv);

    if(NULL == nv_ptr)
    {
        yError() << "HostTransceiver::write(): Unable to get pointer to desired NV with id32" << id32;
        return false;
    }


    lock_nvs(true);
    eores = eo_nv_Set(&nv, data, forcewriteOfReadOnly, eo_nv_upd_dontdo);
    lock_nvs(false);

    // marco.accame on 09 apr 2014:
    // we write data into
    if(eores_OK != eores)
    {
        bool ROvariable = (eo_nv_rwmode_RO == eo_nv_GetRWmode(&nv)) ? true : false;
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::write(): eo_nv_Set() has failed for IP" << remoteipstring << "and" << nvinfo << "RO mode = " << ((ROvariable) ? "true." : "false.");

        if((eobool_false == forcewriteOfReadOnly) && (true == ROvariable))
        {
            yError() << "HostTransceiver::write(): called with forcewriteOfReadOnly = false for a READ ONLY variable";

        }
        return false;
    }

    return true;
}


// if signature is eo_rop_SIGNATUREdummy (0xffffffff) we dont send the signature. if writelocalcache is true we copy data into local ram of the EOnv 
bool HostTransceiver::addSetROP__(const eOprotID32_t id32, const void* data, const uint32_t signature, bool writelocalrxcache)
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
        yError() << "HostTransceiver::addSetROP__() called w/ invalid id on BOARD /w IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }

    if(NULL == data)
    {
        yError() << "HostTransceiver::addSetROP__() called w/ with NULL data";
        return false;
    }
    
    if(true == writelocalrxcache)
    {
        EOnv    nv;
        EOnv*   nv_ptr = NULL;

        nv_ptr = getnvhandler(id32, &nv);

        if(NULL == nv_ptr)
        {
            yError() << "HostTransceiver::addSetROP__(): Unable to get pointer to desired NV with id32" << id32;
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
            yError() << "HostTransceiver::addSetROP__(): Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
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
    ropdesc.data                = reinterpret_cast<uint8_t *>(const_cast<void*>(data));
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
            yWarning() << "HostTransceiver::addSetROP__(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "unsuccessful at attempt num " << i+1 <<
                          "with id: " << nvinfo;

            eo_transceiver_lasterror_tx_Get(pc104txrx, &err, &info0, &info1, &info2);
            yWarning() << "HostTransceiver::addSetROP__(): eo_transceiver_lasterror_tx_Get() detected: err=" << err << "infos = " << info0 << info1 << info2;

            yarp::os::Time::delay(delayAfterROPloadingFailure);
        }
        else
        {
            if(i!=0)
            {
                char nvinfo[128];
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "HostTransceiver::addSetROP__(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "successful ONLY at attempt num " << i+1 <<
                              "with id: " << nvinfo;                
            }

            ret = true;
        }
    }
    if(!ret)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addSetROP__(): ERROR in eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring+1 << "after all attempts" <<
                    "with id: " << nvinfo;
    }

    return ret;
}



bool HostTransceiver::addROPset(const eOprotID32_t id32, const void* data, const uint32_t signature)
{
   return(HostTransceiver::addSetROP__(id32, data, signature, false));
}


bool HostTransceiver::isID32supported(const eOprotID32_t id32)
{
    return (eobool_false == eoprot_id_isvalid(protboardnumber, id32)) ? false : true;
}

bool HostTransceiver::addROPask(const eOprotID32_t id32, const uint32_t signature)
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
        yError() << "HostTransceiver::addROPask() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
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
            yWarning() << "HostTransceiver::addROPask(): eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring<< "unsuccessfull at attempt num " << i+1 <<
                          "with id: " << nvinfo;

            eo_transceiver_lasterror_tx_Get(pc104txrx, &err, &info0, &info1, &info2);
            yWarning() << "HostTransceiver::addROPask(): eo_transceiver_lasterror_tx_Get() detected: err=" << err << "infos = " << info0 << info1 << info2;

            yarp::os::Time::delay(delayAfterROPloadingFailure);
        }
        else
        {
            if(i!=0)
            {
                char nvinfo[128];
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "HostTransceiver::addROPask(): eo_transceiver_OccasionalROP_Load() for BOARD /w IP" << remoteipstring << "succesful ONLY at attempt num " << i+1 <<
                              "with id: " << nvinfo;

            }
            ret = true;
        }
    }
    if(!ret)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::addROPask(): ERROR in eo_transceiver_OccasionalROP_Load() for BOARD w/ IP" << remoteipstring << "after all attempts " <<
                    "with id: " << nvinfo;
    }

    return ret;
}



bool HostTransceiver::read(const eOprotID32_t id32, void *data)
{      
    if(eobool_false == eoprot_id_isvalid(protboardnumber, id32))
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::read() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
                    "with id: " << nvinfo;
        return false;
    }
    
    if(NULL == data)
    {
        yError() << "HostTransceiver:read() called w/ NULL data";
        return false;
    }       
    
    EOnv nv;
    EOnv *nv_ptr = getnvhandler(id32, &nv);

    if(NULL == nv_ptr)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::read() Unable to get pointer to desired NV with: " << nvinfo;
        return false;
    }

    uint16_t size = 0;
    // marco.accame: protection is inside getNVvalue() member function  
    bool ret = getNVvalue(nv_ptr, reinterpret_cast<uint8_t *>(data), &size);
 

    return ret;
}



// somebody passes the received packet - this is used just as an interface
bool HostTransceiver::parseUDP(const void *data, const uint16_t size)
{
    if(NULL == data)
    {
        yError() << "eo HostTransceiver::parse() called with NULL data";
        return false;
    } 

    if(size > pktsizerx)
    {
        yError() << "eo HostTransceiver::parse() called too big a packet: max size is" << pktsizerx;
        return false;
    }
    
    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    eo_packet_Capacity_Get(p_RxPkt, &capacityrxpkt);
    if(size > capacityrxpkt)
    {
        yError () << "received packet has size " << size << "which is higher than capacity of rx pkt = " << capacityrxpkt << "\n";
        return false;
    } 

    eo_packet_Payload_Set(p_RxPkt, reinterpret_cast<uint8_t*>(const_cast<void*>(data)), size);
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

    return true;
}


bool HostTransceiver::isEPsupported(const eOprot_endpoint_t ep)
{
    if(eobool_true == eoprot_endpoint_configured_is(get_protBRDnumber(), ep))
    {
        return true;
    }

    return false;
}


const void * HostTransceiver::getUDP(size_t &size, uint16_t &numofrops)
{
    const void * udpframe = nullptr;
    size = 0;
    numofrops = 0;

    // marco.accame on 14oct14: as long as this function is called by one thread only, it is possible to limit the protection to
    //                          only one function: eo_transceiver_outpacket_Prepare().


    EOpacket* ptrpkt = NULL;
    uint8_t *data = NULL;
    eOresult_t res;


#if !defined(HOSTTRANSCEIVER_EmptyROPframesAreTransmitted)
    // marco.accame: robotInterface uses only occasionals, thus we dont need to pass arguments for replies and regulars
    // moreover: if we passed those pointers with non-NULL values,  we would lock/unlock internal mutex for them, thus we would spend more time
    uint16_t numofoccasionals = 0;
    lock_transceiver(true);
    eo_transceiver_NumberofOutROPs(pc104txrx, NULL, &numofoccasionals, NULL);
    lock_transceiver(false);
    if(0 == numofoccasionals)
    {
        return nullptr;
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
        return nullptr;
    }

    // this function does not use data used by concurrent threads, thus it can be left un-protected.
    res = eo_transceiver_outpacket_Get(pc104txrx, &ptrpkt);

    if(eores_OK != res)
    {
        return nullptr;
    }

    // after these two lines, in data, size and numofrops we have what we need
    uint16_t tmpsize = 0;
    eo_packet_Payload_Get(ptrpkt, &data, &tmpsize);

    size = tmpsize;
    numofrops = tmpnumofrops;
    udpframe = (tmpnumofrops > 0) ? data : nullptr;


    return udpframe;
}


EOnv* HostTransceiver::getnvhandler(eOprotID32_t id32, EOnv* nv)
{
    eOresult_t    res;
    res = eo_nvset_NV_Get(nvset, id32, nv);
    if(eores_OK != res)
    {
        char nvinfo[128];
        eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
        yError() << "HostTransceiver::getnvhandler() called w/ invalid protid: BOARD w/ IP" << remoteipstring <<
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
        },        // inertial3
        {   // eoprot_tag_as_inertial3_status: ...
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_inertial3,
            EO_INIT(.tag)           eoprot_tag_as_inertial3_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_inertial3_status
        },
        {   // eoprot_tag_as_temperature_status: ...
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_temperature,
            EO_INIT(.tag)           eoprot_tag_as_temperature_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_temperature_status
        },
        {   // eoprot_tag_as_psc_status: ...
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_psc,
            EO_INIT(.tag)           eoprot_tag_as_psc_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_psc_status
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

    // function eoprot_fun_ONSAY_as() is called by any say<> ROP which is received related to the AS endpoint
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
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::parse() detected an ERROR in sequence number from IP = %d.%d.%d.%d. Expected: %llu, Received: %llu, Missing: %llu, Prev Frame TX at %llu us, This Frame TX at %llu us",
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
    //snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::parse() detected an ERROR of type INVALID FRAME from IP = TBD");
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::parse() detected an ERROR of type INVALID FRAME from IP = %d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
    yError() << errmsg;
    snprintf(errmsg, sizeof(errmsg), "hostTransceiver()::parse() detected: ropframesize = %d, ropsizeof = %d, ropsnumberof = %d, ageoframe = %llu, sequencenumber = %llu", ropframesize, header->ropssizeof, header->ropsnumberof, ageofframe, sequencenumber);
    yDebug() << errmsg;

}


bool HostTransceiver::prepareTransceiverConfig2(yarp::os::Searchable &cfgtotal)
{
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
    hosttxrxcfg.remoteboardipv4addr     = remoteipaddr;
    hosttxrxcfg.remoteboardipv4port     = ipport;


    eth::parser::pc104Data pc104data;
    eth::parser::read(cfgtotal, pc104data);
//    eth::parser::print(pc104data);


    eth::parser::boardData brddata;
    eth::parser::read(cfgtotal, brddata);
//    eth::parser::print(brddata);

    capacityofTXpacket = brddata.properties.maxSizeRXpacket;
    maxSizeOfROP = brddata.properties.maxSizeROP;



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




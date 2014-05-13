/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - macros
// --------------------------------------------------------------------------------------------------------------------

#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ //if this macro is defined then ethMenager sends pkts to ems even if they are empty
                                              //ATTENTION: is important to define also the same macro in ethManager.cpp



#if defined(USE_EOPROT_OLD) | defined(USE_EOPROT_XML)
    //#warning --> keeping USE_EOPROT_xxx from cmakelist
#else
    //#warning --> specifying USE_EOPROT_xxx by hand 
    #define USE_EOPROT_OLD
    //#define USE_EOPROT_XML
#endif


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
#include "EOtheErrorManager.h"
#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "EoProtocol.h"



#if defined(USE_EOPROT_OLD)

//#warning --> using USE_EOPROT_OLD

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

#elif   defined(USE_EOPROT_XML)

//#warning --> using USE_EOPROT_XML

#include "EOprotocolConfigurator.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"

#else
    #error --> chose a USE_EOPROT_xxx amongst: USE_EOPROT_OLD, USE_EOPROT_XML
#endif

#include "Debug.h"

#include <yarp/os/Time.h>



hostTransceiver::hostTransceiver() : transMutex(1)
{
    yTrace();

    ipport              = 0;
    localipaddr         = 0;
    remoteipaddr        = 0;

    protboardnumber     = eo_prot_BRDdummy;
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


bool hostTransceiver::init(yarp::os::Searchable &config, uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsizerx, FEAT_boardnumber_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();


    if(NULL != hosttxrx)
    {
        yError() << "hostTransceiver::init(): called but ... its EOhostTransceiver is already created";
        return false; 
    }

    // ok. we can go on. assign values of some member variables

    protboardnumber = featIdBoardNum2nvBoardNum(_board_n);
    localipaddr     = _localipaddr;
    remoteipaddr    = _remoteipaddr;
    ipport          = _ipport; 

#if     defined(USE_EOPROT_OLD)
        yWarning() << "hostTransceiver::init() -> using USE_EOPROT_OLD";
#elif   defined(USE_EOPROT_XML)
        yWarning() << "hostTransceiver::init() -> using USE_EOPROT_XML";
#endif

    if(!initProtocol(config))
    {
        yError() << "hostTransceiver::init() -> hostTransceiver::initProtocol() fails";
        return false;     
    }


    if(eobool_false == eoprot_board_can_be_managed(protboardnumber))
    {
        yError() << "hostTransceiver::init() -> the board " << protboardnumber+1 << "cannot be managed by EOprotocol";
        return false; 
    }


    if(!prepareTransceiverConfig(config))
    {
        yError() << "hostTransceiver::init() -> hostTransceiver::prepareTransceiverConfig() fails";
        return false;     
    }

    // now hosttxrxcfg is ready, thus ...
    // initialise the transceiver: it creates a EOhostTransceiver and its EOnvSet
    hosttxrx = eo_hosttransceiver_New(&hosttxrxcfg);            // never returns NULL. it calls its error manager
    if(hosttxrx == NULL)
    {
        yError() << "hostTransceiver::init(): .... eo_hosttransceiver_New() failed";
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
    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
    {
        return false;
    }


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
        yError() << "hostTransceiver::nvSetData(): error while setting NV data w/ eo_nv_Set()\n";
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
        yError() << "hostTransceiver::addSetMessage__() called w/ invalid protid: protboard = " << protboardnumber <<
                    ", ep = " << eoprot_ID2endpoint(protid) << ", entity = " << eoprot_ID2entity(protid) << "index = " << eoprot_ID2index(protid) <<
                    ", tag = " << eoprot_ID2tag(protid);
        return false;
    }

    if(NULL == data)
    {
        yError() << "hostTransceiver::addSetMessage__() called w/ with NULL data";
        return false;
    }
    
    if(true == writelocalrxcache)
    {
        EOnv    nv;
        EOnv*   nv_ptr = NULL;

        nv_ptr = getNVhandler(protid, &nv);

        if(NULL == nv_ptr)
        {
            yError() << "hostTransceiver::addSetMessage__(): Unable to get pointer to desired NV with protid" << protid;
            return false;
        }

        transMutex.wait();

        // marco.accame on 09 apr 2014:
        // we write data into 
        if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
        {
            // the nv is not writeable
            yError() << "hostTransceiver::addSetMessage__(): Maybe you are trying to write a read-only variable? (eo_nv_Set failed)";
            transMutex.post();
            return false;
        }
        
        transMutex.post();
    }

    eOropdescriptor_t ropdesc = {0};
    
    // marco.accame: use eok_ropdesc_basic to have a basic valid descriptor which is modified later
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
            yWarning() << "hostTransceiver::addSetMessage__(): eo_transceiver_OccasionalROP_Load() unsuccessfull at attempt num " << i+1 << 
                          " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                          ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
            transMutex.post();
            yarp::os::Time::delay(0.001);

        }
        else
        {
            if(i!=0)
            {
                yWarning() << "hostTransceiver::addSetMessage__(): eo_transceiver_OccasionalROP_Load() succesful ONLY at attempt num " << i+1 << 
                              " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                              ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
                
            }
            transMutex.post();
            ret = true;
        }
    }
    if(!ret)
    {
        yError() << "hostTransceiver::addSetMessage__(): ERROR in eo_transceiver_OccasionalROP_Load() after all attempts" <<
                    " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                    ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
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
        yError() << "hostTransceiver::addGetMessage() called w/ invalid protid: protboard = " << protboardnumber <<
                    ", ep = " << eoprot_ID2endpoint(protid) << ", entity = " << eoprot_ID2entity(protid) << "index = " << eoprot_ID2index(protid) <<
                    ", tag = " << eoprot_ID2tag(protid);
        return false;
    }


    eOropdescriptor_t ropdesc = {0};
    // marco.accame: recommend to use eok_ropdesc_basic
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
            yWarning() << "hostTransceiver::addGetMessage__(): eo_transceiver_OccasionalROP_Load() unsuccessfull at attempt num " << i+1 << 
                          " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                          ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
            transMutex.post();
            yarp::os::Time::delay(0.001);
        }
        else
        {
            if(i!=0)
            {
                yWarning() << "hostTransceiver::addGetMessage__(): eo_transceiver_OccasionalROP_Load() succesful ONLY at attempt num " << i+1 << 
                              " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                              ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
                
            }
            transMutex.post();
            ret = true;
        }
    }
    if(!ret)
    {
        yError() << "hostTransceiver::addGetMessage__(): ERROR in eo_transceiver_OccasionalROP_Load() after all attempts" <<
                    " with: ep = " << eoprot_ID2endpoint(protid)  << ", entity = " << eoprot_ID2entity(protid)  << 
                    ", index = " << eoprot_ID2index(protid)  << ", tag = " << eoprot_ID2tag(protid);
    }
    return ret;
}


bool hostTransceiver::readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size)
{      
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "hostTransceiver::readBufferedValue() called w/ invalid protid: protboard = " << protboardnumber <<
                    ", ep = " << eoprot_ID2endpoint(protid) << ", entity = " << eoprot_ID2entity(protid) << "index = " << eoprot_ID2index(protid) <<
                    ", tag = " << eoprot_ID2tag(protid);
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
        yError() << "hostTransceiver::readSentValue() called w/ invalid protid: protboard = " << protboardnumber <<
                    ", ep = " << eoprot_ID2endpoint(protid) << ", entity = " << eoprot_ID2entity(protid) << "index = " << eoprot_ID2index(protid) <<
                    ", tag = " << eoprot_ID2tag(protid);
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
        yError() << "readSentValue: Unable to get pointer to desired NV with id = " << protid;
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
        yError() << "hostTransceiver::getNVhandler() called w/ invalid protid: protboard = " << protboardnumber <<
                    ", ep = " << eoprot_ID2endpoint(protid) << ", entity = " << eoprot_ID2entity(protid) << "index = " << eoprot_ID2index(protid) <<
                    ", tag = " << eoprot_ID2tag(protid);
        return NULL;
    }

    return(nv);
}


bool hostTransceiver::getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size)
{
    bool ret;
    if (NULL == nv)
    {   
    yError() << "hostTransceiver::getNVvalue() called w/ NULL nv value: protboard = " << protboardnumber;
        return false;
    }
//     transMutex.wait();
    (eores_OK == eo_nv_Get(nv, eo_nv_strg_volatile, data, size)) ? ret = true : ret = false;
//     _mutex.post();

    if(false == ret)
    {
        yError() << "hostTransceiver::getNVvalue() fails in eo_nv_Get(): protboard = " << protboardnumber;
    }
    return ret;
}


uint16_t hostTransceiver::getNVnumber(eOnvEP8_t ep)
{
    return(eoprot_endpoint_numberofvariables_get(protboardnumber, ep));
}

uint32_t hostTransceiver::translate_NVid2index(eOprotID32_t protid)
{
    return(eoprot_endpoint_id2prognum(protboardnumber, protid));
}

eOprotBRD_t hostTransceiver::get_protBRDnumber(void)
{
    return(protboardnumber);
}


bool hostTransceiver::initProtocol(yarp::os::Searchable &config)
{
    static bool alreadyinitted = false;

    if(false == alreadyinitted)
    {
        // before using embOBJ we need initialing its system. it is better to init it again in case someone did not do it
        eoy_sys_Initialise(NULL, NULL, NULL);

        static const uint8_t numOfBoardsinRobot =  eoprot_boards_maxnumberof; // to be initialise later or w/ a proper number from XML ...

        // init the protocol to manage all boards of the robot
        if(eores_OK != eoprot_config_board_numberof(numOfBoardsinRobot))
        {
            yError() << "hostTransceiver::initProtocol(): call to eoprot_config_board_numberof() fails.";
            return(false);
        }

#if     defined(USE_EOPROT_OLD)
    
        // nothing to initialise

#elif   defined(USE_EOPROT_XML)

	
	    // configure all the callbacks of all endpoints.
	
	    eoprot_override_mn();
	    eoprot_override_mc();
	    eoprot_override_as();
	    eoprot_override_sk();
#endif
        // ok. all is done correctly
	    alreadyinitted = true;

        // yWarning() << "hostTransceiver::initProtocol() CALLED W/ INITIALISATION";
    }
    else
    {
        // yWarning() << "hostTransceiver::initProtocol() CALLED W/OUT INITIALISATION";
    }

    return(true);
}



void hostTransceiver::eoprot_override_mn(void)
{   // nothing to do  

}

void hostTransceiver::eoprot_override_mc(void)
{
#if     defined(USE_EOPROT_OLD)    
        // nothing to do
#elif   defined(USE_EOPROT_XML)

    static const eOprot_callbacks_endpoint_descriptor_t mc_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_motioncontrol, 
        EO_INIT(.raminitialise)     NULL // or any xxeoprot_fun_INITIALISE_mc 
    };
    
    static const eOprot_callbacks_variable_descriptor_t mc_callbacks_descriptors_vars[] = 
    { 
        // joint
        {   // joint_config
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config
        },
        {   // joint_config_pidposition
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_pidposition,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_pidposition
        },
        {   // joint_config_impedance
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_impedance,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_impedance
        },
        {   // joint_config_pidtorque
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_pidtorque,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_pidtorque
        },
        {   // joint_status
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status
        },
        {   // joint_status_basic
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status_basic,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status_basic
        },
        {   // joint_cmmnds_setpoint
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_cmmnds_setpoint,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_cmmnds_setpoint
        },
        {   // joint_config_limitsofjoint
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_limitsofjoint,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_limitsofjoint
        },
        // motor
        {   // motor_config
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_config,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_config
        },        
        {   // motor_config_maxcurrentofmotor
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_config_maxcurrentofmotor,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor
        },
        {   // motor_status_basic
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_status_basic,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_status_basic
        }                
    };


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- general ram initialise of mc endpoint called by every board.
    
    // we dont do any general initialisation, even if we could do it with a xxeoprot_fun_INITIALISE_mc() function
    //eoprot_config_callbacks_endpoint_set(&mc_callbacks_descriptor_endp);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of mc. common to every board. we use the id, even if the eoprot_config_variable_callback()
    //    operates on any index.
    
    uint32_t number = sizeof(mc_callbacks_descriptors_vars)/sizeof(mc_callbacks_descriptors_vars[0]);
    uint32_t i = 0;
    
    for(i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&mc_callbacks_descriptors_vars[i]);
    }
#endif
}


void hostTransceiver::eoprot_override_as(void)
{
#if     defined(USE_EOPROT_OLD)    
        // nothing to do
#elif   defined(USE_EOPROT_XML)
    static const eOprot_callbacks_endpoint_descriptor_t as_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_analogsensors, 
        EO_INIT(.raminitialise)     NULL // or any xxeoprot_fun_INITIALISE_as 
    };
    
    static const eOprot_callbacks_variable_descriptor_t as_callbacks_descriptors_vars[] = 
    { 
        // strain
        {   // strain_status_calibratedvalues
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_strain,
            EO_INIT(.tag)           eoprot_tag_as_strain_status_calibratedvalues,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_calibratedvalues
        },
        {   // strain_status_uncalibratedvalues
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_strain,
            EO_INIT(.tag)           eoprot_tag_as_strain_status_uncalibratedvalues,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_uncalibratedvalues
        },
        // mais
        {   // mais_status_the15values
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_mais,
            EO_INIT(.tag)           eoprot_tag_as_mais_status_the15values,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_as_mais_status_the15values
        }           
    };


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- general ram initialise of as endpoint called by every board.
    
    // we dont do any general initialisation, even if we could do it with a xxeoprot_fun_INITIALISE_as() function
    //eoprot_config_callbacks_endpoint_set(&as_callbacks_descriptor_endp);



    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of mc. common to every board. we use the id, even if the eoprot_config_variable_callback()
    //    operates on any index.
    
    uint32_t number = sizeof(as_callbacks_descriptors_vars)/sizeof(as_callbacks_descriptors_vars[0]);
    uint32_t i = 0;
    
    for(i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&as_callbacks_descriptors_vars[i]);
    }
#endif   
}


void hostTransceiver::eoprot_override_sk(void)
{
#if     defined(USE_EOPROT_OLD)    
        // nothing to do
#elif   defined(USE_EOPROT_XML)
    static const eOprot_callbacks_endpoint_descriptor_t sk_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_skin, 
        EO_INIT(.raminitialise)     NULL // or any xxeoprot_fun_INITIALISE_sk 
    };
    
    static const eOprot_callbacks_variable_descriptor_t sk_callbacks_descriptors_vars[] = 
    { 
        // skin
        {   // strain_status_calibratedvalues
            EO_INIT(.endpoint)      eoprot_endpoint_skin,
            EO_INIT(.entity)        eoprot_entity_sk_skin,
            EO_INIT(.tag)           eoprot_tag_sk_skin_status_arrayof10canframes,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        eoprot_fun_UPDT_sk_skin_status_arrayof10canframes
        }    
    };


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- general ram initialise of sk endpoint called by every board.
    
    // we dont do any general initialisation, even if we could do it with a xxeoprot_fun_INITIALISE_sk() function
    //eoprot_config_callbacks_endpoint_set(&sk_callbacks_descriptor_endp);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of mc. common to every board. we use the id, even if the eoprot_config_variable_callback()
    //    operates on any index.
    
    uint32_t number = sizeof(sk_callbacks_descriptors_vars)/sizeof(sk_callbacks_descriptors_vars[0]);
    uint32_t i = 0;
    
    for(i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&sk_callbacks_descriptors_vars[i]);
    }
#endif
}


void cpp_protocol_callback_incaseoferror_in_sequencenumberReceived(uint32_t remipv4addr, uint64_t rec_seqnum, uint64_t expected_seqnum)
{  
    long long unsigned int exp = expected_seqnum;
    long long unsigned int rec = rec_seqnum;
    printf("Error in sequence number from 0x%x!!!! \t Expected %llu, received %llu\n", remipv4addr, exp, rec);
};

//extern "C" {
//extern void protocol_callback_incaseoferror_in_sequencenumberReceived(uint32_t remipv4addr, uint64_t rec_seqnum, uint64_t expected_seqnum);
//}


bool hostTransceiver::prepareTransceiverConfig(yarp::os::Searchable &config)
{
    // hosttxrxcfg is a class member ...

    // marco.accame on 10 apr 2014:
    // eo_hosttransceiver_cfg_default contains the EOK_HOSTTRANSCEIVER_* values which are good for reception of a suitable EOframe
    // hovever, in future it would be fine to be able loading the fields inside eOhosttransceiver_cfg_t from a common eOprot_robot.h
    memcpy(&hosttxrxcfg, &eo_hosttransceiver_cfg_default, sizeof(eOhosttransceiver_cfg_t));
    hosttxrxcfg.remoteboardipv4addr     = remoteipaddr;
    hosttxrxcfg.remoteboardipv4port     = ipport;    

    // the nvsetcfg of the board ...
    hosttxrxcfg.nvsetdevcfg             = getNVset_DEVcfg(config);

    if(NULL == hosttxrxcfg.nvsetdevcfg)
    {
        return(false);
    }



    // other configurable parameters for eOhosttransceiver_cfg_t
    // - mutex_fn_new, transprotection, nvsetprotection are left (NULL, eo_trans_protection_none, eo_nvset_protection_none) 
    //   as in default because we dont protect internally w/ a mutex
    // - confmancfg is left NULL as in default because we dont use a confirmation manager.
    
    // marco.accame on 29 apr 2014: so that the EOreceiver calls this funtion in case of error in sequence number
    hosttxrxcfg.extfn.onerrorseqnumber = cpp_protocol_callback_incaseoferror_in_sequencenumberReceived;


    return(true);
}




const eOnvset_DEVcfg_t * hostTransceiver::getNVset_DEVcfg(yarp::os::Searchable &config)
{

    const eOnvset_DEVcfg_t* nvsetdevcfg = NULL;

#if     defined(USE_EOPROT_OLD)

    int _board_n = nvBoardNum2FeatIdBoardNum(get_protBRDnumber());

    switch(_board_n)
    {
    case 1:
        nvsetdevcfg = &eoprot_b01_nvsetDEVcfg;
        break;
    case 2:
        nvsetdevcfg = &eoprot_b02_nvsetDEVcfg;
        break;
    case 3:
        nvsetdevcfg = &eoprot_b03_nvsetDEVcfg;
        break;
    case 4:
        nvsetdevcfg = &eoprot_b04_nvsetDEVcfg;
        break;
    case 5:
        nvsetdevcfg = &eoprot_b05_nvsetDEVcfg;
        break;
    case 6:
        nvsetdevcfg = &eoprot_b06_nvsetDEVcfg;
        break;
    case 7:
        nvsetdevcfg = &eoprot_b07_nvsetDEVcfg;
        break;
    case 8:
        nvsetdevcfg = &eoprot_b08_nvsetDEVcfg;
        break;
    case 9:
        nvsetdevcfg = &eoprot_b09_nvsetDEVcfg;
        break;
    case 10:
        nvsetdevcfg = &eoprot_b10_nvsetDEVcfg;
        break;
    case 11:
        nvsetdevcfg = &eoprot_b11_nvsetDEVcfg;
        break;
    default:
        yError() << "Got a non existing board number" << _board_n;
        //return NULL;
        break;
    }

#elif   defined(USE_EOPROT_XML)

    nvsetdevcfg = NULL;

    eOprotconfig_cfg_t protcfg;
    memcpy(&protcfg, &eo_protconfig_cfg_default, sizeof(eOprotconfig_cfg_t));
    // ok, now i get the values from config and run ...

    // at least ... this one
    protcfg.board                           = get_protBRDnumber();
    
    if(config.isNull())
    {
        yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: entire PROTOCOL group ... using max capabilities";
        //return false;
    }
    else
    {
        int      number;

        if(false == config.check("endpointManagementIsSupported"))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire MN endpoint ... enabled w/ max capabilities" <<
                          " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
        }
        else if((false == config.check("entityMNcommunicationNumberOf")) || (false == config.check("entityMNapplicationNumberOf")))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some MN entities ... using max capabilities" <<
                          " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
        } 
        else
        {   
            // ok, retrieve the numbers ...  
            number  = config.find("endpointManagementIsSupported").asInt();              
            protcfg.ep_management_is_present                = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_management_is_present)
            {
                protcfg.en_mn_entity_comm_numberof          = config.find("entityMNcommunicationNumberOf").asInt();
                protcfg.en_mn_entity_appl_numberof          = config.find("entityMNapplicationNumberOf").asInt();  
            }
            else
            {
                protcfg.en_mn_entity_comm_numberof          = 0;
                protcfg.en_mn_entity_appl_numberof          = 0;
            }
            // sanity check
            if((protcfg.en_mn_entity_comm_numberof > 1) || (protcfg.en_mn_entity_appl_numberof > 1))
            {
                yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of MN entities"  <<
                              " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
            }
        
        }


        if(false == config.check("endpointMotionControlIsSupported"))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire MC endpoint ... enabled w/ max capabilities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
        }
        else if((false == config.check("entityMCjointNumberOf")) || (false == config.check("entityMCmotorNumberOf")) || 
                (false == config.check("entityMCmotorNumberOf")))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some MC entities ... using max capabilities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointMotionControlIsSupported").asInt();              
            protcfg.ep_motioncontrol_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_motioncontrol_is_present)
            {
                protcfg.en_mc_entity_joint_numberof          = config.find("entityMCjointNumberOf").asInt();
                protcfg.en_mc_entity_motor_numberof          = config.find("entityMCmotorNumberOf").asInt();  
                protcfg.en_mc_entity_controller_numberof     = config.find("entityMCcontrollerNumberOf").asInt();
            }
            else
            {
                protcfg.en_mc_entity_joint_numberof          = 0;
                protcfg.en_mc_entity_motor_numberof          = 0;  
                protcfg.en_mc_entity_controller_numberof     = 0;
            }
            // sanity check
            if((protcfg.en_mc_entity_joint_numberof > 16) || (protcfg.en_mc_entity_motor_numberof > 16) ||
               (protcfg.en_mc_entity_controller_numberof > 1))
            {
                yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of MC entities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
            }
        }


        if(false == config.check("endpointAnalogSensorsIsSupported"))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire AS endpoint ... enabled w/ max capabilities" <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
        }
        else if((false == config.check("entityASstrainNumberOf")) || (false == config.check("entityASmaisNumberOf")) || 
                (false == config.check("entityASextorqueNumberOf")))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some AS entities ... using max capabilities"  <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointAnalogSensorsIsSupported").asInt();              
            protcfg.ep_analogsensors_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_analogsensors_is_present)
            {
                protcfg.en_as_entity_strain_numberof        = config.find("entityASstrainNumberOf").asInt();
                protcfg.en_as_entity_mais_numberof          = config.find("entityASmaisNumberOf").asInt();  
                protcfg.en_as_entity_extorque_numberof      = config.find("entityASextorqueNumberOf").asInt();
            }
            else
            {
                protcfg.en_as_entity_strain_numberof        = 0;
                protcfg.en_as_entity_mais_numberof          = 0;  
                protcfg.en_as_entity_extorque_numberof      = 0;
            }
            // sanity check
            if((protcfg.en_as_entity_strain_numberof > 1) || (protcfg.en_as_entity_mais_numberof > 1) ||
               (protcfg.en_as_entity_extorque_numberof > 16))
            {
                yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of AS entities"  <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
            }
        }


        if(false == config.check("endpointSkinIsSupported"))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire SK endpoint ... enabled w/ max capabilities"  <<
                          " (skin) = (" << protcfg.en_sk_entity_skin_numberof << ")";
        }
        else if((false == config.check("entitySKskinNumberOf")))
        {
            yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some SK entities ... using max capabilities"   <<
                          " (skin) = (" << protcfg.en_sk_entity_skin_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointSkinIsSupported").asInt();              
            protcfg.ep_skin_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_skin_is_present)
            {
                protcfg.en_sk_entity_skin_numberof        = config.find("entitySKskinNumberOf").asInt();
            }
            else
            {
                protcfg.en_sk_entity_skin_numberof        = 0;
            }
            // sanity check
            if((protcfg.en_sk_entity_skin_numberof > 2))
            {
                yWarning() << "hostTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of SK entities"   <<
                          " (skin) = (" << protcfg.en_sk_entity_skin_numberof << ")";
            }

        }

        printf("\nprotcfg --> bdr %d, mn = %d, mc = %d, as = %d, sk = %d ... co = %d, ap = %d, jn = %d, mt = %d, ct = %d, st = %d, ma = %d, ex = %d, sk = %d\n",
                        protcfg.board,
                        protcfg.ep_management_is_present,   protcfg.ep_motioncontrol_is_present, protcfg.ep_analogsensors_is_present, protcfg.ep_skin_is_present,
                        protcfg.en_mn_entity_comm_numberof, protcfg.en_mn_entity_appl_numberof,
                        protcfg.en_mc_entity_joint_numberof, protcfg.en_mc_entity_motor_numberof, protcfg.en_mc_entity_controller_numberof,
                        protcfg.en_as_entity_strain_numberof, protcfg.en_as_entity_mais_numberof, protcfg.en_as_entity_extorque_numberof,
                        protcfg.en_sk_entity_skin_numberof
            );


    }

    

    nvsetdevcfg = eo_protconfig_DEVcfg_Get(eo_protconfig_New(&protcfg));


#else
    #error --> define a USE_EOPROT_xxx
#endif

    
    if(NULL == nvsetdevcfg)
    {
        yError() << "hostTransceiver::getNVset_DEVcfg() -> FAILS as it produces a NULL result";   
    }
    
    return(nvsetdevcfg);
}



// eof




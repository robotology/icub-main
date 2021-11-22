/*
 * Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - macros
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

using namespace std;

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "boardTransceiver.hpp"
#include "FeatureInterface.h"

#include "EOYtheSystem.h"
#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "EoProtocol.h"


#include "EOdeviceTransceiver.h"

#include "EOprotocolConfigurator.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"



#include <yarp/os/LogStream.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

using namespace yarp::os;


EOnvSet* arrayofnvsets[16] = {NULL};

BoardTransceiver::BoardTransceiver()
{
    yTrace();

    UDP_socket          = NULL;
    remoteipport        = 0;
    localipaddr         = 0;
    remoteipaddr        = 0;

    protboardnumber     = eo_prot_BRDdummy;
    p_RxPkt             = NULL;
    devtxrx             = NULL;
    transceiver         = NULL;
    nvset               = NULL;

    pApplStatus         = NULL;

    oneNV               = eo_nv_New();

    memcpy(&devtxrxcfg, &eo_devicetransceiver_cfg_default, sizeof(eOdevicetransceiver_cfg_t));       
}

BoardTransceiver::~BoardTransceiver()
{
    yTrace();
}

bool BoardTransceiver::configure(yarp::os::ResourceFinder &rf)
{
    yDebug() << " input is " << rf.toString().c_str();

    if(!rf.check("PC104IpAddress"))
    {
        yError() << "missing PC104IpAddress";
        return false;
    }

    if(!rf.check("emsIpAddress"))
    {
        yError() << "missing emsIpAddress";
        return false;
    }

    if(!rf.check("port"))
    {
        yError() << "missing port";
        return false;
    }

    yDebug() << " I have all params I need!!";
    Bottle parameter1( rf.find("PC104IpAddress").asString() );

    int port      = rf.find("port").asInt32();              // .get(1).asInt32();
    strcpy(_fId.PC104ipAddr.string, parameter1.toString().c_str());
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( rf.find("emsIpAddress").asString() );    // .findGroup("IpAddress");
    strcpy(_fId.EMSipAddr.string, parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    snprintf(_fId.EMSipAddr.string, sizeof(_fId.EMSipAddr.string), "%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    snprintf(_fId.PC104ipAddr.string, sizeof(_fId.PC104ipAddr.string), "%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

    // Check input parameters
    yDebug() << " boardTransceiver - referred to EMS: " << _fId.EMSipAddr.string;


    ACE_UINT32 hostip = (_fId.PC104ipAddr.ip1 << 24) | (_fId.PC104ipAddr.ip2 << 16) | (_fId.PC104ipAddr.ip3 << 8) | (_fId.PC104ipAddr.ip4);
    ACE_INET_Addr myIP((u_short)_fId.PC104ipAddr.port, hostip);
//    myIP.dump();

    if(!createSocket(myIP) )
    {  return NULL;  }

    return true;
}

bool BoardTransceiver::createSocket(ACE_INET_Addr local_addr)
{
    yTrace();

    UDP_socket = new ACE_SOCK_Dgram();
    char tmp[64];

    if(-1 == UDP_socket->open(local_addr))
    {
        local_addr.addr_to_string(tmp, 64);
        yError() <<   "\n/---------------------------------------------------\\"
                 <<   "\n| Unable to bind to local IP address " << tmp
                 <<   "\n\\---------------------------------------------------/";
        delete UDP_socket;
        UDP_socket = NULL;
    }
    else
    {
        yTrace() << "BoardTransceiver created socket correctly!";
    }

    return true;
}

bool BoardTransceiver::init(yarp::os::Searchable &config, uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _remoteipport, uint16_t _pktsizerx, FEAT_boardnumber_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();

    // assign values of some member variables

    protboardnumber = featIdBoardNum2nvBoardNum(_board_n);
    localipaddr     = _localipaddr;
    remoteipaddr    = _remoteipaddr;
    remoteipport    = _remoteipport; 



    if(!initProtocol(config))
    {
        yError() << "BoardTransceiver::initProtocol() fails";
        return false;     
    }


    if(!prepareTransceiverConfig(config))
    {
        yError() << "BoardTransceiver::prepareTransceiverConfig() fails";
        return false;     
    }

    // now devtxrxcfg is ready, thus ...
    // initialise the transceiver: it creates a EOtransceiver and its EOnvSet
    devtxrx     = eo_devicetransceiver_New(&devtxrxcfg);            // never returns NULL. it calls its error manager  
    if(devtxrx == NULL)
        return false;

    // retrieve the transceiver
    transceiver    = eo_devicetransceiver_GetTransceiver(devtxrx);       //CHECK!!!!!
    if(transceiver == NULL)
        return false;

    // retrieve the nvset
    nvset = eo_devicetransceiver_GetNVset(devtxrx);                 //CHECK!!!!!
    if(NULL == nvset)
    {
        return false;
    }

    arrayofnvsets[protboardnumber] = nvset;


    // build the packet used for reception.
    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
        return false;

    eOnvID32_t id32status = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_appl, 0, eoprot_tag_mn_appl_status);
    eo_nvset_NV_Get(nvset, eok_ipv4addr_localhost, id32status, oneNV);

    pApplStatus = (eOmn_appl_status_t*) oneNV->ram;
    pApplStatus->currstate = applstate_config;
    


    return true;
}


void fromDouble(ACE_Time_Value &v, double x,int unit) {
#ifdef YARP_HAS_ACE
        v.msec(static_cast<int>(x*1000/unit+0.5));
#else
        v.tv_usec = static_cast<int>(x*1000000/unit+0.5) % 1000000;
        v.tv_sec = static_cast<int>(x/unit);
#endif
}

// Main loop here!!!
bool BoardTransceiver::updateModule()
{
    ACE_INET_Addr   sender_addr;
    ssize_t         recv_size;
    ACE_Time_Value  recvTimeOut;
    fromDouble(recvTimeOut, 1.0);

    uint8_t         *p_sendData;
    uint16_t        bytes_to_send = 0;

    uint8_t         incoming_msg[RECV_BUFFER_SIZE];

    bool parsepacket = false;
    bool transmitpacket = false; 

    //std::cout << " BoardTransceiver is running happily!" << std::endl;

    // get pkt from socket: blocking call with timeout
    recv_size = UDP_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);

    switch(pApplStatus->currstate)
    {
        case applstate_config:
        {   // in configuration state the ems is triggered only for non-empty packets. it sends back a ropframe even if empty
            if(recv_size > 0)       
            {
                parsepacket = true;
                transmitpacket = true;
            }
        } break;

        case applstate_running:
        {   // in running state the ems is triggered every millisecond. it parses non-empty packets. it always sends a ropframe back, even if empty. 
            if(recv_size > 0)       
            {
                parsepacket = true;
            }
            transmitpacket = true;
        } break;

        case applstate_error:
        {
            if(recv_size > 0)       
            {
                parsepacket = true;
                transmitpacket = true;
            }
        } break;
        
        default:
        {

        } break;

    }

    if(parsepacket)
    {
        onMsgReception(incoming_msg, recv_size);
    }

    if(transmitpacket)
    {
        getTransmit(&p_sendData, &bytes_to_send);

        ssize_t ret = UDP_socket->send(p_sendData, bytes_to_send, remoteipaddr);
        if(ret < 0)
        {
            yError() << "Unable to send a message";
        }
    }

    return true;   // if false the program quits from the forever loop
}



// somebody passes the received packet - this is used just as an interface
void BoardTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
    if(NULL == data)
    {
        yError() << "eo BoardTransceiver::onMsgReception() called with NULL data";
        return;
    } 
    
    yError() << "Received a message with size = " << size;


    uint16_t numofrops;
    uint64_t txtime;
    uint16_t capacityrxpkt = 0;

    // protezione per la scrittura dei dati all'interno della memoria del transceiver, su ricezione di un rop.
    // il mutex e' unico per tutto il transceiver
    eo_packet_Capacity_Get(p_RxPkt, &capacityrxpkt);
    if(size > capacityrxpkt)
    {
        yError () << "received packet has size " << size << "which is higher than capacity of rx pkt = " << capacityrxpkt << "\n";
        return;
    } 

    eo_packet_Payload_Set(p_RxPkt, data, size);
    eo_packet_Addressing_Set(p_RxPkt, remoteipaddr, remoteipport);
    eo_transceiver_Receive(transceiver, p_RxPkt, &numofrops, &txtime);
}



/* This function just modify the pointer 'data', in order to point to transceiver's memory where a copy of the ropframe
 * to be sent is stored.
 * The memory holding this ropframe will be written ONLY in case of a new call of eo_transceiver_Transmit function,
 * therefore it is safe to use it. No concurrency is involved here.
 */
void BoardTransceiver::getTransmit(uint8_t **data, uint16_t *size)
{
    if((NULL == data) || (NULL == size))
    {
        yError() << "eo BoardTransceiver::getTransmit() called with NULL data or size";
        return;
    }  

    uint16_t numofrops;
    EOpacket* ptrpkt = NULL;
    eOresult_t res;
    
    *size = 0;
    *data = NULL;


    res = eo_transceiver_outpacket_Prepare(transceiver, &numofrops);
    if(eores_OK != res)
    {
        yError() << "eo BoardTransceiver::getTransmit(): failure of eo_transceiver_outpacket_Prepare()";
    	return;
    }

    res = eo_transceiver_outpacket_Get(transceiver, &ptrpkt);
    if(eores_OK != res)
    {
        yError() << "eo BoardTransceiver::getTransmit(): failure of eo_transceiver_outpacket_Get()";
    	return;
    }

    // now ptrpkt points to internal tx packet of the transceiver.
    eo_packet_Payload_Get(ptrpkt, data, size);
}



eOprotBRD_t BoardTransceiver::get_protBRDnumber(void)
{
    return(protboardnumber);
}


void embOBJerror(eOerrmanErrorType_t errtype, eOid08_t taskid, const char *eobjstr, const char *info)
{
    static const char* theerrors[] = { "eo_errortype_info", "eo_errortype_warning", "eo_errortype_weak", "eo_errortype_fatal" }; 

    yError() << "embOBJerror(): errtype = " << theerrors[errtype] << "from EOobject = " << eobjstr << " w/ message = " << info;

    if(errtype == eo_errortype_fatal)
    {
        yError() << "embOBJerror(): FATAL ERROR: the calling thread shall now be stopped in a forever loop here inside";
        for(;;);
    }

}

void eoy_initialise(void)
{
    // marco.accame: in here we init the embOBJ system for YARP.
    eOerrman_cfg_t errmanconfig = {0};
    errmanconfig.extfn.usr_on_error        = embOBJerror;
    const eOysystem_cfg_t *syscfg       = NULL;
    const eOmempool_cfg_t *mpoolcfg     = NULL;     // uses standard mode 
    //const eOerrman_cfg_t *errmancf      = NULL;     // uses default mode
    eoy_sys_Initialise(syscfg, mpoolcfg, &errmanconfig);
}

bool BoardTransceiver::initProtocol(yarp::os::Searchable &config)
{
    static bool alreadyinitted = false;

    if(false == alreadyinitted)
    {
        // before using embOBJ we need initialing its system. it is better to init it again in case someone did not do it
        eoy_initialise();

        static const uint8_t numOfBoardsinRobot =  eoprot_boards_maxnumberof; // to be initialise later or w/ a proper number from XML ...

        // init the protocol to manage all boards of the robot
        if(eores_OK != eoprot_config_board_numberof(numOfBoardsinRobot))
        {
            yError() << "BoardTransceiver::initProtocol(): call to eoprot_config_board_numberof() fails.";
            return(false);
        }

	    // configure all the callbacks of all endpoints.
	
	    eoprot_override_mn();
	    eoprot_override_mc();
	    eoprot_override_as();
	    eoprot_override_sk();

        // ok. all is done correctly
	    alreadyinitted = true;

        // yWarning() << "BoardTransceiver::initProtocol() CALLED W/ INITIALISATION";
    }
    else
    {
        // yWarning() << "BoardTransceiver::initProtocol() CALLED W/OUT INITIALISATION";
    }

    return(true);
}

extern "C" {
void boardtransceiver_fun_UPDT_mn_appl_cmmnds_go2state(const EOnv* nv, const eOropdescriptor_t* rd);
}

void BoardTransceiver::eoprot_override_mn(void)
{   // nothing to do  

    static const eOprot_callbacks_endpoint_descriptor_t mn_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_management, 
        EO_INIT(.raminitialise)     NULL // or any xxeoprot_fun_INITIALISE_mn 
    };
    
    static const eOprot_callbacks_variable_descriptor_t mn_callbacks_descriptors_vars[] = 
    { 
        // appl
        {   // 
            EO_INIT(.endpoint)      eoprot_endpoint_management,
            EO_INIT(.entity)        eoprot_entity_mn_appl,
            EO_INIT(.tag)           eoprot_tag_mn_appl_cmmnds_go2state,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        boardtransceiver_fun_UPDT_mn_appl_cmmnds_go2state
        }    
    };


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- general ram initialise of sk endpoint called by every board.
    
    // we dont do any general initialisation, even if we could do it with a xxeoprot_fun_INITIALISE_sk() function
    //eoprot_config_callbacks_endpoint_set(&sk_callbacks_descriptor_endp);


    // ------------------------------------------------------------------------------------------------------------------------------------
    // -- override of the callbacks of variables of mc. common to every board. we use the id, even if the eoprot_config_variable_callback()
    //    operates on any index.
    
    uint32_t number = sizeof(mn_callbacks_descriptors_vars)/sizeof(mn_callbacks_descriptors_vars[0]);
    uint32_t i = 0;
    
    for(i=0; i<number; i++)
    {
        eoprot_config_callbacks_variable_set(&mn_callbacks_descriptors_vars[i]);
    }
}

extern "C" {
void boardtransceiver_fun_UPDT_mc_joint_cmmnds_interactionmode(const EOnv* nv, const eOropdescriptor_t* rd);
}

void BoardTransceiver::eoprot_override_mc(void)
{

    static const eOprot_callbacks_endpoint_descriptor_t mc_callbacks_descriptor_endp = 
    { 
        EO_INIT(.endpoint)          eoprot_endpoint_motioncontrol, 
        EO_INIT(.raminitialise)     NULL // or any xxeoprot_fun_INITIALISE_mc 
    };
    
    static const eOprot_callbacks_variable_descriptor_t mc_callbacks_descriptors_vars[] = 
    { 
        // joint
#if 0
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
        {   // joint_cmmnds_setpoint731
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
#endif

        // in here put what the ems needs to do when some variables arrive

        {   // ...............
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_cmmnds_interactionmode,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        boardtransceiver_fun_UPDT_mc_joint_cmmnds_interactionmode
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

}


void BoardTransceiver::eoprot_override_as(void)
{

    // dont configure anything for now
#if 0

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


void BoardTransceiver::eoprot_override_sk(void)
{
    // dont configure anything for now
#if 0
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


bool BoardTransceiver::prepareTransceiverConfig(yarp::os::Searchable &config)
{

    memcpy(&devtxrxcfg, &eo_devicetransceiver_cfg_default, sizeof(eOdevicetransceiver_cfg_t));

    // the nvsetcfg of the board ...
    devtxrxcfg.nvsetdevcfg             = getNVset_DEVcfg(config);

    if(NULL == devtxrxcfg.nvsetdevcfg)
    {
        return(false);
    }

    devtxrxcfg.remotehostipv4addr       = remoteipaddr;
    devtxrxcfg.remotehostipv4port       = remoteipport;    


    const eOtransceiver_sizes_t devsizes =   
    {
        EO_INIT(.capacityoftxpacket)                1408,            
        EO_INIT(.capacityofrop)                      256,                     
        EO_INIT(.capacityofropframeregulars)        1024, 
        EO_INIT(.capacityofropframeoccasionals)      128,
        EO_INIT(.capacityofropframereplies)          256,
        EO_INIT(.maxnumberofregularrops)              32
    };  

    memcpy(&devtxrxcfg.sizes, &devsizes, sizeof(eOtransceiver_sizes_t));




    // other configurable parameters for eOhosttransceiver_cfg_t
    // - mutex_fn_new, transprotection, nvsetprotection are left (NULL, eo_trans_protection_none, eo_nvset_protection_none) 
    //   as in default because we dont protect internally w/ a mutex
    // - proxycfg is left NULL as in default because we dont use a proxy.
    
    // marco.accame on 29 apr 2014: so that the EOreceiver calls this funtion in case of error in sequence number
    devtxrxcfg.extfn.onerrorseqnumber = cpp_protocol_callback_incaseoferror_in_sequencenumberReceived;


    return(true);
}




const eOnvset_DEVcfg_t * BoardTransceiver::getNVset_DEVcfg(yarp::os::Searchable &config)
{

    const eOnvset_DEVcfg_t* nvsetdevcfg = NULL;



    nvsetdevcfg = NULL;

    eOprotconfig_cfg_t protcfg;
    memcpy(&protcfg, &eo_protconfig_cfg_default, sizeof(eOprotconfig_cfg_t));
    // ok, now i get the values from config and run ...

    // at least ... this one
    protcfg.board                           = get_protBRDnumber();
    
    if(config.isNull())
    {
        yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: entire PROTOCOL group ... using max capabilities";
        //return false;
    }
    else
    {
        int      number;

        if(false == config.check("endpointManagementIsSupported"))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire MN endpoint ... enabled w/ max capabilities" <<
                          " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
        }
        else if((false == config.check("entityMNcommunicationNumberOf")) || (false == config.check("entityMNapplicationNumberOf")))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some MN entities ... using max capabilities" <<
                          " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
        } 
        else
        {   
            // ok, retrieve the numbers ...  
            number  = config.find("endpointManagementIsSupported").asInt32();              
            protcfg.ep_management_is_present                = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_management_is_present)
            {
                protcfg.en_mn_entity_comm_numberof          = config.find("entityMNcommunicationNumberOf").asInt32();
                protcfg.en_mn_entity_appl_numberof          = config.find("entityMNapplicationNumberOf").asInt32();  
            }
            else
            {
                protcfg.en_mn_entity_comm_numberof          = 0;
                protcfg.en_mn_entity_appl_numberof          = 0;
            }
            // sanity check
            if((protcfg.en_mn_entity_comm_numberof > 1) || (protcfg.en_mn_entity_appl_numberof > 1))
            {
                yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of MN entities"  <<
                              " (comm, appl) = (" << protcfg.en_mn_entity_comm_numberof << ", " << protcfg.en_mn_entity_appl_numberof << ")";
            }
        
        }


        if(false == config.check("endpointMotionControlIsSupported"))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire MC endpoint ... enabled w/ max capabilities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
        }
        else if((false == config.check("entityMCjointNumberOf")) || (false == config.check("entityMCmotorNumberOf")) || 
                (false == config.check("entityMCmotorNumberOf")))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some MC entities ... using max capabilities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointMotionControlIsSupported").asInt32();              
            protcfg.ep_motioncontrol_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_motioncontrol_is_present)
            {
                protcfg.en_mc_entity_joint_numberof          = config.find("entityMCjointNumberOf").asInt32();
                protcfg.en_mc_entity_motor_numberof          = config.find("entityMCmotorNumberOf").asInt32();  
                protcfg.en_mc_entity_controller_numberof     = config.find("entityMCcontrollerNumberOf").asInt32();
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
                yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of MC entities"  <<
                          " (joint, motor, contr) = (" << protcfg.en_mc_entity_joint_numberof << ", " << protcfg.en_mc_entity_motor_numberof << 
                          ", " << protcfg.en_mc_entity_controller_numberof << ")";
            }
        }


        if(false == config.check("endpointAnalogSensorsIsSupported"))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire AS endpoint ... enabled w/ max capabilities" <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
        }
        else if((false == config.check("entityASstrainNumberOf")) || (false == config.check("entityASmaisNumberOf")) || 
                (false == config.check("entityASextorqueNumberOf")))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some AS entities ... using max capabilities"  <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointAnalogSensorsIsSupported").asInt32();              
            protcfg.ep_analogsensors_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_analogsensors_is_present)
            {
                protcfg.en_as_entity_strain_numberof        = config.find("entityASstrainNumberOf").asInt32();
                protcfg.en_as_entity_mais_numberof          = config.find("entityASmaisNumberOf").asInt32();  
                protcfg.en_as_entity_extorque_numberof      = config.find("entityASextorqueNumberOf").asInt32();
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
                yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of AS entities"  <<
                          " (strain, mais, extorque) = (" << protcfg.en_as_entity_strain_numberof << ", " << protcfg.en_as_entity_mais_numberof << 
                          ", " << protcfg.en_as_entity_extorque_numberof << ")";
            }
        }


        if(false == config.check("endpointSkinIsSupported"))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of entire SK endpoint ... enabled w/ max capabilities"  <<
                          " (skin) = (" << protcfg.en_sk_entity_skin_numberof << ")";
        }
        else if((false == config.check("entitySKskinNumberOf")))
        {
            yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " misses: mandatory config of some SK entities ... using max capabilities"   <<
                          " (skin) = (" << protcfg.en_sk_entity_skin_numberof << ")";
        } 
        else
        {   
            number  = config.find("endpointSkinIsSupported").asInt32();              
            protcfg.ep_skin_is_present             = (0 == number) ? (eobool_false) : (eobool_true);
            if(eobool_true == protcfg.ep_skin_is_present)
            {
                protcfg.en_sk_entity_skin_numberof        = config.find("entitySKskinNumberOf").asInt32();
            }
            else
            {
                protcfg.en_sk_entity_skin_numberof        = 0;
            }
            // sanity check
            if((protcfg.en_sk_entity_skin_numberof > 2))
            {
                yWarning() << "BoardTransceiver-> board " << get_protBRDnumber()+1 << " has: a strange number of SK entities"   <<
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

    
    if(NULL == nvsetdevcfg)
    {
        yError() << "BoardTransceiver::getNVset_DEVcfg() -> FAILS as it produces a NULL result";   
    }
    
    return(nvsetdevcfg);
}

#if 0
void boardtransceiver_fun_UPDT_mn_appl_cmmnds_go2state(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOmn_appl_state_t *newstate_ptr = (eOmn_appl_state_t *)rd->data;

    switch(*newstate_ptr)
    {
        case applstate_running:
        case applstate_config:
        case applstate_error:
        {
            //applstate = *newstate_ptr;
        } break;
    }
}

void boardtransceiver_fun_UPDT_mc_joint_cmmnds_interactionmode(const EOnv* nv, const eOropdescriptor_t* rd)
{
    EOnv_hid aNV = {0};
    eOnvBRD_t brd = eo_nv_GetBRD(nv);

    EOnvSet* mynvset = arrayofnvsets[brd];

    eOprotIndex_t index = eoprot_ID2index(rd->id32);
    eOenum08_t* pmode = (eOenum08_t*) rd->data;

    eOnvID32_t id32status = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, index, eoprot_tag_mc_joint_status);
    eo_nvset_NV_Get(mynvset, eok_ipv4addr_localhost, id32status, &aNV);
    
    eOmc_joint_status_t jointstatus = {0};
    uint16_t size = 0;

    eOresult_t res = eo_nv_Get(&aNV, eo_nv_strg_volatile, &jointstatus, &size);
    jointstatus.interactionmodestatus = *pmode;
    eo_nv_Set(&aNV, &jointstatus, eobool_true, eo_nv_upd_dontdo);

}
#endif

// eof




/*
 * Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


// --------------------------------------------------------------------------------------------------------------------
// - macros
// --------------------------------------------------------------------------------------------------------------------

#undef _ENABLE_TRASMISSION_OF_EMPTY_ROPFRAME_ //if this macro is defined then ethMenager sends pkts to ems even if they are empty
                                              //ATTENTION: is important to define also the same macro in ethManager.cpp



#if defined(USE_EOPROT_OLD) | defined(USE_EOPROT_ROBOT) | defined(USE_EOPROT_XML)

    #warning --> keeping USE_EOPROT_xxx from cmakelist
#else
    #warning --> specifying USE_EOPROT_xxx by hand 
    #define USE_EOPROT_OLD
    //#define USE_EOPROT_ROBOT
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

#include "boardTransceiver.hpp"
#include "FeatureInterface.h"

#include "EOYtheSystem.h"
#include "EoCommon.h"
#include "EOnv.h"
#include "EOnv_hid.h"
#include "EOrop.h"
#include "EoProtocol.h"



#if defined(USE_EOPROT_OLD)

#warning --> using USE_EOPROT_OLD

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

#elif   defined(USE_EOPROT_ROBOT)

#warning --> using USE_EOPROT_ROBOT

#include "eOprot_robot.h"

#elif   defined(USE_EOPROT_XML)

#warning --> using USE_EOPROT_XML

//#include "EOnvsetDEVbuilder.h"
#include "EOprotocolConfigurator.h"

#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "EoProtocolSK.h"

#else
    #error --> chose a USE_EOPROT_xxx amongst: USE_EOPROT_ROBOT, USE_EOPROT_OLD, USE_EOPROT_XML
#endif

#include "Debug.h"

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>

using namespace yarp::os;

BoardTransceiver::BoardTransceiver() : transMutex(1)
{
    yTrace();

    UDP_socket          = NULL;
    ipport              = 0;
    localipaddr         = 0;
    remoteipaddr        = 0;

    protboardnumber     = eo_prot_BRDdummy;
    p_RxPkt             = NULL;
    hosttxrx            = NULL;
    pc104txrx           = NULL;
    nvset               = NULL;
    memcpy(&hosttxrxcfg, &eo_boardtransceiver_cfg_default, sizeof(eOboardtransceiver_cfg_t));       // CHECK ??
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

    int port      = rf.find("port").asInt();              // .get(1).asInt();
    strcpy(_fId.PC104ipAddr.string, parameter1.toString().c_str());
    _fId.PC104ipAddr.port = port;

    Bottle parameter2( rf.find("emsIpAddress").asString() );    // .findGroup("IpAddress");
    strcpy(_fId.EMSipAddr.string, parameter2.toString().c_str());
    _fId.EMSipAddr.port = port;

    sscanf(_fId.EMSipAddr.string,"\"%d.%d.%d.%d", &_fId.EMSipAddr.ip1, &_fId.EMSipAddr.ip2, &_fId.EMSipAddr.ip3, &_fId.EMSipAddr.ip4);
    sscanf(_fId.PC104ipAddr.string,"\"%d.%d.%d.%d", &_fId.PC104ipAddr.ip1, &_fId.PC104ipAddr.ip2, &_fId.PC104ipAddr.ip3, &_fId.PC104ipAddr.ip4);

    sprintf(_fId.EMSipAddr.string,"%u.%u.%u.%u:%u", _fId.EMSipAddr.ip1, _fId.EMSipAddr.ip2, _fId.EMSipAddr.ip3, _fId.EMSipAddr.ip4, _fId.EMSipAddr.port);
    sprintf(_fId.PC104ipAddr.string,"%u.%u.%u.%u:%u", _fId.PC104ipAddr.ip1, _fId.PC104ipAddr.ip2, _fId.PC104ipAddr.ip3, _fId.PC104ipAddr.ip4, _fId.PC104ipAddr.port);

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

bool BoardTransceiver::init(yarp::os::Searchable &config, uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsizerx, FEAT_boardnumber_t _board_n)
{
    // the configuration of the transceiver: it is specific of a given remote board
    yTrace();

    // assign values of some member variables

    protboardnumber = featIdBoardNum2nvBoardNum(_board_n);
    localipaddr     = _localipaddr;
    remoteipaddr    = _remoteipaddr;
    ipport          = _ipport; 



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

    // now hosttxrxcfg is ready, thus ...
    // initialise the transceiver: it creates a EOtransceiver and its EOnvSet
    //hosttxrx     = eo_boardtransceiver_New(&hosttxrxcfg);            // never returns NULL. it calls its error manager  //CHECK!!!!!
    if(hosttxrx == NULL)
        return false;

    // retrieve the transceiver
    pc104txrx    = eo_boardtransceiver_GetTransceiver(hosttxrx);       //CHECK!!!!!
    if(pc104txrx == NULL)
        return false;

    // retrieve the nvset
    nvset = eo_boardtransceiver_GetNVset(hosttxrx);                 //CHECK!!!!!
    if(NULL == nvset)
    {
        return false;
    }


    // build the packet used for reception.
    p_RxPkt = eo_packet_New(_pktsizerx);
    if(p_RxPkt == NULL)
        return false;


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

    std::cout << " BoardTransceiver is running happily!" << std::endl;

    //get pkt from socket: blocking call with timeout
    recv_size = UDP_socket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0, &recvTimeOut);

    if(recv_size > 0)       // I got a message
    {
        onMsgReception(incoming_msg, recv_size);
    }

    getTransmit(&p_sendData, &bytes_to_send);

    ssize_t ret = UDP_socket->send(p_sendData, bytes_to_send, pc104Addr);
    if(ret < 0)
        yError() << "Unable to send a message";

    return true;   // if false the program quits from the forever loop
}






bool BoardTransceiver::nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
{
    if((NULL == nv) || (NULL == dat))
    {
        yError() << "eo BoardTransceiver: called nvSetData() with NULL nv or dat";
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
bool BoardTransceiver::addSetMessage__(eOprotID32_t protid, uint8_t* data, uint32_t signature, bool writelocalrxcache)
{

    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo BoardTransceiver: called addSetMessage__() with invalid protid";
        return false;
    }

    if(NULL == data)
    {
        yError() << "eo BoardTransceiver: called addSetMessage__() with NULL data";
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
    
    // marco.accame: recommend to use eok_ropdesc_basic to have a basic valid descriptor which is modified later
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

bool BoardTransceiver::addSetMessage(eOprotID32_t protid, uint8_t* data)
{
   return(BoardTransceiver::addSetMessage__(protid, data, eo_rop_SIGNATUREdummy, false));
}

bool BoardTransceiver::addSetMessageAndCacheLocally(eOprotID32_t protid, uint8_t* data)
{
   return(BoardTransceiver::addSetMessage__(protid, data, eo_rop_SIGNATUREdummy, true));
}

bool BoardTransceiver::addSetMessageWithSignature(eOprotID32_t protid, uint8_t* data, uint32_t sig)
{
    return(BoardTransceiver::addSetMessage__(protid, data, sig, false));
}


bool BoardTransceiver::addGetMessage(eOprotID32_t protid)
{
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo BoardTransceiver: called addGetMessage() with invalid protid";
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


bool BoardTransceiver::readBufferedValue(eOprotID32_t protid,  uint8_t *data, uint16_t* size)
{      
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo BoardTransceiver: called readBufferedValue() with invalid protid";
        return false;
    }
    
    if((NULL == data) || (NULL == size))
    {
        yError() << "eo BoardTransceiver: called readBufferedValue() with NULL data or size";
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

bool BoardTransceiver::readSentValue(eOprotID32_t protid, uint8_t *data, uint16_t* size)
{
    bool ret = false;
    if(eobool_false == eoprot_id_isvalid(protboardnumber, protid))
    {
        yError() << "eo BoardTransceiver: called readSentValue() with invalid protid";
        return false;
    }

    if((NULL == data) || (NULL == size))
    {
        yError() << "eo BoardTransceiver: called readSentValue() with NULL data or size";
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
void BoardTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
    if(NULL == data)
    {
        yError() << "eo BoardTransceiver::onMsgReception() called with NULL data";
        return;
    } 
    
    yError() << "Received a message!!!";


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


EOnv* BoardTransceiver::getNVhandler(eOprotID32_t protid, EOnv* nv)
{
    eOresult_t    res;
    res = eo_nvset_NV_Get(nvset, remoteipaddr, protid, nv);
    if(eores_OK != res)
    {
        return NULL;
    }

    return(nv);
}


bool BoardTransceiver::getNVvalue(EOnv *nv, uint8_t* data, uint16_t* size)
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
#undef OLDMODE

#ifdef OLDMODE
#include "EOconstvector_hid.h"
#endif

uint16_t BoardTransceiver::getNVnumber(eOnvEP8_t ep)
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

uint32_t BoardTransceiver::translate_NVid2index(eOprotID32_t protid)
{
    return(eoprot_endpoint_id2prognum(protboardnumber, protid));
}

eOprotBRD_t BoardTransceiver::get_protBRDnumber(void)
{
    return(protboardnumber);
}


bool BoardTransceiver::initProtocol(yarp::os::Searchable &config)
{
    static bool alreadyinitted = false;

    if(!alreadyinitted)
    {
#if     defined(USE_EOPROT_OLD)
    
        // nothing to initialise

#elif   defined(USE_EOPROT_ROBOT)

        // initialise the protocol for the robot
        eoprot_robot_Initialise();

#elif   defined(USE_EOPROT_XML)

        static const uint8_t numOfBoardsinRobot =  12; // to be initialise later or w/ a proper number

        // init the protocol to manage all boards of the robot
        if(eores_OK != eoprot_config_board_numberof(numOfBoardsinRobot))
        {
            return(false);
        }
	
	    // configure all the callbacks of all endpoints.
	
        eoprot_override_mn();
        eoprot_override_mc();
        eoprot_override_as();
        eoprot_override_sk();
#endif
        // ok. all is done correctly
	    alreadyinitted = true;
    }

    return(true);
}



void BoardTransceiver::eoprot_override_mn(void)
{  

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
        {   // joint_config
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config   // CHECK!!!????
        },
        {   // joint_config_pidposition
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_pidposition,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_pidposition
        },
        {   // joint_config_impedance
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_impedance,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_impedance
        },
        {   // joint_config_pidtorque
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_pidtorque,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_pidtorque
        },
        {   // joint_status
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status
        },
        {   // joint_status_basic
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_status_basic,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_status_basic
        },
        {   // joint_cmmnds_setpoint
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_cmmnds_setpoint,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_cmmnds_setpoint
        },
        {   // joint_config_limitsofjoint
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_joint,
            EO_INIT(.tag)           eoprot_tag_mc_joint_config_limitsofjoint,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_joint_config_limitsofjoint
        },
        // motor
        {   // motor_config
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_config,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_config
        },        
        {   // motor_config_maxcurrentofmotor
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_config_maxcurrentofmotor,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_config_maxcurrentofmotor
        },
        {   // motor_status_basic
            EO_INIT(.endpoint)      eoprot_endpoint_motioncontrol,
            EO_INIT(.entity)        eoprot_entity_mc_motor,
            EO_INIT(.tag)           eoprot_tag_mc_motor_status_basic,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_mc_motor_status_basic
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
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_calibratedvalues
        },
        {   // strain_status_uncalibratedvalues
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_strain,
            EO_INIT(.tag)           eoprot_tag_as_strain_status_uncalibratedvalues,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_as_strain_status_uncalibratedvalues
        },
        // mais
        {   // mais_status_the15values
            EO_INIT(.endpoint)      eoprot_endpoint_analogsensors,
            EO_INIT(.entity)        eoprot_entity_as_mais,
            EO_INIT(.tag)           eoprot_tag_as_mais_status_the15values,
            EO_INIT(.init)          NULL,
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_as_mais_status_the15values
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
   
}


void BoardTransceiver::eoprot_override_sk(void)
{
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
            EO_INIT(.update)        NULL
//            EO_INIT(.update)        eoprot_fun_UPDT_sk_skin_status_arrayof10canframes
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
    // hosttxrxcfg is a class member ...

    // marco.accame on 10 apr 2014:
    // eOboardtransceiver_cfg_t contains the EOK_HOSTTRANSCEIVER_* values which are good for reception of a suitable EOframe
    // hovever, in future it would be fine to be able loading the fields inside eOhosttransceiver_cfg_t from a common eOprot_robot.h
    memcpy(&hosttxrxcfg, &eo_boardtransceiver_cfg_default, sizeof(eOboardtransceiver_cfg_t));
    hosttxrxcfg.remotehostipv4addr     = remoteipaddr;
    hosttxrxcfg.remotehostipv4port     = ipport;

    // the nvsetcfg of the board ...
    hosttxrxcfg.nvsetdevcfg             = getNVset_DEVcfg(config);



    // other configurable parameters for eOboardtransceiver_cfg_t
    // - mutex_fn_new, transprotection, nvsetprotection are left (NULL, eo_trans_protection_none, eo_nvset_protection_none) 
    //   as in default because we dont protect internally w/ a mutex
    // - confmancfg is left NULL as in default because we dont use a confirmation manager.
    
    // marco.accame on 29 apr 2014: so that the EOreceiver calls this funtion in case of error in sequence number
    hosttxrxcfg.extfn.onerrorseqnumber = cpp_protocol_callback_incaseoferror_in_sequencenumberReceived;


    return(true);
}




const eOnvset_DEVcfg_t * BoardTransceiver::getNVset_DEVcfg(yarp::os::Searchable &config)
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

#elif   defined(USE_EOPROT_ROBOT)

    // initialise the protocol for the robot. if already initted by another board it just returns ok
    eoprot_robot_Initialise();
    uint8_t protboardindex = get_protBRDnumber();
    uint8_t numboards = eoprot_robot_DEVcfg_numberof();
    if(protboardindex >= numboards)
    {   // the robot does not have such a board
        yError() << "BoardTransceiver::getNVset_DEVcfg() called for an invalid board number";
        return false;
    }
    nvsetdevcfg = eoprot_robot_DEVcfg_get(protboardindex);


#elif   defined(USE_EOPROT_XML)

    nvsetdevcfg = NULL;

    eOprotconfig_cfg_t protcfg;
    memcpy(&protcfg, &eo_protconfig_cfg_default, sizeof(eOprotconfig_cfg_t));
    // ok, now i get the values from config and run ...
    
    #warning --> must add code to use the bottle to fill values ....
    if(config.isNull())
    {
        yWarning() << "BoardTransceiver: Can't find PROTOCOL group in config bottle ... using max capabilities";
        //return false;
    }

    nvsetdevcfg = eo_protconfig_DEVcfg_Get(eo_protconfig_New(&protcfg));



#else
    #error --> define a USE_EOPROT_xxx
#endif
    
    return(nvsetdevcfg);
}


// eof




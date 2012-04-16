
/* @file       hostTransceiver.c
    @brief      This file implements internal implementation of a nv object.
    @author     marco.accame@iit.it
    @date       09/03/2010
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

using namespace std;

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include "hostTransceiver.hpp"
#include <yarp/os/impl/Logger.h>

extern "C" {
#include "EoCommon.h"
#include "EOnv_hid.h"

// the endpoints on that particular ems
#include "eOcfg_EPs_rem_board.h"
}

// mutex
#include <pthread.h>

using namespace yarp::os::impl;

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

// originally defined here as a global static -- this prevent instantiation of one of those objects (I need one x ems)
//static hostTransceiver thehosttransceiver;

//static EOhostTransceiver *hosttxrx = NULL;
//static EOtransceiver *pc104txrx = NULL;
//static EOnvsCfg *pc104nvscfg = NULL:


// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


hostTransceiver::hostTransceiver()
{

}

hostTransceiver::~hostTransceiver()
{

}

void hostTransceiver::init(uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsize = EOK_HOSTTRANSCEIVER_capacityofpacket)
{
	YARP_INFO(Logger::get(),"hostTransceiver::init", Logger::get().log_files.f3);

    // the configuration of the transceiver: it is specific of a given remote board
    eOhosttransceiver_cfg_t hosttxrxcfg = 
    {
        EO_INIT(.vectorof_endpoint_cfg) 	eo_cfg_EPs_vectorof_rem_board,
        EO_INIT(.hashfunction_ep2index)		eo_cfg_nvsEP_rem_board_fptr_hashfunction_ep2index,
        EO_INIT(remoteboardipv4addr)    	_remoteipaddr,
        EO_INIT(.remoteboardipv4port)   	_ipport,
        EO_INIT(.tobedefined)           	0
    };
    
    localipaddr  = _localipaddr;
    remoteipaddr = _remoteipaddr;
    ipport       = _ipport;
    
    // initialise the transceiver: it creates a EOtransceiver and its nvsCfg by loading all the endpoints
    hosttxrx     = eo_hosttransceiver_New(&hosttxrxcfg);
    
    // retrieve teh transceiver
    pc104txrx    = eo_hosttransceiver_Transceiver(hosttxrx);
    
    // retrieve the nvscfg
    pc104nvscfg  = eo_hosttransceiver_NVsCfg(hosttxrx);

    pkt = eo_packet_New(_pktsize);
}

void hostTransceiver::load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid)
{
	//pc104txrx
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = opc;
    ropinfo.nvep        = ep;

    ropinfo.nvid = nvid;
    eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);

}

void hostTransceiver::s_eom_hostprotoc_extra_protocoltransceiver_configure_regular_rops_on_board(void)
{
    EOarray *upto10 = (EOarray*) & eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local->upto10rop2signal;
    eOropSIGcfg_t sigcfg;
    char str[128];
    static uint8_t reset = 0;

    eo_array_Reset(upto10);

    if(0 == reset)
    {

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork;
        sigcfg.plustime = 1;
        eo_array_PushBack(upto10, &sigcfg);


        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__bootprocess;
        sigcfg.plustime = 1;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__applicationinfo;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__boardinfo;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipnetmask;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipaddress;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

        sigcfg.ep = EOK_cfg_nvsEP_base_endpoint;
        sigcfg.id = EOK_cfg_nvsEP_base_NVID__remoteipport;
        sigcfg.plustime = 0;
        eo_array_PushBack(upto10, &sigcfg);

    }

    load_occasional_rop(eo_ropcode_set, EOK_cfg_nvsEP_mngmnt_endpoint, EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal);

    if(1 == reset)
    {
        printf( "added a set<__upto10rop2signal, empty-list>");
    }
    else
    {
        printf("added a set<__upto10rop2signal, list>");
    }


    reset = (0==reset) ? (1) : (0);
}

// vecchio //
void hostTransceiver::hostTransceiver_ConfigureRegularsOnRemote(void)
{
    // 1. at first write the nv that we want to set: the EOK_cfg_nvsEP_mngmnt_NVID__upto15epid2signal
    //    there are two alternative ways to do that:
    //    A. QUICK MODE - to write directly onto the local struct of the endpoint. WE USE THIS SECOND WAY because we know
    //       the name of the specific variable to use in the rop
    //    B. FORMAL MODE (and more generic): to use teh nvscfg to retrieve the handle of a EOnv which is relative to the 
    //       pair (endpoint, id) which we wnat to manipulate, and then to use the methods of EOnv to set the value of that variable.
    //       we shall use this mode in function hostTransceiver_AddSetROP()
    
    EOarray *upto15 = (EOarray*) & eo_cfg_nvsEP_mngmnt_usr_rem_board_mem_local->upto10rop2signal;
#warning "cambiato?? per compilare scommentato definizione eOnvEPID_t in EOnv.h"
//    eOnvEPID_t epid;

    // clear the variable.
//    eo_array_Reset(upto15);

    // push back in teh array the first nv that the remote board shall signal
//    epid.ep = EOK_cfg_nvsEP_base_endpoint;
//    epid.id = EOK_cfg_nvsEP_base_NVID_ipnetwork__ipaddress;
//    eo_array_PushBack(upto15, &epid);

    // push back in teh array the second nv that the remote board shall signal
//    epid.ep = EOK_cfg_nvsEP_base_endpoint;
//    epid.id = EOK_cfg_nvsEP_base_NVID__bootprocess;
//    eo_array_PushBack(upto15, &epid);
    
    // 2. then put the rop set<upto15epid2signal, data> inside the transceiver
//    s_hostTransceiver_AddSetROP_with_data_already_set(EOK_cfg_nvsEP_mngmnt_endpoint, EOK_cfg_nvsEP_mngmnt_NVID__upto10rop2signal);
}


// somebody adds a set-rop  plus data.
void hostTransceiver::hostTransceiver_AddSetROP(uint16_t ep, uint16_t id, uint8_t* data, uint16_t size)
{
    uint16_t ss;
    EOnv nv;

    // 1. search for teh EOnv with pair (ep, id)
            
 //   if(eores_OK != eo_nvscfg_GetNV(thehosttransceiver.pc104nvscfg, thehosttransceiver.remoteipaddr, ep, id, &nv))
    {
        // there is no such variable with (remoteipaddr-ep-id) 
 //       return;
    }
    
    // 1bis. verify that the datasize is correct.
    ss = eo_nv_Size(&nv, data);
    if(ss < size)
    {
        // non faccio nulla intanto quando scrivo uso la capacita' della nv.
        // sarebbe bene pero' emettere un warning        
    }
    
    
    // 2. set the data. dont force the set so that we dont write if the nv is readonly.
    
    if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
    {   
        // teh nv is not writeable
        return;
    }
    
    
    // 3. add the rop 
    s_hostTransceiver_AddSetROP_with_data_already_set(ep, id);

}


// somebody passes the received packet - this is used just as an interface
void hostTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
	//this->pkt = data;
	SetReceived(data, size);
}

// and Processes it
void hostTransceiver::SetReceived(uint8_t *data, uint16_t size)
{
    uint16_t numofrops;
    uint64_t txtime;

    eo_packet_Payload_Set(pkt, data, size);
    eo_packet_Addressing_Set(pkt, remoteipaddr, ipport);
    eo_transceiver_Receive(pc104txrx, pkt, &numofrops, &txtime);
}

// somebody retrieves what must be transmitted
void hostTransceiver::getTransmit(uint8_t **data, uint16_t *size)
{
    uint16_t numofrops;
    
    eo_transceiver_Transmit(pc104txrx, &pkt, &numofrops);
    
    eo_packet_Payload_Get(pkt, data, size);
    
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

void hostTransceiver::s_hostTransceiver_AddSetROP_with_data_already_set(uint16_t ep, uint16_t id)
{
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = eo_ropcode_set;
    ropinfo.nvep        = ep;
    ropinfo.nvid        = id;
    
    eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);
}

void hostTransceiver::s_hostTransceiver_AddGetROP(uint16_t ep, uint16_t id)
{
    eo_transceiver_ropinfo_t ropinfo;

    ropinfo.ropcfg      = eok_ropconfig_basic;
    ropinfo.ropcode     = eo_ropcode_ask;
    ropinfo.nvep        = ep;
    ropinfo.nvid        = id;

    eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);
}


void hostTransceiver::askNV(uint16_t endpoint, uint16_t id, uint8_t* data, uint16_t* size)
{
	// add the rop to the ropframe
	load_occasional_rop(eo_ropcode_ask, endpoint, id);

	// wait fot the packet to be sent and for the reply to reach me!!
	EOnv	*nv = getNVhandler( endpoint,  id);

	// eo mutex
	//eOresult_t res = eov_mutex_Take(nv->mtx, eok_reltimeINFINITE);

	pthread_mutex_t mutex;
	pthread_mutex_init(&mutex, NULL);
	nv->mtx = &mutex;
	pthread_mutex_lock(&mutex);
	pthread_mutex_lock(&mutex);


	// now, get the value
	getNVvalue(nv, data, size);
}

EOnv* hostTransceiver::getNVhandler(uint16_t endpoint, uint16_t id)
{
	YARP_INFO(Logger::get(),"hostTransceiver::getNVvalue", Logger::get().log_files.f3);

	uint8_t		ondevindex = 0, onendpointindex = 0 , onidindex = 0;
	EOtreenode	*nvTreenodeRoot;
	EOnv 		*nvRoot;
    eOresult_t res;

    // convetire come parametro, oppure mettere direttam l'indirizzo ip come parametro...
    eOnvOwnership_t	nvownership = eo_nv_ownership_remote;

	res = eo_nvscfg_GetIndices(	this->pc104nvscfg, remoteipaddr, endpoint, id, &ondevindex,	&onendpointindex, &onidindex);

	// if the nvscfg does not have the triple (ip, ep, id) then we return an error
	if(eores_OK != res)
	{
		// Do something about this case
		YARP_ERROR(Logger::get(), " WTF!! NV not found!!!\n",  Logger::get().log_files.f3);
	}
	else
	{
		// 2 passaggi e copia della nv
		// we need a treenode of the nv
		//nvTreenodeRoot = eo_nvscfg_GetTreeNode(this->pc104nvscfg, ondevindex, onendpointindex, onidindex);  //  ?? non usato??
		// but also the handle to the nv.
		//eo_nvscfg_GetNV(this->pc104nvscfg, ondevindex, onendpointindex, onidindex, nvTreenodeRoot, &nvRoot);

		// più diretta e restituisce solo il puntatore che è ciò che mi serve qui, ma non ho accesso a nvsCfg_allnvs
		//nvRoot = eo_matrix3d_At(pc104nvscfg->allnvs, ondevindex, onendpointindex, onidindex);
		nvRoot = eo_nvscfg_GetNVhandler(this->pc104nvscfg, ondevindex, onendpointindex, onidindex, nvTreenodeRoot);

		//getNVvalue(nvRoot, data, size);
	}
	return(nvRoot);
}


void hostTransceiver::getNVvalue(EOnv *nvRoot, uint8_t* data, uint16_t* size)
{
	YARP_INFO(Logger::get(),"hostTransceiver::getNVvalue", Logger::get().log_files.f3);

    if (NULL != nvRoot)
    {
		// get te actual value
		eo_nv_remoteGet(nvRoot, data, size);
	}
}








// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





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

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"

#include "string.h"
#include "hostTransceiver.hpp"
#include <yarp/os/impl/Logger.h>

#include "EoCommon.h"
#include "EOnv_hid.h"

#include "Debug.h"


#define _DEBUG_ON_FILE_
#undef _DEBUG_ON_FILE_

#ifdef _DEBUG_ON_FILE_
#define SOGLIA						70000
#define MAX_ACQUISITION 			10000
uint64_t idx = 0;
uint64_t max_idx = MAX_ACQUISITION*7*16*2 / 8;
uint64_t utime[MAX_ACQUISITION*16*7*2*2] = {0};
uint64_t errors[MAX_ACQUISITION*16*7*2*2][2] = {0};
uint64_t nErr=0;
FILE *outFile = NULL;
#endif



hostTransceiver::hostTransceiver()
{
	yTrace();
}

hostTransceiver::~hostTransceiver()
{
	yTrace();
}

void hostTransceiver::init(uint32_t _localipaddr, uint32_t _remoteipaddr, uint16_t _ipport, uint16_t _pktsize, uint8_t _board_n)
{
	// the configuration of the transceiver: it is specific of a given remote board
	yTrace();
	eOhosttransceiver_cfg_t hosttxrxcfg;
	hosttxrxcfg.remoteboardipv4addr   	= 	_remoteipaddr;
	hosttxrxcfg.remoteboardipv4port 	=  	_ipport;
	hosttxrxcfg.tobedefined 			=   0;


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
	}

	localipaddr  = _localipaddr;
	remoteipaddr = _remoteipaddr;
	ipport       = _ipport;

	// initialise the transceiver: it creates a EOtransceiver and its nvsCfg by loading all the endpoints
	hosttxrx     = eo_hosttransceiver_New(&hosttxrxcfg);

	// retrieve teh transceiver
	pc104txrx    = eo_hosttransceiver_Transceiver(hosttxrx);

	// retrieve the nvscfg
	pc104nvscfg  = eo_hosttransceiver_NVsCfg(hosttxrx);

	pktTx = eo_packet_New(_pktsize);
	pktRx = eo_packet_New(_pktsize);

	// save board specific configs for later use (they may be needed by some callbacks)
	EPvector = hosttxrxcfg.vectorof_endpoint_cfg;
	EPhash_function_ep2index = hosttxrxcfg.hashfunction_ep2index;
}

bool hostTransceiver::nvSetData(const EOnv *nv, const void *dat, eObool_t forceset, eOnvUpdate_t upd)
{
	_mutex.wait();
	bool ret = true;
	if( eores_OK != eo_nv_Set(nv, dat, forceset, upd))
	{
		yError() << "Error while setting NV data\n";
		ret = false;
	}
	_mutex.post();
	return ret;
}

bool hostTransceiver::load_occasional_rop(eOropcode_t opc, uint16_t ep, uint16_t nvid)
{
	bool ret = false;
	eo_transceiver_ropinfo_t ropinfo;

	ropinfo.ropcfg      = eok_ropconfig_basic;
	ropinfo.ropcode     = opc;
	ropinfo.nvep        = ep;
	ropinfo.nvid 		= nvid;

	_mutex.wait();
	eOresult_t res = eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo);
	_mutex.post();

	if(eores_OK == res)
		ret = true;
	else
	{
		yError() << "Error while loading ROP in ropframe\n";
		ret = false;
	}
	return ret;
}

bool hostTransceiver::AddROP(eOropcode_t opc, uint16_t endpoint, uint16_t nvid, uint8_t* data, uint16_t size)
{
	uint16_t		ondevindex = 0, onendpointindex = 0 , onidindex = 0;
	EOtreenode		*nvTreenodeRoot= NULL;
	eOresult_t 		res;
	EOnv			nv;
	uint16_t 		nvSize;

	// convetire come parametro, oppure mettere direttam l'indirizzo ip come parametro...
	eOnvOwnership_t	nvownership = eo_nv_ownership_remote;

	_mutex.wait();
	res = eo_nvscfg_GetIndices(	this->pc104nvscfg, remoteipaddr, endpoint, nvid, &ondevindex,	&onendpointindex, &onidindex);

	// if the nvscfg does not have the triple (ip, ep, id) then we return an error
	if(eores_OK != res)
	{
		// Do something about this case?
		yError() << "Error while loading ROP in ropframe\n";
		_mutex.post();
		return false;
	}

	// nv MUST be a pointer to an already existing object, created by the caller!!
	if(NULL == eo_nvscfg_GetNV(this->pc104nvscfg, ondevindex, onendpointindex, onidindex, nvTreenodeRoot, &nv) )
	{
		// Do something about this case?
		_mutex.post();
		return false;
	}


	// Verify that the datasize is correct.
	nvSize = eo_nv_Size(&nv, data);
	if(nvSize < size)
	{
		// Do something about this case?
		// sarebbe bene pero' emettere un warning
		_mutex.post();
		return false;
	}

	if(eores_OK != eo_nv_Set(&nv, data, eobool_false, eo_nv_upd_dontdo))
	{
		// the nv is not writeable
		_mutex.post();
		return false;
	}


	eo_transceiver_ropinfo_t ropinfo;

	ropinfo.ropcfg      = eok_ropconfig_basic;
	ropinfo.ropcode     = opc;
	ropinfo.nvep        = endpoint;
	ropinfo.nvid 		= nvid;

	if(eores_OK != eo_transceiver_rop_occasional_Load(pc104txrx, &ropinfo))
	{
		yError() << "Error while loading ROP in ropframe\n";
		_mutex.post();
		return false;
	}

	_mutex.post();

	// everything fine!
	return true;
}


// somebody passes the received packet - this is used just as an interface
void hostTransceiver::onMsgReception(uint8_t *data, uint16_t size)
{
	uint16_t numofrops;
	uint64_t txtime;
	_mutex.wait();
	eo_packet_Payload_Set(pktRx, data, size);
	eo_packet_Addressing_Set(pktRx, remoteipaddr, ipport);
	eo_transceiver_Receive(pc104txrx, pktRx, &numofrops, &txtime);
	_mutex.post();

	// Debug
//	checkDataForDebug(data, size);
}

// and Processes it
void checkDataForDebug(uint8_t *data, uint16_t size)
{
	//	yTrace();

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

// somebody retrieves what must be transmitted
void hostTransceiver::getTransmit(uint8_t **data, uint16_t *size)
{
	uint16_t numofrops;
	_mutex.wait();
	eo_transceiver_Transmit(pc104txrx, &pktTx, &numofrops);
	eo_packet_Payload_Get(pktTx, data, size);
	_mutex.post();
}


EOnv* hostTransceiver::getNVhandler(uint16_t endpoint, uint16_t id, EOnv *nvRoot)
{
	//	yTrace();
	_mutex.wait();
	uint16_t		ondevindex = 0, onendpointindex = 0 , onidindex = 0;
	EOtreenode		*nvTreenodeRoot= NULL;
	eOresult_t 		res;

	// convetire come parametro, oppure mettere direttam l'indirizzo ip come parametro...
	eOnvOwnership_t	nvownership = eo_nv_ownership_remote;

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
	_mutex.post();
	return(nvRoot);
}


void hostTransceiver::getNVvalue(EOnv *nvRoot, uint8_t* data, uint16_t* size)
{
	//	yTrace();
	if (NULL == nvRoot)
	{
		return;
	}
	_mutex.wait();
	eo_nv_remoteGet(nvRoot, data, size);
	_mutex.post();
}


void hostTransceiver::getHostData(const EOconstvector **pEPvector, eOuint16_fp_uint16_t *pEPhash_function)
{
	*pEPvector = EPvector;
	*pEPhash_function = EPhash_function_ep2index;
}

uint16_t hostTransceiver::getNVnumber(int boardNum, eOnvEP_t ep)
{
	yTrace();
	const EOconstvector* EPvector = eo_cfg_nvsEP_board_EPs_nvscfgep_get(boardNum);
	uint16_t epindex = eo_cfg_nvsEP_board_EPs_epindex_get(boardNum, ep);
	return eo_cfg_nvsEP_board_NVs_endpoint_numberof_get(EPvector, epindex);
}

uint16_t hostTransceiver::translate_NVid2index(uint8_t boardNum, eOnvEP_t ep, eOnvID_t nvid)
{
//	yTrace();
	EOconstvector* pEPvector = eo_cfg_nvsEP_board_EPs_nvscfgep_get(boardNum);
	uint16_t epindex = eo_cfg_nvsEP_board_EPs_epindex_get(boardNum, ep);
	eOnvscfg_EP_t* EPcfg = eo_cfg_nvsEP_board_EPs_cfg_get(pEPvector,  epindex);
	return eo_cfg_nvsEP_board_NVs_endpoint_Nvindex_get(EPcfg, nvid);
}


/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FEATUREINTERFACE_H_
#define FEATUREINTERFACE_H_



#include <stdint.h>
//#include <stdbool.h>
typedef uint8_t fakestdbool_t;
#define fakestdbool_true    1
#define fakestdbool_false   0

#include <string.h>

extern void eo_receiver_callback_incaseoferror_in_sequencenumberReceived(uint64_t rec_seqnum, uint64_t expected_seqnum);

#ifdef WIN32
//	#pragma warning(disable:4355)
#endif

#define MSG010960 "WARNING-> on april 16 2013 some work is ongoing to clean SIZE_INFO etc."
#if defined(_MSC_VER)
    #pragma message(MSG010960)
#else
    #warning MSG010960
#endif

#include "EOnv_hid.h"
#include "EoMotionControl.h"

#define SIZE_INFO     128

#ifdef __cplusplus

extern "C"
{
#endif

#include "EOconstvector_hid.h"


typedef enum
{
    Management    = 0x00,
    AnalogMais    = 0x01,
    AnalogStrain  = 0x02,
    MotionControl = 0x03,
    Skin          = 0x04
} FeatureType;


typedef struct
{
    uint16_t  port;
    int       ip1,ip2,ip3,ip4;
    char      string[64];
}FEAT_ip_addr;

/** \anchor FEAT_ID   */
typedef struct
{
    uint8_t         boardNum;
    eOnvEP_t        ep;
    void            *handle;

//    ACE_INET_Addr   PC104ipAddr;
//    ACE_INET_Addr   EMSipAddr;

    FEAT_ip_addr      PC104ipAddr;
    FEAT_ip_addr      EMSipAddr;
    // eoStuff
    const EOconstvector  *EPvector;
    eOuint16_fp_uint16_t  EPhash_function;

    // Following are additional and optional info for debug, DO NOT COUNT ON THEM as identifiers for searches!!
    // They may be removed very soon!
    FeatureType           type;
    char                  name[SIZE_INFO];
} FEAT_ID;


#ifdef _SETPOINT_TEST_
typedef enum
{
    proccessed_all_rec_pkt              = 0,
    reached_cfgmaxnumofRXpackets        = 1,
    error_in_reception                  = 2,
    rx_phase_finished                   = 3
}exit_rx_phase_contitions_t;
typedef struct
{
    eOabstime_t             time;           //8B
    eOmeas_position_t       setpoint;      //4B
    uint8_t                 numofrecpkt;    //num of pkt(ropframe) received
    uint8_t                 numofprocesspkt; //num of pkt (ropframe) processed
    int8_t                  exit_cond;
    uint8_t                 diff_packets;
}setpoint_test_data_t;

void check_received_debug_data(FEAT_ID *id, int jointNum, setpoint_test_data_t *test_data_ptr);
#endif

void initCallback(void *p);

fakestdbool_t addEncoderTimeStamp(FEAT_ID *id, int jointNum);
fakestdbool_t findAndFill(FEAT_ID *id, void *sk_array);
fakestdbool_t handle_AS_data(FEAT_ID *id, void *as_array);
void * get_MChandler_fromEP(eOnvEP_t ep);
fakestdbool_t MCmutex_post(void * p, uint16_t epindex, uint16_t nvindex);

fakestdbool_t EP_NV_2_index(eOnvEP_t ep, eOnvID_t nvid, uint16_t *epindex, uint16_t *nvindex);

void transceiver_wait(eOnvEP_t ep);
void transceiver_post(eOnvEP_t ep);

#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */


// eof




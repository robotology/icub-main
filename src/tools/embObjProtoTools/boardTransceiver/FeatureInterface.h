/*
 * Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#ifndef FEATUREINTERFACE_H_
#define FEATUREINTERFACE_H_



#include <stdint.h>
typedef uint8_t fakestdbool_t;
#define fakestdbool_true    1
#define fakestdbool_false   0

#include <string.h>

extern void eo_receiver_callback_incaseoferror_in_sequencenumberReceived(uint64_t rec_seqnum, uint64_t expected_seqnum);

#ifdef WIN32
//	#pragma warning(disable:4355)
#endif

#include "EoCommon.h"
#include "EoProtocol.h"

#define FEAT_SIZE_INFO     128

#ifdef __cplusplus
extern "C"
{
#endif



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
} FEAT_ip_addr;

typedef uint8_t FEAT_boardnumber_t;     // boards are numbered in range [1, maxnum]. moreover 0xff is the invalid value.
#define FEAT_boardnumber_dummy      0xff

/** \anchor FEAT_ID   */
typedef struct
{
    FEAT_boardnumber_t  boardNum;
    eOprotEndpoint_t    ep;
    void                *handle;

    FEAT_ip_addr        PC104ipAddr;
    FEAT_ip_addr        EMSipAddr;

    // Following are additional and optional info for debug, DO NOT COUNT ON THEM as identifiers for searches!!
    // They may be removed very soon!
    FeatureType         type;
    char                name[FEAT_SIZE_INFO];
} FEAT_ID;


#ifdef _SETPOINT_TEST_

#include "EoMeasures.h"     // to see: eOmeas_position_t

typedef enum
{
    proccessed_all_rec_pkt              = 0,
    reached_cfgmaxnumofRXpackets        = 1,
    error_in_reception                  = 2,
    rx_phase_finished                   = 3
} exit_rx_phase_contitions_t;

typedef struct
{
    eOabstime_t             time;               // 8B
    eOmeas_position_t       setpoint;           // 4B
    uint8_t                 numofrecpkt;        // num of pkt(ropframe) received
    uint8_t                 numofprocesspkt;    // num of pkt (ropframe) processed
    int8_t                  exit_cond;
    uint8_t                 diff_packets;
} setpoint_test_data_t;

void check_received_debug_data(FEAT_ID *id, int jointNum, setpoint_test_data_t *test_data_ptr);
#endif

void initCallback(void *p);

//fakestdbool_t addEncoderTimeStamp(FEAT_ID *id, int jointNum);
//fakestdbool_t findAndFill(FEAT_ID *id, void *sk_array, eOprotID32_t id32);
//fakestdbool_t handle_AS_data(FEAT_ID *id, void *as_array, eOprotID32_t id32);

//// requires boardnum in range [1, max] as used by cpp objects
//void * get_MChandler_fromEP(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep);

//fakestdbool_t MCmutex_post(void * p, uint32_t prognum);

//// it converts the protocol board number with range [0, max-1] into the range used by cpp object [1, max]
//FEAT_boardnumber_t nvBoardNum2FeatIdBoardNum(eOprotBRD_t nvboardnum);

eOprotBRD_t featIdBoardNum2nvBoardNum(FEAT_boardnumber_t fid_boardnum);

//double feat_yarp_time_now(void);


#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */


// eof





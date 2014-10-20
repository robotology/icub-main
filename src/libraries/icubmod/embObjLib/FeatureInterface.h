/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
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


void initCallback(void *p);

fakestdbool_t addEncoderTimeStamp(FEAT_boardnumber_t boardnum, eOprotID32_t id32);

fakestdbool_t feat_manage_motioncontrol_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void* rxdata);

fakestdbool_t feat_manage_skin_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *arrayofcanframes);

fakestdbool_t feat_manage_analogsensors_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *as_array);

// requires boardnum in range [1, max] as used by cpp objects
void * get_MChandler_fromEP(FEAT_boardnumber_t boardnum, eOprotEndpoint_t ep);

fakestdbool_t MCmutex_post(void * p, uint32_t prognum);

// it converts the protocol board number with range [0, max-1] into the range used by cpp object [1, max]
FEAT_boardnumber_t nvBoardNum2FeatIdBoardNum(eOprotBRD_t nvboardnum);

eOprotBRD_t featIdBoardNum2nvBoardNum(FEAT_boardnumber_t fid_boardnum);

double feat_yarp_time_now(void);

fakestdbool_t feat_signal_network_reply(eOprotBRD_t brd, eOprotID32_t id32, uint32_t signature);

void* ace_mutex_new(void);

// returns 0 on success to take mutex, -3 on failure upon timeout, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(), tout_usec is in microsec (no timeout is 0xffffffff).
int8_t ace_mutex_take(void* m, uint32_t tout_usec);

// returns 0 on success to take mutex, -1 on genric failure of releasing mutex, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(),
int8_t ace_mutex_release(void* m);


#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */


// eof





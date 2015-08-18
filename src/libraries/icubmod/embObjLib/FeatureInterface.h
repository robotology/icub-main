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


#ifdef __cplusplus
extern "C"
{
#endif

void embObjPrintTrace(char *string);
void embObjPrintDebug(char *string);
void embObjPrintInfo(char *string);
void embObjPrintWarning(char *string);
void embObjPrintError(char *string);
void embObjPrintFatal(char *string);


typedef uint8_t FEAT_boardnumber_t;     // boards are numbered in range [1, maxnum]. moreover 0xff is the invalid value.
enum { FEAT_boardnumber_dummy = 0xff};


void feat_Initialise(void *ethman);

void feat_DeInitialise();

fakestdbool_t feat_addEncoderTimeStamp(FEAT_boardnumber_t boardnum, eOprotID32_t id32);

fakestdbool_t feat_manage_motioncontrol_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void* rxdata);

fakestdbool_t feat_manage_skin_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *arrayofcandata);

fakestdbool_t feat_manage_analogsensors_data(FEAT_boardnumber_t boardnum, eOprotID32_t id32, void *data);

// requires boardnum in range [1, max] as used by cpp objects
void * feat_MC_handler_get(FEAT_boardnumber_t boardnum, eOprotID32_t id32);

fakestdbool_t feat_MC_mutex_post(void * mchandler, uint32_t prognum);

// it converts the protocol board number with range [0, max-1] into the range used by cpp object [1, max]
FEAT_boardnumber_t nvBoardNum2FeatIdBoardNum(eOprotBRD_t nvboardnum);

eOprotBRD_t featIdBoardNum2nvBoardNum(FEAT_boardnumber_t fid_boardnum);

double feat_yarp_time_now(void);

fakestdbool_t feat_signal_network_reply(eOprotBRD_t brd, eOprotID32_t id32, uint32_t signature);

fakestdbool_t feat_embObjCANPrintHandler(eOprotBRD_t brd, eOmn_info_basic_t* infobasic);

void* ace_mutex_new(void);

// returns 0 on success to take mutex, -3 on failure upon timeout, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(), tout_usec is in microsec (no timeout is 0xffffffff).
int8_t ace_mutex_take(void* m, uint32_t tout_usec);

// returns 0 on success to take mutex, -1 on genric failure of releasing mutex, -2 on failure upon null pointer. m is pointer obtained w/ ace_mutex_new(),
int8_t ace_mutex_release(void* m);

void ace_mutex_delete(void* m);


#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */


// eof





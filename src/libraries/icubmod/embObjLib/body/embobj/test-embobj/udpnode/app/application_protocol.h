
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _APPLICATION_PROTOCOL_H_
#define _APPLICATION_PROTOCOL_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
// empty-section

// - external dependencies --------------------------------------------------------------------------------------------
#include "EoCommon.h"
#include "osal.h"

// - public #define  --------------------------------------------------------------------------------------------------
#define PACKET_SIZE         512

#define NODE_ID             0
#define NODE_ID_PC104       254
#define NODE_ID_BROADCAST   255

#define MAX_CMD_PARAM_NUM   2

#define CMD_SET_TIME                1
#define CMD_SEND_START              2
#define CMD_SEND_STOP               3
#define CMD_GET_STATISTICS          4
#define CMD_LISTEN_START            5
#define CMD_LISTEN_STOP              6
#define CMD_SEND_and_LISTEN_START   7
#define CMD_SEND_and_LISTEN_STOP    8
#define CMD_REPLAY_START            9
#define CMD_REPLAY_STOP             10
 
// - declaration of public user-defined types -------------------------------------------------------------------------  

typedef __packed struct 
{
    uint32_t secs;          /**< number of seconds */
    uint32_t nano;          /**< number of nano-seconds */
} eOpkDnanotime_t;

typedef enum
{
    PKT_TYPE_CMD = 0,
    PKT_TYPE_DATA = 1,
    PKT_TYPE_STATISTIC = 2,
    PKT_TYPE_ACK =3
}pkt_type_t;



typedef __packed struct
{
    uint8_t node_id_src;
    uint8_t node_id_dest;
    pkt_type_t type; //its size is uint8_t.  if strict ANSI C compiler's option  is disabled enum size is fit to range of value
    uint8_t pad1;   
}pkt_header_t; 


typedef __packed struct
{
    uint32_t cmd;
    uint32_t param[MAX_CMD_PARAM_NUM];
    uint8_t pad [PACKET_SIZE - sizeof(pkt_header_t) - 12];
}command_payload_t; 

typedef __packed struct
{
    __packed struct pc104_info
    {
        uint32_t seq_num;
        eOpkDnanotime_t send_time; //linux time format
    }pc104_info;
    __packed struct ems_info
    {
        uint32_t seq_num;
        eOpkDnanotime_t send_time;
        eOpkDnanotime_t receive_time;
        osal_lifetime_t idle_time;
    }ems_info;
    uint8_t pad [PACKET_SIZE - sizeof(pkt_header_t) - 40];
}data_payload_t; 



typedef __packed struct
{
    uint32_t received_pkt;
    uint32_t sent_pkt;
    uint32_t lost_pkt;
    uint8_t pad [PACKET_SIZE - sizeof(pkt_header_t) - 12];

}statistics_payload_t;

typedef struct
{
     pkt_header_t hdr;
     union
     {
       statistics_payload_t stat_pload;
       data_payload_t       data_pload;
       command_payload_t    cmd_pload;
     }data;
}packet_t; 



typedef struct
{
    uint32_t rec_pkt;
    uint32_t lost_pkt;
    uint32_t sent_pkt;
}statistic_datastruct_t;

// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



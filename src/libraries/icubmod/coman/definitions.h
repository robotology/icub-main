/*
	defienitions.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>

#define MAX_DSP_BOARDS  30
#define MAX_MC_BOARDS  25
#define MAX_FT_BOARDS  2

#define BCAST_MC_DATA_PACKETS 0xBB // BCAST_DATA_PACKETS MotorController
#define BCAST_FT_DATA_PACKETS 0xBC // BCAST_DATA_PACKETS ForceTorqueSens
#define BCAST_MA_DATA_PACKETS 0xBD // BCAST_DATA_PACKETS MultiAxis ?!?

#define _1_SEC          1000
// 10 mins at 1Khz loop
//#define LOG_SIZE    _1_SEC * 60 * 10
// 60 secs at 1Khz loop
#define LOG_SIZE    _1_SEC * 60


/**
 * @defgroup DataStructures Data Structures
 * @ingroup RoboLLI
 * @brief Here are the data structures with brief descriptions: 
 *  
 * @{ 
 */ 

/**
 * log tx references
 */
typedef struct {
    unsigned long long  ts;
    int                 pos[MAX_DSP_BOARDS];
} log_ctrl_t;


/**
 * McBoard motor controller broadcast data 
 *  
 * WARNING struct is generated with c_gen.py script 
 * - policy value       0x28FF  0b10100011111111 
 * - extra policy value 0x0004  0b100 
 *  
 */
typedef struct {
	unsigned char _header;
	unsigned char _n_bytes;
	unsigned char _command;
	unsigned char _board_id;
	int	 Position;
	short	 Velocity;
	short	 Torque;
	short	 PID_out;
	int	 PID_err;
	int	 Current;
	int	 Temp_DC;
	int	 Timestamp;
	short	 Abs_pos;
	short	 Motor_state;
	int	 Req_Target_Pos;
	unsigned char _chk;
	void fprint(FILE *fp) {
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,Torque,PID_out,PID_err,Current,Temp_DC,Timestamp,Abs_pos,Motor_state,Req_Target_Pos);
	}
    void sprint(char *buff) {
        sprintf(buff, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,Torque,PID_out,PID_err,Current,Temp_DC,Timestamp,Abs_pos,Motor_state,Req_Target_Pos);
    }
    void sprint_(char *buff) {
        sprintf(buff, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", Position,Velocity,Torque,PID_out,PID_err,Current,Temp_DC,Timestamp,Abs_pos,Motor_state,Req_Target_Pos);
    }
} __attribute__((__packed__)) mc_bc_data_t;


/**
 * FtBoard force torque sensor broadcast data
 *  
 * WARNING struct is generated with c_gen.py script
 * - policy value       0x0002  0b01
 * - extra policy value 0x0000  0b0 
 *  
 */ 
typedef struct {
	unsigned char _header;
	unsigned char _n_bytes;
	unsigned char _command;
	unsigned char _board_id;
	int	 fx;
	int	 fy;
	int	 fz;
	int	 tx;
	int	 ty;
	int	 tz;
	unsigned char _chk;
	void fprint(FILE *fp) {
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", fx,fy,fz,tx,ty,tz);
	}
    void sprint(char *buff) {
        sprintf(buff, "%d\t%d\t%d\t%d\t%d\t%d\n", fx,fy,fz,tx,ty,tz);
    }
} __attribute__((__packed__)) ft_bc_data_t;

/**
 * common udp header tx by DSP boards
 */
typedef struct {
    unsigned char _header;
	unsigned char _n_bytes;
	unsigned char _command;
	unsigned char _board_id;
} bc_header_t;

/**
 * convinient way to access broadcast data
 */
typedef	union {
    bc_header_t     bc_header;
    mc_bc_data_t    mc_bc_data;
    ft_bc_data_t    ft_bc_data;
} bc_data_t;

/**
 * logging DSP boards broadcast data
 */
typedef struct {
    uint64_t        ts;
    bc_data_t    bc_data;
} log_t;

/**
 * ...
 */
typedef struct {
    int     multiplier;
    short   offset;
} __attribute__((__packed__)) torque_factor_t;

/** @}
 */


#endif

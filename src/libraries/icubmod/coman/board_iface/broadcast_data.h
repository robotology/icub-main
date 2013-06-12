/*
    broadcast.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2013 , alessio.margan@iit.it)

    Generated:
        2013-03-18 16:17:20
*/

#ifndef __BROADCAST_DATA_H__
#define __BROADCAST_DATA_H__

#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include <map>
#include <string>
typedef std::map<std::string, int> bc_data_map_t;

/*  
 * WARNING struct is generated with c_gen.py script
 * - policy value 0x89BB 0b1000100110111011
 * - extra policy value 0x10C1 0b1000011000001
 *  
 */ 
typedef struct {
	unsigned char _header;
	unsigned char _n_bytes;
	unsigned char _command;
	unsigned char _board_id;
	int	 Position;
	short	 Velocity;
	short	 PID_out;
	int	 PID_err;
	int	 Current;
	int	 Tendon_tor;
	char	 Faults_0;
	char	 Faults_1;
	short	 Height;
	short	 Hip_pos;
	int	 Target_pos;
	int	 Lin_enc_raw;
	int	 Delta_tor;
	short	 Lin_enc_vel;
	unsigned char _chk;
	void check_policy(unsigned short policy, unsigned short extra_policy) {
		assert(0x28FF == policy);
		assert(0x0004 == extra_policy);
	}
	void fprint(FILE *fp) {
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,PID_out,PID_err,Current,Tendon_tor,Faults_0,Faults_1,Height,Hip_pos,Target_pos,Lin_enc_raw,Delta_tor,Lin_enc_vel);
	}
	void sprint(char *buff, size_t size) {
		snprintf(buff, size, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,PID_out,PID_err,Current,Tendon_tor,Faults_0,Faults_1,Height,Hip_pos,Target_pos,Lin_enc_raw,Delta_tor,Lin_enc_vel);
	}
	void to_map(bc_data_map_t &bc_map) {
		bc_map["Position"] = Position;
		bc_map["Velocity"] = Velocity;
		bc_map["PID_out"] = PID_out;
		bc_map["PID_err"] = PID_err;
		bc_map["Current"] = Current;
		bc_map["Tendon_tor"] = Tendon_tor;
		bc_map["Faults_0"] = Faults_0;
		bc_map["Faults_1"] = Faults_1;
		bc_map["Height"] = Height;
		bc_map["Hip_pos"] = Hip_pos;
		bc_map["Target_pos"] = Target_pos;
		bc_map["Lin_enc_raw"] = Lin_enc_raw;
		bc_map["Delta_tor"] = Delta_tor;
		bc_map["Lin_enc_vel"] = Lin_enc_vel;
	}
} __attribute__((__packed__)) mc_bc_data_t;
/*  
 * WARNING struct is generated with c_gen.py script
 * - policy value 0x0002 0b10
 * - extra policy value 0x0000 0b0
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
	void check_policy(unsigned short policy, unsigned short extra_policy) {
		assert(0x0002 == policy);
		assert(0x0000 == extra_policy);
	}
	void fprint(FILE *fp) {
		fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", fx,fy,fz,tx,ty,tz);
	}
	void sprint(char *buff, size_t size) {
		snprintf(buff, size, "%d\t%d\t%d\t%d\t%d\t%d\n", fx,fy,fz,tx,ty,tz);
	}
	void to_map(bc_data_map_t &bc_map) {
		bc_map["fx"] = fx;
		bc_map["fy"] = fy;
		bc_map["fz"] = fz;
		bc_map["tx"] = tx;
		bc_map["ty"] = ty;
		bc_map["tz"] = tz;
	}
} __attribute__((__packed__)) ft_bc_data_t;

#endif

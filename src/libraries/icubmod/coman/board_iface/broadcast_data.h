/*
    broadcast.h

    Copyright (C) 2012 Italian Institute of Technology

    Developer:
        Alessio Margan (2013 , alessio.margan@iit.it)

    Generated:
        2013-11-25 15:59:22
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
 * - policy value 0x893F 0b1000100100111111
 * - extra policy value 0x0001 0b1
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
    unsigned char Faults_0;
    unsigned char Faults_1;
    short	 Link_pos;
    short	 Link_deflection;
    int	 Target_pos;
    unsigned char _chk;
    const unsigned short get_policy(void) { return 0x893F; }
    const unsigned short get_extra_policy(void) { return 0x0001; }
    const unsigned char faults(void) { return Faults_0; }
    void fprint(FILE *fp) {
        fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,Torque,PID_out,PID_err,Current,Faults_0,Faults_1,Link_pos,Link_deflection,Target_pos);
    }
    void sprint(char *buff, size_t size) {
        snprintf(buff, size, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", Position,Velocity,Torque,PID_out,PID_err,Current,Faults_0,Faults_1,Link_pos,Link_deflection,Target_pos);
    }
    void to_map(bc_data_map_t &bc_map) {
        bc_map["Position"] = Position;
        bc_map["Velocity"] = Velocity;
        bc_map["Torque"] = Torque;
        bc_map["PID_out"] = PID_out;
        bc_map["PID_err"] = PID_err;
        bc_map["Current"] = Current;
        bc_map["Faults_0"] = Faults_0;
        bc_map["Faults_1"] = Faults_1;
        bc_map["Link_pos"] = Link_pos;
        bc_map["Link_deflection"] = Link_deflection;
        bc_map["Target_pos"] = Target_pos;
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
    const unsigned short get_policy(void) { return 0x0002; }
    const unsigned short get_extra_policy(void) { return 0x0000; }
    const unsigned short faults(void) { return 0x0; }
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

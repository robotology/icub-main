// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2013  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <byteswap.h>
#include <string>
#include <map>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>


// Enumeration of possible IMU commands
enum cmd {
    CMD_RAW                      =  0xC1,
    CMD_ACCEL_ANGRATE            =  0xC2,
    CMD_DELVEL_DELANG            =  0xC3,
    CMD_CONTINUOUS               =  0xC4,
    CMD_ORIENT                   =  0xC5,
    CMD_ATT_UPDATE               =  0xC6,
    CMD_MAG_VEC                  =  0xC7,
    CMD_ACCEL_ANGRATE_ORIENT     =  0xC8,
    CMD_WRITE_ACCEL_BIAS         =  0xC9,
    CMD_WRITE_GYRO_BIAS          =  0xCA,
    CMD_ACCEL_ANGRATE_MAG        =  0xCB,
    CMD_ACCEL_ANGRATE_MAG_ORIENT =  0xCC,
    CMD_CAPTURE_GYRO_BIAS        =  0xCD,
    CMD_EULER                    =  0xCE,
    CMD_EULER_ANGRATE            =  0xCF,
    CMD_TEMPERATURES             =  0xD1,
    CMD_GYROSTAB_ANGRATE_MAG     =  0xD2,
    CMD_DELVEL_DELANG_MAG        =  0xD3,
    CMD_SAMPLING_SETTING         =  0xDB,
    CMD_QUATERNION               =  0xDF,
    CMD_DEV_ID_STR               =  0xEA,
    CMD_STOP_CONTINUOUS          =  0xFA
};

typedef union {
    struct {
        float   x;
        float   y;
        float   z;
    };
    uint32_t    _bytes[3];
    void swap_vect(void) { for (int i=0; i<3; i++) _bytes[i] = bswap_32(_bytes[i]);}
} _3f_vect_t;

typedef union {
    struct {
        float   q0;
        float   q1;
        float   q2;
        float   q3;
    };
    uint32_t    _bytes[4];
    void swap_vect(void) { for (int i=0; i<4; i++) _bytes[i] = bswap_32(_bytes[i]);}
} _4f_vect_t;

typedef union {
    struct {
        float   m11;
        float   m12;
        float   m13;
        float   m21;
        float   m22;
        float   m23;
        float   m31;
        float   m32;
        float   m33;
    };
    uint32_t    _bytes[9];
    void swap_vect(void) { for (int i=0; i<9; i++) _bytes[i] = bswap_32(_bytes[i]);}
} _3f_matx_t;

////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////

// 0xC2 : Acceleration & Angular Rate
// BigEndian
struct _C2_ {
    uint8_t     cmd;
    _3f_vect_t  acc;
    _3f_vect_t  angRate;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _C2_ acc_angRate_t;

// 0xC5 : Orientation Matrix
// BigEndian
struct _C5_ {
    uint8_t     cmd;
    _3f_matx_t  orientMat;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _C5_ orientMat_t;

// 0xC8 : Acceleration, Angular Rate & Orientation Matrix
// BigEndian
struct _C8_ {
    uint8_t     cmd;
    _3f_vect_t  acc;
    _3f_vect_t  angRate;
    _3f_matx_t  orientMat;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _C8_ acc_ang_orient_t;

// 0xCB : Acceleration, Angular Rate & Magnetometer Vectors
// BigEndian
struct _CB_ {
    uint8_t     cmd;
    _3f_vect_t  acc;
    _3f_vect_t  angRate;
    _3f_vect_t  mag;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _CB_ acc_ang_mag_t;

// 0xCC : Acceleration, Angular Rate & Magnetometer Vectors & Orientation Matrix
// BigEndian
struct _CC_ {
    uint8_t     cmd;
    _3f_vect_t  acc;
    _3f_vect_t  angRate;
    _3f_vect_t  mag;
    _3f_matx_t  orientMat;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _CC_ acc_ang_mag_orient_t;

// 0xCE : Euler Angles
// BigEndian
struct _CE_ {
    uint8_t     cmd;
    _3f_vect_t  eul;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _CE_ eul_t;

// 0xCF : Euler Angles & Angular Rate
// BigEndian
struct _CF_ {
    uint8_t     cmd;
    _3f_vect_t  eul;
    _3f_vect_t  angRate;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _CF_ eul_angRate_t;

// 0xDF : Quaternion
// BigEndian
struct _DF_ {
    uint8_t     cmd;
    _4f_vect_t  quat;
    uint32_t    timer;
    uint16_t    checksum;
} __attribute__((__packed__));
typedef _DF_ quat_t;


typedef union {
    acc_angRate_t           aa;
    acc_ang_orient_t        aaom;
    acc_ang_mag_orient_t    aamom;
    acc_ang_mag_t           aam;
    orientMat_t             om;
    eul_angRate_t           ea;
    eul_t                   eu;
    quat_t                  quat;
    uint8_t                 buffer[80];
} data_3DM_GX3_t;


typedef void*(*funptr_t)(data_3DM_GX3_t &);

typedef struct {
    uint8_t cmd;
    uint8_t expSize;
    data_3DM_GX3_t data;
    funptr_t process;
} imu_cmd_t;

#endif /* DATATYPES_H_ */

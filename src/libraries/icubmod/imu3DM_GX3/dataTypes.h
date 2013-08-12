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

//////////////////////////////////////////////////////
//   herlper functions
/////////////////////////////////////////////////////

inline unsigned short calc_checksum(uint8_t *buff, int len)
{
    unsigned short checksum = 0;
    for (int i = 0; i < len; i++) { checksum += buff[i]; }
    return checksum;
}



inline void * process_C2(data_3DM_GX3_t &data)
{
    acc_angRate_t * aa = &data.aa;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(acc_angRate_t)-2);
    aa->acc.swap_vect();
    aa->angRate.swap_vect();
    aa->timer = bswap_32(aa->timer);
    aa->checksum = bswap_16(aa->checksum);
    return (void*)(checksum == aa->checksum);
}

inline void * print_C2(data_3DM_GX3_t &data)
{
    acc_angRate_t * aa = &data.aa;
    printf("Accel - Ang rate [0xC2]\n");
    printf("\t%f %f %f\n\t%f %f %f\n",
            aa->acc.x,
            aa->acc.y,
            aa->acc.z,
            aa->angRate.x,
            aa->angRate.y,
            aa->angRate.z);
    return 0;
}

inline void * process_C8(data_3DM_GX3_t &data)
{
    acc_ang_orient_t * aaom = &data.aaom;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(acc_ang_orient_t)-2);
    aaom->acc.swap_vect();
    aaom->angRate.swap_vect();
    aaom->orientMat.swap_vect();
    aaom->timer = bswap_32(aaom->timer);
    aaom->checksum = bswap_16(aaom->checksum);
    //printf("0x%02X 0x%02X\n", checksum, aaom->checksum);
    return (void*)(checksum == aaom->checksum);
}

inline void * print_C8(data_3DM_GX3_t &data)
{
    acc_ang_orient_t * aaom = &data.aaom;

    printf("Accel - Ang rate - Mag - Orient M [0xCC]\n");
    printf("\t%f %f %f\n",
            aaom->acc.x,
            aaom->acc.y,
            aaom->acc.z);
    printf("\t%f %f %f\n",
            aaom->angRate.x,
            aaom->angRate.y,
            aaom->angRate.z);
    printf("\t[%f %f %f\n\t%f %f %f\n\t%f %f %f]\n",
            aaom->orientMat.m11,
            aaom->orientMat.m12,
            aaom->orientMat.m13,
            aaom->orientMat.m21,
            aaom->orientMat.m22,
            aaom->orientMat.m23,
            aaom->orientMat.m31,
            aaom->orientMat.m32,
            aaom->orientMat.m33);
    return 0;
}

inline void * process_CB(data_3DM_GX3_t &data)
{
    acc_ang_mag_t * aam = &data.aam;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(acc_ang_mag_t)-2);
    aam->acc.swap_vect();
    aam->angRate.swap_vect();
    aam->mag.swap_vect();
    aam->timer = bswap_32(aam->timer);
    aam->checksum = bswap_16(aam->checksum);
    //printf("0x%02X 0x%02X\n", checksum, aamom->checksum);
    return (void*)(checksum == aam->checksum);
}

inline void * print_CB(data_3DM_GX3_t &data)
{
    acc_ang_mag_t * aam = &data.aam;

    printf("Accel - Ang rate - Mag - Orient M [0xCC]\n");
    printf("\t%f %f %f\n",
            aam->acc.x,
            aam->acc.y,
            aam->acc.z);
    printf("\t%f %f %f\n",
            aam->angRate.x,
            aam->angRate.y,
            aam->angRate.z);
    printf("\t%f %f %f\n",
            aam->mag.x,
            aam->mag.y,
            aam->mag.z);
    return 0;
}

inline void * process_CC(data_3DM_GX3_t &data)
{
    acc_ang_mag_orient_t * aamom = &data.aamom;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(acc_ang_mag_orient_t)-2);
    aamom->acc.swap_vect();
    aamom->angRate.swap_vect();
    aamom->mag.swap_vect();
    aamom->orientMat.swap_vect();
    aamom->timer = bswap_32(aamom->timer);
    aamom->checksum = bswap_16(aamom->checksum);
    //printf("0x%02X 0x%02X\n", checksum, aamom->checksum);
    return (void*)(checksum == aamom->checksum);
}

inline void * print_CC(data_3DM_GX3_t &data)
{
    acc_ang_mag_orient_t * aamom = &data.aamom;

    printf("Accel - Ang rate - Mag - Orient M [0xCC]\n");
    printf("\t%f %f %f\n",
            aamom->acc.x,
            aamom->acc.y,
            aamom->acc.z);
    printf("\t%f %f %f\n",
            aamom->angRate.x,
            aamom->angRate.y,
            aamom->angRate.z);
    printf("\t%f %f %f\n",
            aamom->mag.x,
            aamom->mag.y,
            aamom->mag.z);
    printf("\t[%f %f %f\n\t%f %f %f\n\t%f %f %f]\n",
            aamom->orientMat.m11,
            aamom->orientMat.m12,
            aamom->orientMat.m13,
            aamom->orientMat.m21,
            aamom->orientMat.m22,
            aamom->orientMat.m23,
            aamom->orientMat.m31,
            aamom->orientMat.m32,
            aamom->orientMat.m33);
    return 0;
}

inline void * process_CE(data_3DM_GX3_t &data)
{
    eul_t * eu = &data.eu;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(eul_t)-2);
    eu->eul.swap_vect();
    eu->timer = bswap_32(eu->timer);
    eu->checksum = bswap_16(eu->checksum);
    return (void*)(checksum == eu->checksum);
}

inline void * print_CE(data_3DM_GX3_t &data)
{
    eul_t * eu = &data.eu;
    printf("Euler ang [0xCE]\n");
    printf("\t%f %f %f\n",
            eu->eul.x,
            eu->eul.y,
            eu->eul.z);
    return 0;
}

inline void * process_CF(data_3DM_GX3_t &data)
{
    eul_angRate_t * ea = &data.ea;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(eul_angRate_t)-2);
    ea->eul.swap_vect();
    ea->angRate.swap_vect();
    ea->timer = bswap_32(ea->timer);
    ea->checksum = bswap_16(ea->checksum);
    return (void*)(checksum == ea->checksum);
}

inline void * print_CF(data_3DM_GX3_t &data)
{
    eul_angRate_t * ea = &data.ea;
    printf("Euler ang [0xCE]\n");
    printf("\t%f %f %f\n",
            ea->eul.x,
            ea->eul.y,
            ea->eul.z);
    printf("\t%f %f %f\n",
            ea->angRate.x,
            ea->angRate.y,
            ea->angRate.z);
    return 0;
}

inline void * process_DF(data_3DM_GX3_t &data)
{
    quat_t * quat = &data.quat;
    unsigned short checksum = calc_checksum(data.buffer, sizeof(quat_t)-2);
    quat->quat.swap_vect();
    quat->timer = bswap_32(quat->timer);
    quat->checksum = bswap_16(quat->checksum);
    return (void*)(checksum == quat->checksum);
}

inline void * print_DF(data_3DM_GX3_t &data)
{
    quat_t * quat = &data.quat;
    printf("Quaternion [0xDF]\n");
    printf("\t%f %f %f %f\n",
            quat->quat.q0,
            quat->quat.q1,
            quat->quat.q2,
            quat->quat.q3);
    return 0;
}


#endif /* DATATYPES_H_ */

/* Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
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

#ifndef _ST_M1_DATATYPES_
#define _ST_M1_DATATYPES_

#include <byteswap.h>
#include <string>
#include <map>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#define counter_base    3
#define accel_base      5
#define gyro_base       11
#define magneto_base    17
#define temperat_base   23
#define quaternion_base 37

typedef struct  {
    //     uint16_t    counter;
    int16_t     accel[3];
    int16_t     gyro[3];
    int16_t     magn[3];
    int16_t     temp;
    uint32_t    euler_raw[3];
    uint32_t    quat[4];
} __attribute__ ((__packed__)) Pippo;




typedef struct {
    uint8_t frameControl;
    uint8_t length;
    uint8_t messageId;
    uint8_t *payload;
} __attribute__((__packed__))
ST_IMU_Frame;

// Mask for Frame Control byte fields
#define FRAME_TYPE_MASK     OXC0
#define ACK_MASK            OX20
#define LF_MF_MASK          OX10
#define FRAME_VERSION_MASK  OX0C
#define QOS_MASK            OX03

typedef enum{
    control = 0,
    data,
    ack,
    nack
} FrameTypes;


// list of available Communication control commands
#define iNEMO_Connect           0x00
#define iNEMO_Disconnect        0x01
#define iNEMO_Reset_Board       0x02
#define iNEMO_Enter_DFU_Mode    0x00
#define iNEMO_Trace             0x07
#define iNEMO_Led_Control       0x08


// QoS possible values
#define NORMAL_PRIORITY 0X00
#define MEDIUM_PRIORITY 0X01
#define HIGH_PRIORITY   0X02
#define FRU             0X03


// list of available Board information commands
// TBD

// list of available Sensor setting commands

// Sensor_Type list

// etc etc etc...



#if 0
// 0xC5 : Orientation Matrix
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
#endif

#endif /* _ST_M1_DATATYPES_ */

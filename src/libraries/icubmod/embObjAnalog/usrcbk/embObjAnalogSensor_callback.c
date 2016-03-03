/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
 * website: www.robotcub.org
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

/* @file       embObjAnaloSensor_callback.c
    @brief     This file keeps examples for ems / pc104 of the user-defined functions used for all endpoints in as
    @author     marco.accame@iit.it
    @date       05/02/2012
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "EoCommon.h"
#include "EoProtocol.h"
#include "EOnv.h"
#include "EOarray.h"

#include "FeatureInterface.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EoProtocolAS.h"



// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void handle_data(const EOnv* nv, const eOropdescriptor_t* rd);

static void handle_data_inertial(const EOnv* nv, const eOropdescriptor_t* rd);

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty section



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eoprot_fun_ONSAY_as(const EOnv* nv, const eOropdescriptor_t* rd)
{
    // marco.accame on 18 mar 2014: this function is called when a say<id32, data> rop is received
    // and the id32 is about the analog sensors endpoint. this function is common to every board.
    // it is used this function and not another one because inside the hostTransceiver object it was called:
    // eoprot_config_onsay_endpoint_set(eoprot_endpoint_analogsensors, eoprot_fun_ONSAY_as);

    // the aim of this function is to wake up a thread which is blocked because it has sent an ask<id32>
    // the wake up funtionality is implemented in one mode only:
    // a. in initialisation, embObjAnalogSensor sets some values and then reads them back.
    //    the read back sends an ask<id32, signature=0xaa000000>. in such a case the board sends back
    //    a say<id32, data, signature = 0xaa000000>. thus, if the received signature is 0xaa000000, then
    //    we must unblock using feat_signal_network_reply().

    if(0xaa000000 == rd->signature)
    {   // case a:
        if(eobool_false == feat_signal_network_reply(eo_nv_GetIP(nv), rd->id32, rd->signature))
        {
            char str[256] = {0};
            char nvinfo[128];
            eoprot_ID2information(rd->id32, nvinfo, sizeof(nvinfo));
            snprintf(str, sizeof(str), "eoprot_fun_ONSAY_as() received an unexpected message w/ 0xaa000000 signature for %s", nvinfo);
            embObjPrintWarning(str);
            return;
        }
    }
}


extern void eoprot_fun_UPDT_as_strain_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
}


extern void eoprot_fun_UPDT_as_strain_status_fullscale(const EOnv* nv, const eOropdescriptor_t* rd)
{
#if 0
    // marco.accame: this code has been used for debug, to verify the reception of the sig<fullscale> sent by the ems
    // now it is of no use. 
    EOarray *array = (EOarray*)rd->data;
    uint8_t size = eo_array_Size(array);


    printf("eoprot_fun_UPDT_as_strain_status_fullscale() has received array of size %d and values: [",
           size
           );

    int i=0;
    for(i=0; i<6; i++)
    {
        uint16_t *a0 = (uint16_t *) eo_array_At(array, i);
        uint16_t a00 = (NULL == a0) ? 0xffff : (*a0);
        printf(" %d,", a00);
    }
    printf(" ]\n");

#endif
}


extern void eoprot_fun_UPDT_as_strain_status_calibratedvalues(const EOnv* nv, const eOropdescriptor_t* rd)
{
    handle_data(nv, rd);
}

extern void eoprot_fun_UPDT_as_strain_status_uncalibratedvalues(const EOnv* nv, const eOropdescriptor_t* rd)
{
    handle_data(nv, rd);
}


extern void eoprot_fun_UPDT_as_mais_status_the15values(const EOnv* nv, const eOropdescriptor_t* rd)
{
    handle_data(nv, rd);
}

extern void eoprot_fun_UPDT_as_mais_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_as_mais_config_datarate(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_as_mais_config_mode(const EOnv* nv, const eOropdescriptor_t* rd)
{
}



// - inertial

extern void eoprot_fun_UPDT_as_inertial_config(const EOnv* nv, const eOropdescriptor_t* rd)
{
}

extern void eoprot_fun_UPDT_as_inertial_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    handle_data_inertial(nv, rd);
}

//extern void eoprot_fun_UPDT_as_inertial_status_accelerometer(const EOnv* nv, const eOropdescriptor_t* rd)
//{
//    handle_data_inertial_acc(nv, rd);
//}

//extern void eoprot_fun_UPDT_as_inertial_status_gyroscope(const EOnv* nv, const eOropdescriptor_t* rd)
//{
//    handle_data_inertial_gyr(nv, rd);
//}




// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void handle_data(const EOnv* nv, const eOropdescriptor_t* rd)
{
    EOarray* arrayof = (EOarray*)rd->data;
    uint8_t sizeofarray = eo_array_Size(arrayof);
    if(0 != sizeofarray)
    {
        feat_manage_analogsensors_data(eo_nv_GetIP(nv), rd->id32, (void *)arrayof);
    }
}

static void handle_data_inertial(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOas_inertial_status_t *inertialstatus  = (eOas_inertial_status_t*)rd->data;
    feat_manage_analogsensors_data(eo_nv_GetIP(nv), rd->id32, (void *)inertialstatus);
}



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




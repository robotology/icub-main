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


/*  @file       EOProtocolAS_fun_userdef.c
    @brief      This file keeps callbacks used for AS protocol in icub-main
    @author     marco.accame@iit.it
    @date       22 mar 2016
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

static void handle_data_analogarray(const EOnv* nv, const eOropdescriptor_t* rd);

static void handle_data_inertial(const EOnv* nv, const eOropdescriptor_t* rd);

static void handle_data_inertial3(const EOnv* nv, const eOropdescriptor_t* rd);

static void handle_data_temperature(const EOnv* nv, const eOropdescriptor_t* rd);

static void handle_data_psc(const EOnv* nv, const eOropdescriptor_t* rd);
// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------
// empty section



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------


extern void eoprot_fun_ONSAY_as(const EOnv* nv, const eOropdescriptor_t* rd)
{
    feat_signal_network_onsay(eo_nv_GetIP(nv), rd->id32, rd->signature);
}


extern void eoprot_fun_UPDT_as_strain_status_calibratedvalues(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_analogarray(nv, rd);
    }
}


extern void eoprot_fun_UPDT_as_strain_status_uncalibratedvalues(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_analogarray(nv, rd);
    }
}


extern void eoprot_fun_UPDT_as_mais_status_the15values(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_analogarray(nv, rd);
    }
}


extern void eoprot_fun_UPDT_as_inertial_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_inertial(nv, rd);
    }
}

extern void eoprot_fun_UPDT_as_inertial3_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_inertial3(nv, rd);
    }
}


extern void eoprot_fun_UPDT_as_temperature_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_temperature(nv, rd);
    }
}

extern void eoprot_fun_UPDT_as_psc_status(const EOnv* nv, const eOropdescriptor_t* rd)
{
    if(eo_ropcode_sig == rd->ropcode)
    {
        handle_data_psc(nv, rd);
    }
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void handle_data_analogarray(const EOnv* nv, const eOropdescriptor_t* rd)
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


static void handle_data_inertial3(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOas_inertial3_status_t *inertial3status  = (eOas_inertial3_status_t*)rd->data;
    feat_manage_analogsensors_data(eo_nv_GetIP(nv), rd->id32, (void *)inertial3status);
}


static void handle_data_temperature(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOas_temperature_status_t *tempstatus  = (eOas_temperature_status_t*)rd->data;
    feat_manage_analogsensors_data(eo_nv_GetIP(nv), rd->id32, (void *)tempstatus);
}

static void handle_data_psc(const EOnv* nv, const eOropdescriptor_t* rd)
{
    eOas_psc_status_t *pscstatus  = (eOas_psc_status_t*)rd->data;
    feat_manage_analogsensors_data(eo_nv_GetIP(nv), rd->id32, (void *)pscstatus);
}


// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------




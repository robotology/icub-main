/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        usbdsc.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************/

/*********************************************************************
 * Descriptor specific type definitions are defined in:
 * system\usb\usbdefs\usbdefs_std_dsc.h
 ********************************************************************/

#ifndef USBDSC_H
#define USBDSC_H

/** I N C L U D E S *************************************************/
#include "system\typedefs.h"
#include "autofiles\usbcfg.h"

#if defined(USB_USE_CDC)
#include "system\usb\class\cdc\cdc.h"
#endif

#include "system\usb\usb.h"

/** D E F I N I T I O N S *******************************************/

#define CFG01 rom struct                            \
{   USB_CFG_DSC             cd01;                   \
    USB_INTF_DSC            i01a00;                 \
    USB_CDC_HEADER_FN_DSC   cdc_header_fn_i01a00;   \
    USB_CDC_ACM_FN_DSC      cdc_acm_fn_i01a00;      \
    USB_CDC_UNION_FN_DSC    cdc_union_fn_i01a00;    \
    USB_CDC_CALL_MGT_FN_DSC cdc_call_mgt_fn_i01a00; \
    USB_EP_DSC              ep02i_i01a00;           \
    USB_INTF_DSC            i02a00;                 \
    USB_EP_DSC              ep03o_i02a00;           \
    USB_EP_DSC              ep03i_i02a00;           \
} cfg01

/** E X T E R N S ***************************************************/
extern rom USB_DEV_DSC device_dsc;
extern CFG01;
extern rom const unsigned char *rom USB_CD_Ptr[];
extern rom const unsigned char *rom USB_SD_Ptr[];

extern rom pFunc ClassReqHandler[1];

#endif //USBDSC_H

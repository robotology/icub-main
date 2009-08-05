/*********************************************************************
 *
 *             Microchip USB C18 Firmware -  CDC Version 1.0
 *
 *********************************************************************
 * FileName:        cdc.h
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
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       7/21/04     Original.
 ********************************************************************/
#ifndef CDC_H
#define CDC_H

/** I N C L U D E S **********************************************************/
#include "system\typedefs.h"

/** D E F I N I T I O N S ****************************************************/

/* Class-Specific Requests */
#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

/* Notifications *
 * Note: Notifications are polled over
 * Communication Interface (Interrupt Endpoint)
 */
#define NETWORK_CONNECTION          0x00
#define RESPONSE_AVAILABLE          0x01
#define SERIAL_STATE                0x20


/* Device Class Code */
#define CDC_DEVICE                  0x02

/* Communication Interface Class Code */
#define COMM_INTF                   0x02

/* Communication Interface Class SubClass Codes */
#define ABSTRACT_CONTROL_MODEL      0x02

/* Communication Interface Class Control Protocol Codes */
#define V25TER                      0x01    // Common AT commands ("Hayes(TM)")


/* Data Interface Class Codes */
#define DATA_INTF                   0x0A

/* Data Interface Class Protocol Codes */
#define NO_PROTOCOL                 0x00    // No class specific protocol required


/* Communication Feature Selector Codes */
#define ABSTRACT_STATE              0x01
#define COUNTRY_SETTING             0x02

/* Functional Descriptors */
/* Type Values for the bDscType Field */
#define CS_INTERFACE                0x24
#define CS_ENDPOINT                 0x25

/* bDscSubType in Functional Descriptors */
#define DSC_FN_HEADER               0x00
#define DSC_FN_CALL_MGT             0x01
#define DSC_FN_ACM                  0x02    // ACM - Abstract Control Management
#define DSC_FN_DLM                  0x03    // DLM - Direct Line Managment
#define DSC_FN_TELEPHONE_RINGER     0x04
#define DSC_FN_RPT_CAPABILITIES     0x05
#define DSC_FN_UNION                0x06
#define DSC_FN_COUNTRY_SELECTION    0x07
#define DSC_FN_TEL_OP_MODES         0x08
#define DSC_FN_USB_TERMINAL         0x09
/* more.... see Table 25 in USB CDC Specification 1.1 */

/* CDC Bulk IN transfer states */
#define CDC_TX_READY                0
#define CDC_TX_BUSY                 1
#define CDC_TX_BUSY_ZLP             2       // ZLP: Zero Length Packet
#define CDC_TX_COMPLETING           3

/******************************************************************************
 * Macro:           BOOL mUSBUSARTIsTxTrfReady(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This macro is used to check if the CDC class is ready
 *                  to send more data.
 *                  Typical Usage: if(mUSBUSARTIsTxTrfReady())
 *
 * Note:            None
 *****************************************************************************/
#define mUSBUSARTIsTxTrfReady()     (cdc_trf_state == CDC_TX_READY)

/******************************************************************************
 * Macro:           (bit) mCDCUsartRxIsBusy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This macro is used to check if CDC bulk OUT endpoint is
 *                  busy (owned by SIE) or not.
 *                  Typical Usage: if(mCDCUsartRxIsBusy())
 *
 * Note:            None
 *****************************************************************************/
#define mCDCUsartRxIsBusy()         CDC_BULK_BD_OUT.Stat.UOWN

/******************************************************************************
 * Macro:           (bit) mCDCUsartTxIsBusy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This macro is used to check if CDC bulk IN endpoint is
 *                  busy (owned by SIE) or not.
 *                  Typical Usage: if(mCDCUsartTxIsBusy())
 *
 * Note:            None
 *****************************************************************************/
#define mCDCUsartTxIsBusy()         CDC_BULK_BD_IN.Stat.UOWN

/******************************************************************************
 * Macro:           byte mCDCGetRxLength(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          mCDCGetRxLength returns cdc_rx_len
 *
 * Side Effects:    None
 *
 * Overview:        mCDCGetRxLength is used to retrieve the number of bytes
 *                  copied to user's buffer by the most recent call to
 *                  getsUSBUSART function.
 *
 * Note:            None
 *****************************************************************************/
#define mCDCGetRxLength()           cdc_rx_len

/******************************************************************************
 * Macro:           void mUSBUSARTTxRam(byte *pData, byte len)
 *
 * PreCondition:    cdc_trf_state must be in the CDC_TX_READY state.
 *                  
 *                  Value of 'len' must be equal to or smaller than 255 bytes.
 *
 * Input:           pDdata  : Pointer to the starting location of data bytes
 *                  len     : Number of bytes to be transferred
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Use this macro to transfer data located in data memory.
 *                  Use this macro when:
 *                  1. Data stream is not null-terminated
 *                  2. Transfer length is known
 *
 *                  Remember: cdc_trf_state must == CDC_TX_READY
 *                  Unlike putsUSBUSART, there is not code double checking
 *                  the transfer state. Unexpected behavior will occur if
 *                  this function is called when cdc_trf_state != CDC_TX_READY
 *
 * Note:            This macro only handles the setup of the transfer. The
 *                  actual transfer is handled by CDCTxService().
 *****************************************************************************/
#define mUSBUSARTTxRam(pData,len)   \
{                                   \
    pCDCSrc.bRam = pData;           \
    cdc_tx_len = len;               \
    cdc_mem_type = _RAM;            \
    cdc_trf_state = CDC_TX_BUSY;    \
}

/******************************************************************************
 * Macro:           void mUSBUSARTTxRom(rom byte *pData, byte len)
 *
 * PreCondition:    cdc_trf_state must be in the CDC_TX_READY state.
 *                  
 *                  Value of 'len' must be equal to or smaller than 255 bytes.
 *
 * Input:           pDdata  : Pointer to the starting location of data bytes
 *                  len     : Number of bytes to be transferred
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Use this macro to transfer data located in program memory.
 *                  Use this macro when:
 *                  1. Data stream is not null-terminated
 *                  2. Transfer length is known
 *
 *                  Remember: cdc_trf_state must == CDC_TX_READY
 *                  Unlike putrsUSBUSART, there is not code double checking
 *                  the transfer state. Unexpected behavior will occur if
 *                  this function is called when cdc_trf_state != CDC_TX_READY
 *
 * Note:            This macro only handles the setup of the transfer. The
 *                  actual transfer is handled by CDCTxService().
 *****************************************************************************/
#define mUSBUSARTTxRom(pData,len)   \
{                                   \
    pCDCSrc.bRom = pData;           \
    cdc_tx_len = len;               \
    cdc_mem_type = _ROM;            \
    cdc_trf_state = CDC_TX_BUSY;    \
}

/** S T R U C T U R E S ******************************************************/

/* Line Coding Structure */
#define LINE_CODING_LENGTH          0x07

typedef union _LINE_CODING
{
    struct
    {
        byte _byte[LINE_CODING_LENGTH];
    };
    struct
    {
        DWORD   dwDTERate;          // Complex data structure
        byte    bCharFormat;
        byte    bParityType;
        byte    bDataBits;
    };
} LINE_CODING;

typedef union _CONTROL_SIGNAL_BITMAP
{
    byte _byte;
    struct
    {
        unsigned DTE_PRESENT;       // [0] Not Present  [1] Present
        unsigned CARRIER_CONTROL;   // [0] Deactivate   [1] Activate
    };
} CONTROL_SIGNAL_BITMAP;


/* Functional Descriptor Structure - See CDC Specification 1.1 for details */

/* Header Functional Descriptor */
typedef struct _USB_CDC_HEADER_FN_DSC
{
    byte bFNLength;
    byte bDscType;
    byte bDscSubType;
    word bcdCDC;
} USB_CDC_HEADER_FN_DSC;

/* Abstract Control Management Functional Descriptor */
typedef struct _USB_CDC_ACM_FN_DSC
{
    byte bFNLength;
    byte bDscType;
    byte bDscSubType;
    byte bmCapabilities;
} USB_CDC_ACM_FN_DSC;

/* Union Functional Descriptor */
typedef struct _USB_CDC_UNION_FN_DSC
{
    byte bFNLength;
    byte bDscType;
    byte bDscSubType;
    byte bMasterIntf;
    byte bSaveIntf0;
} USB_CDC_UNION_FN_DSC;

/* Call Management Functional Descriptor */
typedef struct _USB_CDC_CALL_MGT_FN_DSC
{
    byte bFNLength;
    byte bDscType;
    byte bDscSubType;
    byte bmCapabilities;
    byte bDataInterface;
} USB_CDC_CALL_MGT_FN_DSC;

/** E X T E R N S ************************************************************/
extern byte cdc_rx_len;

extern byte cdc_trf_state;
extern POINTER pCDCSrc;
extern byte cdc_tx_len;
extern byte cdc_mem_type;

/** P U B L I C  P R O T O T Y P E S *****************************************/
void USBCheckCDCRequest(void);
void CDCInitEP(void);
byte getsUSBUSART(char *buffer, byte len);
void putrsUSBUSART(const rom char *data);
void putsUSBUSART(char *data);
void CDCTxService(void);

#endif //CDC_H

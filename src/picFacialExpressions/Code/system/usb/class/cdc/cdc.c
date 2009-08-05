/*********************************************************************
 *
 *             Microchip USB C18 Firmware -  CDC Version 1.0
 *
 *********************************************************************
 * FileName:        cdc.c
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
 * Rawin Rojvanit       11/19/04    Original. RS-232 Emulation Subset
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"

#ifdef USB_USE_CDC

/** V A R I A B L E S ********************************************************/
#pragma udata
byte cdc_rx_len;            // total rx length

byte cdc_trf_state;         // States are defined cdc.h
POINTER pCDCSrc;            // Dedicated source pointer
POINTER pCDCDst;            // Dedicated destination pointer
byte cdc_tx_len;            // total tx length
byte cdc_mem_type;          // _ROM, _RAM

LINE_CODING line_coding;    // Buffer to store line coding information
CONTROL_SIGNAL_BITMAP control_signal_bitmap;

/*
 * SEND_ENCAPSULATED_COMMAND and GET_ENCAPSULATED_RESPONSE are required
 * requests according to the CDC specification.
 * However, it is not really being used here, therefore a dummy buffer is
 * used for conformance.
 */
#define dummy_length    0x08
byte dummy_encapsulated_cmd_response[dummy_length];

/** P R I V A T E  P R O T O T Y P E S ***************************************/

/** D E C L A R A T I O N S **************************************************/
#pragma code

/** C L A S S  S P E C I F I C  R E Q ****************************************/
/******************************************************************************
 * Function:        void USBCheckCDCRequest(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine checks the setup data packet to see if it
 *                  knows how to handle it
 *
 * Note:            None
 *****************************************************************************/
void USBCheckCDCRequest(void)
{
    /*
     * If request recipient is not an interface then return
     */
    if(SetupPkt.Recipient != RCPT_INTF) return;

    /*
     * If request type is not class-specific then return
     */
    if(SetupPkt.RequestType != CLASS) return;

    /*
     * Interface ID must match interface numbers associated with
     * CDC class, else return
     */
    if((SetupPkt.bIntfID != CDC_COMM_INTF_ID)&&
       (SetupPkt.bIntfID != CDC_DATA_INTF_ID)) return;
    
    switch(SetupPkt.bRequest)
    {
        case SEND_ENCAPSULATED_COMMAND:
            ctrl_trf_session_owner = MUID_CDC;
            pSrc.bRam = (byte*)&dummy_encapsulated_cmd_response;
            usb_stat.ctrl_trf_mem = _RAM;
            LSB(wCount) = dummy_length;
            break;
        case GET_ENCAPSULATED_RESPONSE:
            ctrl_trf_session_owner = MUID_CDC;
            // Populate dummy_encapsulated_cmd_response first.
            pDst.bRam = (byte*)&dummy_encapsulated_cmd_response;
            break;
        case SET_COMM_FEATURE:                  // Optional
            break;
        case GET_COMM_FEATURE:                  // Optional
            break;
        case CLEAR_COMM_FEATURE:                // Optional
            break;
        case SET_LINE_CODING:
            ctrl_trf_session_owner = MUID_CDC;
            pDst.bRam = (byte*)&line_coding;    // Set destination
            break;
        case GET_LINE_CODING:
            ctrl_trf_session_owner = MUID_CDC;
            pSrc.bRam = (byte*)&line_coding;    // Set source
            usb_stat.ctrl_trf_mem = _RAM;       // Set memory type
            LSB(wCount) = LINE_CODING_LENGTH;   // Set data count
            break;
        case SET_CONTROL_LINE_STATE:
            ctrl_trf_session_owner = MUID_CDC;
            control_signal_bitmap._byte = LSB(SetupPkt.W_Value);
            break;
        case SEND_BREAK:                        // Optional
            break;
        default:
            break;
    }//end switch(SetupPkt.bRequest)

}//end USBCheckCDCRequest

/** U S E R  A P I ***********************************************************/

/******************************************************************************
 * Function:        void CDCInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        CDCInitEP initializes CDC endpoints, buffer descriptors,
 *                  internal state-machine, and variables.
 *                  It should be called after the USB host has sent out a
 *                  SET_CONFIGURATION request.
 *                  See USBStdSetCfgHandler() in usb9.c for examples.
 *
 * Note:            None
 *****************************************************************************/
void CDCInitEP(void)
{
    //Abstract line coding information
    line_coding.dwDTERate._dword = 115200;      // baud rate
    line_coding.bCharFormat = 0x00;             // 1 stop bit
    line_coding.bParityType = 0x00;             // None
    line_coding.bDataBits = 0x08;               // 5,6,7,8, or 16

    cdc_trf_state = CDC_TX_READY;
    cdc_rx_len = 0;
    
    CDC_COMM_UEP = EP_IN|HSHK_EN;               // Enable 1 Comm pipe
    CDC_DATA_UEP = EP_OUT_IN|HSHK_EN;           // Enable 2 data pipes

    /*
     * Do not have to init Cnt of IN pipes here.
     * Reason:  Number of bytes to send to the host
     *          varies from one transaction to
     *          another. Cnt should equal the exact
     *          number of bytes to transmit for
     *          a given IN transaction.
     *          This number of bytes will only
     *          be known right before the data is
     *          sent.
     */
    CDC_INT_BD_IN.ADR = (byte*)&cdc_notice;     // Set buffer address
    CDC_INT_BD_IN.Stat._byte = _UCPU|_DAT1;     // Set status

    CDC_BULK_BD_OUT.Cnt = sizeof(cdc_data_rx);  // Set buffer size
    CDC_BULK_BD_OUT.ADR = (byte*)&cdc_data_rx;  // Set buffer address
    CDC_BULK_BD_OUT.Stat._byte = _USIE|_DAT0|_DTSEN;// Set status

    CDC_BULK_BD_IN.ADR = (byte*)&cdc_data_tx;   // Set buffer size
    CDC_BULK_BD_IN.Stat._byte = _UCPU|_DAT1;    // Set buffer address

}//end CDCInitEP

/******************************************************************************
 * Function:        byte getsUSBUSART(char *buffer,
 *                                    byte len)
 *
 * PreCondition:    Value of input argument 'len' should be smaller than the
 *                  maximum endpoint size responsible for receiving bulk
 *                  data from USB host for CDC class.
 *                  Input argument 'buffer' should point to a buffer area that
 *                  is bigger or equal to the size specified by 'len'.
 *
 * Input:           buffer  : Pointer to where received bytes are to be stored
 *                  len     : The number of bytes expected.
 *
 * Output:          The number of bytes copied to buffer.
 *
 * Side Effects:    Publicly accessible variable cdc_rx_len is updated with
 *                  the number of bytes copied to buffer.
 *                  Once getsUSBUSART is called, subsequent retrieval of
 *                  cdc_rx_len can be done by calling macro mCDCGetRxLength().
 *
 * Overview:        getsUSBUSART copies a string of bytes received through
 *                  USB CDC Bulk OUT endpoint to a user's specified location. 
 *                  It is a non-blocking function. It does not wait
 *                  for data if there is no data available. Instead it returns
 *                  '0' to notify the caller that there is no data available.
 *
 * Note:            If the actual number of bytes received is larger than the
 *                  number of bytes expected (len), only the expected number
 *                  of bytes specified will be copied to buffer.
 *                  If the actual number of bytes received is smaller than the
 *                  number of bytes expected (len), only the actual number
 *                  of bytes received will be copied to buffer.
 *****************************************************************************/
byte getsUSBUSART(char *buffer, byte len)
{
    cdc_rx_len = 0;
    
    if(!mCDCUsartRxIsBusy())
    {
        /*
         * Adjust the expected number of bytes to equal
         * the actual number of bytes received.
         */
        if(len > CDC_BULK_BD_OUT.Cnt)
            len = CDC_BULK_BD_OUT.Cnt;
        
        /*
         * Copy data from dual-ram buffer to user's buffer
         */
        for(cdc_rx_len = 0; cdc_rx_len < len; cdc_rx_len++)
            buffer[cdc_rx_len] = cdc_data_rx[cdc_rx_len];

        /*
         * Prepare dual-ram buffer for next OUT transaction
         */
        CDC_BULK_BD_OUT.Cnt = sizeof(cdc_data_rx);
        mUSBBufferReady(CDC_BULK_BD_OUT);
    }//end if
    
    return cdc_rx_len;
    
}//end getsUSBUSART

/******************************************************************************
 * Function:        void putsUSBUSART(char *data)
 *
 * PreCondition:    cdc_trf_state must be in the CDC_TX_READY state.
 *                  
 *                  The string of characters pointed to by 'data' must equal
 *                  to or smaller than 255 bytes.
 *
 * Input:           data    : Pointer to a null-terminated string of data.
 *                            If a null character is not found, 255 bytes
 *                            of data will be transferred to the host.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        putsUSBUSART writes a string of data to the USB including
 *                  the null character. Use this version, 'puts', to transfer
 *                  data located in data memory.
 *
 * Note:            The transfer mechanism for device-to-host(put) is more
 *                  flexible than host-to-device(get). It can handle
 *                  a string of data larger than the maximum size of bulk IN
 *                  endpoint. A state machine is used to transfer a long
 *                  string of data over multiple USB transactions.
 *                  See CDCTxService() for more details.
 *****************************************************************************/
void putsUSBUSART(char *data)
{
    byte len;

    /*
     * User should have checked that cdc_trf_state is in CDC_TX_READY state
     * before calling this function.
     * As a safety precaution, this fuction checks the state one more time
     * to make sure it does not override any pending transactions.
     *
     * Currently it just quits the routine without reporting any errors back
     * to the user.
     *
     * Bottomline: User MUST make sure that mUSBUSARTIsTxTrfReady()==1
     *             before calling this function!
     * Example:
     * if(mUSBUSARTIsTxTrfReady())
     *     putsUSBUSART(pData);
     *
     * IMPORTANT: Never use the following blocking while loop to wait:
     * while(!mUSBUSARTIsTxTrfReady())
     *     putsUSBUSART(pData);
     *
     * The whole firmware framework is written based on cooperative
     * multi-tasking and a blocking code is not acceptable.
     * Use a state machine instead.
     */
    if(cdc_trf_state != CDC_TX_READY) return;
    
    /*
     * While loop counts the number of bytes to send including the
     * null character.
     */
    len = 0;
    do
    {
        len++;
        if(len == 255) break;       // Break loop once max len is reached.
    }while(*data++);
    
    /*
     * Re-adjust pointer to its initial location
     */
    data-=len;
    
    /*
     * Second piece of information (length of data to send) is ready.
     * Call mUSBUSARTTxRam to setup the transfer.
     * The actual transfer process will be handled by CDCTxService(),
     * which should be called once per Main Program loop.
     */
    mUSBUSARTTxRam((byte*)data,len);     // See cdc.h
}//end putsUSBUSART

/******************************************************************************
 * Function:        void putrsUSBUSART(const rom char *data)
 *
 * PreCondition:    cdc_trf_state must be in the CDC_TX_READY state.
 *                  
 *                  The string of characters pointed to by 'data' must equal
 *                  to or smaller than 255 bytes.
 *
 * Input:           data    : Pointer to a null-terminated string of data.
 *                            If a null character is not found, 255 bytes
 *                            of data will be transferred to the host.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        putrsUSBUSART writes a string of data to the USB including
 *                  the null character. Use this version, 'putrs', to transfer
 *                  data literals and data located in program memory.
 *
 * Note:            The transfer mechanism for device-to-host(put) is more
 *                  flexible than host-to-device(get). It can handle
 *                  a string of data larger than the maximum size of bulk IN
 *                  endpoint. A state machine is used to transfer a long
 *                  string of data over multiple USB transactions.
 *                  See CDCTxService() for more details.
 *****************************************************************************/
void putrsUSBUSART(const rom char *data)
{
    byte len;

    /*
     * User should have checked that cdc_trf_state is in CDC_TX_READY state
     * before calling this function.
     * As a safety precaution, this fuction checks the state one more time
     * to make sure it does not override any pending transactions.
     *
     * Currently it just quits the routine without reporting any errors back
     * to the user.
     *
     * Bottomline: User MUST make sure that mUSBUSARTIsTxTrfReady()
     *             before calling this function!
     * Example:
     * if(mUSBUSARTIsTxTrfReady())
     *     putsUSBUSART(pData);
     *
     * IMPORTANT: Never use the following blocking while loop to wait:
     * while(cdc_trf_state != CDC_TX_READY)
     *     putsUSBUSART(pData);
     *
     * The whole firmware framework is written based on cooperative
     * multi-tasking and a blocking code is not acceptable.
     * Use a state machine instead.
     */
    if(cdc_trf_state != CDC_TX_READY) return;
    
    /*
     * While loop counts the number of bytes to send including the
     * null character.
     */
    len = 0;
    do
    {
        len++;
        if(len == 255) break;       // Break loop once max len is reached.
    }while(*data++);
    
    /*
     * Re-adjust pointer to its initial location
     */
    data-=len;
    
    /*
     * Second piece of information (length of data to send) is ready.
     * Call mUSBUSARTTxRom to setup the transfer.
     * The actual transfer process will be handled by CDCTxService(),
     * which should be called once per Main Program loop.
     */
    mUSBUSARTTxRom((rom byte*)data,len); // See cdc.h

}//end putrsUSBUSART

/******************************************************************************
 * Function:        void CDCTxService(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        CDCTxService handles device-to-host transaction(s).
 *                  This function should be called once per Main Program loop.
 *
 * Note:            None
 *****************************************************************************/
void CDCTxService(void)
{
    byte byte_to_send;
    
    if(mCDCUsartTxIsBusy()) return;
    /*
     * Completing stage is necessary while [ mCDCUSartTxIsBusy()==1 ].
     * By having this stage, user can always check cdc_trf_state,
     * and not having to call mCDCUsartTxIsBusy() directly.
     */
    if(cdc_trf_state == CDC_TX_COMPLETING)
        cdc_trf_state = CDC_TX_READY;
    
    /*
     * If CDC_TX_READY state, nothing to do, just return.
     */
    if(cdc_trf_state == CDC_TX_READY) return;
    
    /*
     * If CDC_TX_BUSY_ZLP state, send zero length packet
     */
    if(cdc_trf_state == CDC_TX_BUSY_ZLP)
    {
        CDC_BULK_BD_IN.Cnt = 0;
        cdc_trf_state = CDC_TX_COMPLETING;
    }
    else if(cdc_trf_state == CDC_TX_BUSY)
    {
        /*
         * First, have to figure out how many byte of data to send.
         */
    	if(cdc_tx_len > sizeof(cdc_data_tx))
    	    byte_to_send = sizeof(cdc_data_tx);
    	else
    	    byte_to_send = cdc_tx_len;

        /*
         * Next, load the number of bytes to send to Cnt in buffer descriptor
         */
        CDC_BULK_BD_IN.Cnt = byte_to_send;

        /*
         * Subtract the number of bytes just about to be sent from the total.
         */
    	cdc_tx_len = cdc_tx_len - byte_to_send;
    	        
        pCDCDst.bRam = (byte*)&cdc_data_tx; // Set destination pointer
        
        if(cdc_mem_type == _ROM)            // Determine type of memory source
        {
            while(byte_to_send)
            {
                *pCDCDst.bRam = *pCDCSrc.bRom;
                pCDCDst.bRam++;
                pCDCSrc.bRom++;
                byte_to_send--;
            }//end while(byte_to_send)
        }
        else // _RAM
        {
            while(byte_to_send)
            {
                *pCDCDst.bRam = *pCDCSrc.bRam;
                pCDCDst.bRam++;
                pCDCSrc.bRam++;
                byte_to_send--;
            }//end while(byte_to_send._word)
        }//end if(cdc_mem_type...)
        
        /*
         * Lastly, determine if a zero length packet state is necessary.
         * See explanation in USB Specification 2.0: Section 5.8.3
         */
        if(cdc_tx_len == 0)
        {
            if(CDC_BULK_BD_IN.Cnt == sizeof(cdc_data_tx))
                cdc_trf_state = CDC_TX_BUSY_ZLP;
            else
                cdc_trf_state = CDC_TX_COMPLETING;
        }//end if(cdc_tx_len...)
            
    }//end if(cdc_tx_sate == CDC_TX_BUSY)
    
    /*
     * Both CDC_TX_BUSY and CDC_TX_BUSY_ZLP states use the following macro
     */
    mUSBBufferReady(CDC_BULK_BD_IN);

}//end CDCTxService

#endif //USB_USE_CDC

/** EOF cdc.c ****************************************************************/

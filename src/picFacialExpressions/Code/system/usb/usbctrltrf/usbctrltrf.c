/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        usbctrltrf.c
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
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"

/** V A R I A B L E S ********************************************************/
#pragma udata
byte ctrl_trf_state;                // Control Transfer State
byte ctrl_trf_session_owner;        // Current transfer session owner

POINTER pSrc;                       // Data source pointer
POINTER pDst;                       // Data destination pointer
WORD wCount;                        // Data counter

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void USBCtrlTrfSetupHandler(void);
void USBCtrlTrfOutHandler(void);
void USBCtrlTrfInHandler(void);

/** D E C L A R A T I O N S **************************************************/
#pragma code
/******************************************************************************
 * Function:        void USBCtrlEPService(void)
 *
 * PreCondition:    USTAT is loaded with a valid endpoint address.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        USBCtrlEPService checks for three transaction types that
 *                  it knows how to service and services them:
 *                  1. EP0 SETUP
 *                  2. EP0 OUT
 *                  3. EP0 IN
 *                  It ignores all other types (i.e. EP1, EP2, etc.)
 *
 * Note:            None
 *****************************************************************************/
void USBCtrlEPService(void)
{   
    if(USTAT == EP00_OUT)
    {
        if(ep0Bo.Stat.PID == SETUP_TOKEN)           // EP0 SETUP
            USBCtrlTrfSetupHandler();
        else                                        // EP0 OUT
            USBCtrlTrfOutHandler();
    }
    else if(USTAT == EP00_IN)                       // EP0 IN
        USBCtrlTrfInHandler();
    
}//end USBCtrlEPService

/******************************************************************************
 * Function:        void USBCtrlTrfSetupHandler(void)
 *
 * PreCondition:    SetupPkt buffer is loaded with valid USB Setup Data
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine is a task dispatcher and has 3 stages.
 *                  1. It initializes the control transfer state machine.
 *                  2. It calls on each of the module that may know how to
 *                     service the Setup Request from the host.
 *                     Module Example: USB9, HID, CDC, MSD, ...
 *                     As new classes are added, ClassReqHandler table in
 *                     usbdsc.c should be updated to call all available
 *                     class handlers.
 *                  3. Once each of the modules has had a chance to check if
 *                     it is responsible for servicing the request, stage 3
 *                     then checks direction of the transfer to determine how
 *                     to prepare EP0 for the control transfer.
 *                     Refer to USBCtrlEPServiceComplete() for more details.
 *
 * Note:            Microchip USB Firmware has three different states for
 *                  the control transfer state machine:
 *                  1. WAIT_SETUP
 *                  2. CTRL_TRF_TX
 *                  3. CTRL_TRF_RX
 *                  Refer to firmware manual to find out how one state
 *                  is transitioned to another.
 *
 *                  A Control Transfer is composed of many USB transactions.
 *                  When transferring data over multiple transactions,
 *                  it is important to keep track of data source, data
 *                  destination, and data count. These three parameters are
 *                  stored in pSrc,pDst, and wCount. A flag is used to
 *                  note if the data source is from ROM or RAM.
 *
 *****************************************************************************/
void USBCtrlTrfSetupHandler(void)
{
    byte i;
    
    /* Stage 1 */
    ctrl_trf_state = WAIT_SETUP;
    ctrl_trf_session_owner = MUID_NULL;     // Set owner to NULL
    wCount._word = 0;
    
    /* Stage 2 */
    USBCheckStdRequest();                   // See system\usb9\usb9.c
    
    for(i=0;i < (sizeof(ClassReqHandler)/sizeof(pFunc));i++)
    {
        if(ctrl_trf_session_owner != MUID_NULL)break;
        ClassReqHandler[i]();               // See autofiles\usbdsc.c
    }//end while
        
    /* Stage 3 */
    USBCtrlEPServiceComplete();
    
}//end USBCtrlTrfSetupHandler

/******************************************************************************
 * Function:        void USBCtrlTrfOutHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine handles an OUT transaction according to
 *                  which control transfer state is currently active.
 *
 * Note:            Note that if the the control transfer was from
 *                  host to device, the session owner should be notified
 *                  at the end of each OUT transaction to service the
 *                  received data.
 *
 *****************************************************************************/
void USBCtrlTrfOutHandler(void)
{
    if(ctrl_trf_state == CTRL_TRF_RX)
    {
        USBCtrlTrfRxService();
        
        /*
         * Don't have to worry about overwriting _KEEP bit
         * because if _KEEP was set, TRNIF would not have been
         * generated in the first place.
         */
        if(ep0Bo.Stat.DTS == 0)
            ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
        else
            ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;
    }
    else    // CTRL_TRF_TX
        USBPrepareForNextSetupTrf();
    
}//end USBCtrlTrfOutHandler

/******************************************************************************
 * Function:        void USBCtrlTrfInHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine handles an IN transaction according to
 *                  which control transfer state is currently active.
 *
 *
 * Note:            A Set Address Request must not change the acutal address
 *                  of the device until the completion of the control
 *                  transfer. The end of the control transfer for Set Address
 *                  Request is an IN transaction. Therefore it is necessary
 *                  to service this unique situation when the condition is
 *                  right. Macro mUSBCheckAdrPendingState is defined in
 *                  usb9.h and its function is to specifically service this
 *                  event.
 *****************************************************************************/
void USBCtrlTrfInHandler(void)
{
    mUSBCheckAdrPendingState();         // Must check if in ADR_PENDING_STATE
    
    if(ctrl_trf_state == CTRL_TRF_TX)
    {
        USBCtrlTrfTxService();
        
        if(ep0Bi.Stat.DTS == 0)
            ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;
        else
            ep0Bi.Stat._byte = _USIE|_DAT0|_DTSEN;
    }
    else // CTRL_TRF_RX
        USBPrepareForNextSetupTrf();

}//end USBCtrlTrfInHandler

/******************************************************************************
 * Function:        void USBCtrlTrfTxService(void)
 *
 * PreCondition:    pSrc, wCount, and usb_stat.ctrl_trf_mem are setup properly.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should be called from only two places.
 *                  One from USBCtrlEPServiceComplete() and one from
 *                  USBCtrlTrfInHandler(). It takes care of managing a
 *                  transfer over multiple USB transactions.
 *
 * Note:            This routine works with isochronous endpoint larger than
 *                  256 bytes and is shown here as an example of how to deal
 *                  with BC9 and BC8. In reality, a control endpoint can never
 *                  be larger than 64 bytes.
 *****************************************************************************/
void USBCtrlTrfTxService(void)
{    
    WORD byte_to_send;
    
    /*
     * First, have to figure out how many byte of data to send.
     */
    if(wCount._word < EP0_BUFF_SIZE)
        byte_to_send._word = wCount._word;
    else
        byte_to_send._word = EP0_BUFF_SIZE;
    
    /*
     * Next, load the number of bytes to send to BC9..0 in buffer descriptor
     */
    ep0Bi.Stat.BC9 = 0;
    ep0Bi.Stat.BC8 = 0;
    ep0Bi.Stat._byte |= MSB(byte_to_send);
    ep0Bi.Cnt = LSB(byte_to_send);
    
    /*
     * Subtract the number of bytes just about to be sent from the total.
     */
    wCount._word = wCount._word - byte_to_send._word;
    
    pDst.bRam = (byte*)&CtrlTrfData;        // Set destination pointer

    if(usb_stat.ctrl_trf_mem == _ROM)       // Determine type of memory source
    {
        while(byte_to_send._word)
        {
            *pDst.bRam = *pSrc.bRom;
            pDst.bRam++;
            pSrc.bRom++;
            byte_to_send._word--;
        }//end while(byte_to_send._word)
    }
    else // RAM
    {
        while(byte_to_send._word)
        {
            *pDst.bRam = *pSrc.bRam;
            pDst.bRam++;
            pSrc.bRam++;
            byte_to_send._word--;
        }//end while(byte_to_send._word)
    }//end if(usb_stat.ctrl_trf_mem == _ROM)
    
}//end USBCtrlTrfTxService

/******************************************************************************
 * Function:        void USBCtrlTrfRxService(void)
 *
 * PreCondition:    pDst and wCount are setup properly.
 *                  pSrc is always &CtrlTrfData
 *                  usb_stat.ctrl_trf_mem is always _RAM.
 *                  wCount should be set to 0 at the start of each control
 *                  transfer.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        *** This routine is only partially complete. Check for
 *                  new version of the firmware.
 *
 * Note:            None
 *****************************************************************************/
void USBCtrlTrfRxService(void)
{
    WORD byte_to_read;

    MSB(byte_to_read) = 0x03 & ep0Bo.Stat._byte;    // Filter out last 2 bits
    LSB(byte_to_read) = ep0Bo.Cnt;
    
    /*
     * Accumulate total number of bytes read
     */
    wCount._word = wCount._word + byte_to_read._word;
    
    pSrc.bRam = (byte*)&CtrlTrfData;

    while(byte_to_read._word)
    {
        *pDst.bRam = *pSrc.bRam;
        pDst.bRam++;
        pSrc.bRam++;
        byte_to_read._word--;
    }//end while(byte_to_read._word)    
    
}//end USBCtrlTrfRxService

/******************************************************************************
 * Function:        void USBCtrlEPServiceComplete(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine wrap up the ramaining tasks in servicing
 *                  a Setup Request. Its main task is to set the endpoint
 *                  controls appropriately for a given situation. See code
 *                  below.
 *                  There are three main scenarios:
 *                  a) There was no handler for the Request, in this case
 *                     a STALL should be sent out.
 *                  b) The host has requested a read control transfer,
 *                     endpoints are required to be setup in a specific way.
 *                  c) The host has requested a write control transfer, or
 *                     a control data stage is not required, endpoints are
 *                     required to be setup in a specific way.
 *
 *                  Packet processing is resumed by clearing PKTDIS bit.
 *
 * Note:            None
 *****************************************************************************/
void USBCtrlEPServiceComplete(void)
{
    if(ctrl_trf_session_owner == MUID_NULL)
    {
        /*
         * If no one knows how to service this request then stall.
         * Must also prepare EP0 to receive the next SETUP transaction.
         */
        ep0Bo.Cnt = EP0_BUFF_SIZE;
        ep0Bo.ADR = (byte*)&SetupPkt;
        
        ep0Bo.Stat._byte = _USIE|_BSTALL;
        ep0Bi.Stat._byte = _USIE|_BSTALL;
    }
    else    // A module has claimed ownership of the control transfer session.
    {
        if(SetupPkt.DataDir == DEV_TO_HOST)
        {
            if(SetupPkt.wLength < wCount._word)
                wCount._word = SetupPkt.wLength;
            USBCtrlTrfTxService();
            ctrl_trf_state = CTRL_TRF_TX;
            /*
             * Control Read:
             * <SETUP[0]><IN[1]><IN[0]>...<OUT[1]> | <SETUP[0]>
             * 1. Prepare OUT EP to respond to early termination
             *
             * NOTE:
             * If something went wrong during the control transfer,
             * the last status stage may not be sent by the host.
             * When this happens, two different things could happen
             * depending on the host.
             * a) The host could send out a RESET.
             * b) The host could send out a new SETUP transaction
             *    without sending a RESET first.
             * To properly handle case (b), the OUT EP must be setup
             * to receive either a zero length OUT transaction, or a
             * new SETUP transaction.
             *
             * Since the SETUP transaction requires the DTS bit to be
             * DAT0 while the zero length OUT status requires the DTS
             * bit to be DAT1, the DTS bit check by the hardware should
             * be disabled. This way the SIE could accept either of
             * the two transactions.
             *
             * Furthermore, the Cnt byte should be set to prepare for
             * the SETUP data (8-byte or more), and the buffer address
             * should be pointed to SetupPkt.
             */
            ep0Bo.Cnt = EP0_BUFF_SIZE;
            ep0Bo.ADR = (byte*)&SetupPkt;            
            ep0Bo.Stat._byte = _USIE;           // Note: DTSEN is 0!
    
            /*
             * 2. Prepare IN EP to transfer data, Cnt should have
             *    been initialized by responsible request owner.
             */
            ep0Bi.ADR = (byte*)&CtrlTrfData;
            ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;
        }
        else    // (SetupPkt.DataDir == HOST_TO_DEV)
        {
            ctrl_trf_state = CTRL_TRF_RX;
            /*
             * Control Write:
             * <SETUP[0]><OUT[1]><OUT[0]>...<IN[1]> | <SETUP[0]>
             *
             * 1. Prepare IN EP to respond to early termination
             *
             *    This is the same as a Zero Length Packet Response
             *    for control transfer without a data stage
             */
            ep0Bi.Cnt = 0;
            ep0Bi.Stat._byte = _USIE|_DAT1|_DTSEN;

            /*
             * 2. Prepare OUT EP to receive data.
             */
            ep0Bo.Cnt = EP0_BUFF_SIZE;
            ep0Bo.ADR = (byte*)&CtrlTrfData;
            ep0Bo.Stat._byte = _USIE|_DAT1|_DTSEN;
        }//end if(SetupPkt.DataDir == DEV_TO_HOST)
    }//end if(ctrl_trf_session_owner == MUID_NULL)
    
    /*
     * PKTDIS bit is set when a Setup Transaction is received.
     * Clear to resume packet processing.
     */
    UCONbits.PKTDIS = 0;

}//end USBCtrlEPServiceComplete

/******************************************************************************
 * Function:        void USBPrepareForNextSetupTrf(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The routine forces EP0 OUT to be ready for a new Setup
 *                  transaction, and forces EP0 IN to be owned by CPU.
 *
 * Note:            None
 *****************************************************************************/
void USBPrepareForNextSetupTrf(void)
{
    ctrl_trf_state = WAIT_SETUP;            // See usbctrltrf.h
    ep0Bo.Cnt = EP0_BUFF_SIZE;              // Defined in usbcfg.h
    ep0Bo.ADR = (byte*)&SetupPkt;
    ep0Bo.Stat._byte = _USIE|_DAT0|_DTSEN;  // EP0 buff dsc init, see usbmmap.h
    ep0Bi.Stat._byte = _UCPU;               // EP0 IN buffer initialization
}//end USBPrepareForNextSetupTrf

/** EOF usbctrltrf.c *********************************************************/

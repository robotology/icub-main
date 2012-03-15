// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _APPL_COMMON_H_
#define _APPL_COMMON_H_

// - doxy begin -------------------------------------------------------------------------------------------------------
// empty-section

// - external dependencies --------------------------------------------------------------------------------------------
// empty-section

// - public #define  --------------------------------------------------------------------------------------------------
#define _DEBUG_PRINT_          //definendo questo macro abilito di stampare tutti i messaggi di errman.
#define _WITHOUT_SHALIB_       //definendo questa macro non leggo da ROM i dati (sia qulli delle shared lib che quelli dell'appl)


//************************ macro for shared data beetwen eth and can (1 and 2) *****************************
#define ETHCAN_SHDATA_ITEMSIZE          64
#define ETHCAN_SHDATA_CAPACITY          2

#define ETHCAN1_SHDATA_ITEMSIZE         ETHCAN_SHDATA_ITEMSIZE
#define ETHCAN1_SHDATA_CAPACITY         ETHCAN_SHDATA_CAPACITY

#define ETHCAN2_SHDATA_ITEMSIZE         ETHCAN_SHDATA_ITEMSIZE
#define ETHCAN2_SHDATA_CAPACITY         ETHCAN_SHDATA_CAPACITY


//*********************** common evt definition ************************************************************

#define         EVT_TASK_START             (1 << 1)  //TODO: il valore e' da rivedere
#define         EVT_TASK_STOP  		       (1 << 0)  //TODO: il valore e' da rivedere
#define         MSG_TASK_START             0X1		 //TODO: il valore e' da rivedere
#define         MSG_TASK_STOP  		       0X2 		 //TODO: il valore e' da rivedere

#define EVT_CHECK(var, EVTMASK)             (EVTMASK == (var&EVTMASK))

//*********************** common communication values ********************************************************
#define         ETH_CONTROL_DGRAM_PAYLOAD_SIZE  64   /**<size of payload of eth datagram used for system control */
#define         ETH_RUN_DGRAM_PAYLOAD_SIZE      64   /**<size of payload of eth datagram used exceghe information in normal running mode */

#define         SYSCONTROL_SOCKET_PORT          3333
#define         RUNNING_SOCKET_PORT             3334

//*********************** for debug popouse *******************************************************************
#ifdef _DEBUG_PRINT_
    #define APPL_CHECKandPRINT_ERROR(res, t,o,i)\
            if(eores_OK != res)\
               eo_errman_Error(eo_errman_GetHandle(), t, o, i);\

    #define APPL_ERRMAN_ERROR(t,o,i)   eo_errman_Error(eo_errman_GetHandle(), t, o, i);
#else

    #define APPL_CHECKandPRINT_ERROR(res, t,o,i)    ;
    #define APPL_ERRMAN_ERROR(t,o,i)                ;
#endif


// - declaration of public user-defined types ------------------------------------------------------------------------- 
// empty-section
    
// - declaration of extern public variables, ...deprecated: better using use _get/_set instead ------------------------
// empty-section

// - declaration of extern public functions ---------------------------------------------------------------------------
// empty-section

// - doxy end ---------------------------------------------------------------------------------------------------------
// empty-section

#endif  // include-guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



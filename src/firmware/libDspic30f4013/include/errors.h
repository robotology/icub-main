#ifndef ERRORS_H
#define ERRORS_H

// error codes
#define ERR_MUX_INDEXING            1
#define ERR_DAC_VALUE2BIG           2
// CAN command parameters invalid 
#define ERR_CAN_PARAMETERS_INVALID  3
// CAN command unavailable 
#define ERR_CAN_COMMAND_UNAVAILABLE 4

// SG to TF matrix indexing invalid 
#define ERR_CAN_MATRIX_INDEXING     5

// IIR Filter Number of biquads too big  
#define ERR_CAN_IIR_NBQ2BIG         6
// IIR Filter Coeff Indexing error  
#define ERR_CAN_IIR_COEF_INDEXING   7

// SW CAN Messages Rx buffer overflow
#define ERR_CAN_RXBUFF_OVERFLOW     8

void EEnqueue(unsigned x);
#define SendCanProblem() {  i = (msg->CAN_Per_Msg_Class << 8 ) | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( msg->CAN_Poll_Msg_Source ); datalen = 4; Txdata[0]=msg->CAN_Per_Msg_PayLoad[0];Txdata[1] ='B';Txdata [2] ='U';Txdata[3] ='G';}

#endif

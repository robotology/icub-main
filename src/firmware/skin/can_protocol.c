#include "can_protocol.h"

#define CAN_MSG_CLASS_BOARDTEST 0x200
// For messages of class 010 the meaning of data/ID is defined as follows:
//  -------------------------- -------------------------
// |           11b            |           8B            |
//  -------  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     | B[0]   |     B[1-7]     |
// |class	| Source | Dest   | C type |    Payload     |
//  -------  -------  -------  -------  -------  ------- 
// Frame formats
#define CAN_STANDARD_FORMAT 0
#define CAN_EXTENDED_FORMAT 1

// Frame types
#define CAN_DATA_FRAME        0
#define CAN_REMOTE_FRAME    1

#define CAN_MSG_CLASS_POLLING 0x000
// For messages of class 000 the meaning of data/ID is defined as follows:
//  -------------------------- -------------------------
// |           11b            |           8B            |
//  -------  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     | B[0]   |     B[1-7]     |
// | class  | Source | Dest   | C type |    Payload     |
//  -------  -------  -------  -------  -------  ------- 

#define CAN_MSG_CLASS_PERIODIC 0x300
// For messages of class 001 the meaning of data/ID is defined as follows:
//  -------------------------- ----------------
// |           11b            |        8B      |
//  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     |      B[0-7]    |
// |class	| Source | Type   |     Payload    |
//  -------  -------  -------  -------  ------- 

#define CAN_MSG_CLASS_BOARDTEST 0x200
// For messages of class 010 the meaning of data/ID is defined as follows:
//  -------------------------- -------------------------
// |           11b            |           8B            |
//  -------  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     | B[0]   |     B[1-7]     |
// |class	| Source | Dest   | C type |    Payload     |
//  -------  -------  -------  -------  -------  ------- 

#define CAN_MSG_CLASS_TACTSENSOR 0x300
// For messages of class 011 the meaning of data/ID is defined as follows:
//  ------------------------------- ------------------------
// |           11b                 |           8B            |
//  -------  -------  ------------- -------  -------  ------- 
// | 3b     |   4b     |   4b      |     B[0]    |     B[1-7]   |
// |class	| BoardID  | Ntriangle |   Payload                  |
//  -------  -------  -------  -------  -------  ------- 
/*
B [0]
It is divided in 4 bits of Options and 4 bits for the payload
Options
Bit 7 : Message number (there are two messages for sending all the data coming from        each triangle)  
Bit 6 : Resolution : 8 or 16 bits  (respectively 1 and 0 ) 
Bit 5 : NU
Bit 4 : NU
Bit 3:1 Pressure Measurements

B[1-7] Pressure Measurements
*/
#define CAN_MSG_CLASS_LOADER   7

// For messages of class 011 the meaning of data/ID is defined as follows:
//  -------------------------- ------------------------
// |           11b            |           8B            |
//  -------  -------  -------   -------  -------  ------- 
// | 3b     | 4b     |   4b  |  B[0]   |     B[1-7]     |
// |class	| Source | Dest  |   type |    Payload     |
//  -------  -------  -------  -------  -------  ------- 



#define CAN_TX_SOFTWARE_BUFFER_SIZE 10

//
// Needed CAN test commands for AdC (class 010)
//

// Incoming messages


	// board_MODE
				
#define SIXTEEN_BITS 0x0
#define	EIGHT_BITS   0x1
#define CALIB        0x3
				
//CONFIG_TYPE
				
#define CONFIG_SINGLE  0x00 (12 indipendent measurements)
#define CONFIG_THREE   0x01 (3 macro areas) 
#define CONFIG_ALL     0x02 (1 macro area) 

// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _HEXPARSER_H_
#define _HEXPARSER_H_


/** @file       hexParser.h
    @brief      This header file implements public interface to a parser for Intel HEX file.
    @author     marco.accame@iit.it
    @date       05/12/2009
**/

/** @defgroup hexparser Parser for HEX files
    The HEX parser allows ...... 
 
    @todo acemor-facenda: do documentation.
    
    @{        
 **/


// - external dependencies --------------------------------------------------------------------------------------------

//#include "shalPART.h"
#include "eEcommon.h"


// - public #define  --------------------------------------------------------------------------------------------------




// - declaration of public user-defined types ------------------------------------------------------------------------- 

typedef enum               
{
    hexparres_OK                = 0,        // compatible with common type for future change
    hexparres_NOK_generic       = -1,
    hexparres_NOK_finished      = -2,        // not compatible .......
} hexparResult_t; 


// - declaration of extern public functions ---------------------------------------------------------------------------


extern hexparResult_t hexpar_init(char *filename);
extern hexparResult_t hexpar_deinit(void);

extern hexparResult_t hexparVerify(const uint32_t addrlow, const uint32_t addrhigh, uint32_t *lower, uint32_t *upper);

extern hexparResult_t hexparVerifyBin(const uint32_t addrlow, const uint32_t addrhigh, eEmoduleInfo_t **proc);

extern hexparResult_t hexparErasePages(uint32_t lower, uint32_t upper);

extern hexparResult_t hexparGetData(uint8_t **data, uint8_t *size, uint32_t *addrdata, uint8_t *eof_flag);

extern hexparResult_t hexparGetBinData(uint8_t **data, uint16_t *size, uint32_t *addrdata, uint8_t *eof_flag);

extern hexparResult_t hexparWriteData(uint32_t address, uint8_t *data, uint16_t size);

extern hexparResult_t hexpar_page_get(const uint32_t pagemaxsize,  uint32_t *pageaddress, uint8_t *pagedata, uint32_t *pagesize);    



/** @}            
    end of group hexparser 
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




#include "prog.h"

/* Control bits description */

#define FLASH_FIU_CNTL_BUSY   0x8000u
#define FLASH_FIU_CNTL_IFREN  0x0040u
#define FLASH_FIU_CNTL_XE     0x0020u
#define FLASH_FIU_CNTL_YE     0x0010u
#define FLASH_FIU_CNTL_PROG   0x0008u
#define FLASH_FIU_CNTL_ERASE  0x0004u
#define FLASH_FIU_CNTL_MAS1   0x0002u
#define FLASH_FIU_CNTL_NVSTR  0x0001u

#define FLASH_FIU_PE_DPE      0x8000u
#define FLASH_FIU_PE_IPE      0x4000u
#define FLASH_FIU_PE_ROW      0x03ffu

#define FLASH_FIU_EE_DEE      0x8000u
#define FLASH_FIU_EE_IEE      0x4000u
#define FLASH_FIU_EE_PAGE     0x007fu

#define FLASH_FIU_IE_0        0x0001u
#define FLASH_FIU_IE_1        0x0002u
#define FLASH_FIU_IE_2        0x0004u
#define FLASH_FIU_IE_3        0x0008u
#define FLASH_FIU_IE_4        0x0010u
#define FLASH_FIU_IE_5        0x0020u
#define FLASH_FIU_IE_6        0x0040u
#define FLASH_FIU_IE_7        0x0080u
#define FLASH_FIU_IE_8        0x0100u
#define FLASH_FIU_IE_9        0x0200u
#define FLASH_FIU_IE_10       0x0400u
#define FLASH_FIU_IE_11       0x0800u

#define FLASH_FIU_IS_0        0x0001u
#define FLASH_FIU_IS_1        0x0002u
#define FLASH_FIU_IS_2        0x0004u
#define FLASH_FIU_IS_3        0x0008u
#define FLASH_FIU_IS_4        0x0010u
#define FLASH_FIU_IS_5        0x0020u
#define FLASH_FIU_IS_6        0x0040u
#define FLASH_FIU_IS_7        0x0080u
#define FLASH_FIU_IS_8        0x0100u
#define FLASH_FIU_IS_9        0x0200u
#define FLASH_FIU_IS_10       0x0400u
#define FLASH_FIU_IS_11       0x0800u

/* Flash Interface Units parameters */

#define FLASH_PAGE_LENGTH     0x0100u
#define FLASH_PAGE_MASK       0x00ffu 
#define FLASH_PAGE_SHIFT      8u

#define FLASH_RAW_LENGTH      0x0020u
#define FLASH_RAW_MASK        0x001fu
#define FLASH_RAW_SHIFT       5u



                                     
void * flashmemCopyXtoP  ( void *dest, const void *src, size_t count );
void * flashmemCopyPtoX  ( void *dest, const void *src, size_t count );


void flashHWErasePage        ( arch_sFlash * FiuBase, 
                                      tpMemCopy pMemCopy, 
                                      UWord16 Address );
void flashHWProgramWord      ( arch_sFlash * FiuBase, 
                                      tpMemCopy pMemCopy, 
                                      UWord16 Address, 
                                      UWord16 * pData );
                                       
                                      
                                      
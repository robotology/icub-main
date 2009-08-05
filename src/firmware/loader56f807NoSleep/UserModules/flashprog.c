/*****************************************************************************
* Modules Included:  flashHWErasePage()
*					 flashmemCopyXtoP()
*                    flashmemCopyPtoX()
*					 flashHWErasePage()
*                    flashHWProgramWord()
*
******************************************************************************/
#include "bootloader.h"
#include "comCAN.h"
#include "sparser.h"
#include "prog.h"
#include "flashprog.h"

/*****************************************************************************
*
* Module:         flashHWErasePage()
*
* Description:    Erase flash page
*
* Returns:        None
*
* Arguments:      FiuBase - base address of Flash Information Unit register
*                           block
*                 pMemCopy - pointer to function that can copy data from X
*                           memory into destination memory
*                 Address  - address anywhere within page
*
* Range Issues:   None
*
* Special Issues: None
*
* Test Method:    None
*
*****************************************************************************/
void flashHWErasePage( arch_sFlash * FiuBase, tpMemCopy pMemCopy, UWord16 Address )
{


   UWord16 TmpData = 0;
   UWord16 TmpWord;

   /* Check flash mode - it shoud be Standby or Read */

   while (FiuBase->ControlReg & ~FLASH_FIU_CNTL_IFREN)
   {
   }

   /* disable all interrupts */
//   periphMemWrite( 0, &FiuBase->IntSourceReg);
   FiuBase->IntSourceReg = 0;

   /* Enable erase by setting IEE and page number in FIU_EE register */
//   periphMemWrite( FLASH_FIU_EE_IEE | ((Address & 0x7FFF)>> FLASH_PAGE_SHIFT),
//                  &FiuBase->EraseReg);

   FiuBase->EraseReg = FLASH_FIU_EE_IEE | ((Address & 0x7FFF)>> FLASH_PAGE_SHIFT);


   /* Write any value to page to start erase */
   (*pMemCopy)((UWord16 *)Address, &TmpData, sizeof(UWord16));


   /* wait while erase operation will be completed */
   while (FiuBase->ControlReg & FLASH_FIU_CNTL_BUSY)
   {
   }

#if 0
   /* check errors */
   TmpWord = periphMemRead( &FiuBase->IntSource );

   if (TmpWord & 0x0003)   /* IS[2] has been set in this moment */
                         /* Illegal read/write access to flash during erase */
   {
//      assert (FALSE); /* Illegal read/write access to flash during erase */
   }
#endif

   FiuBase->EraseReg = 0;
   FiuBase->IntSourceReg = 0;

}

/*****************************************************************************
*
* Module:         flashHWProgramWord()
*
* Description:    Program one word into flash
*
* Returns:        None
*
* Arguments:      FiuBase - base address of Flash Information Unit register
*                           block
*                 pMemCopy - pointer to function that can copy data from X
*                           memory into destination memory
*                 Address  - address anywhere within page
*                 pData    - pointer to data word.
*
* Range Issues:   None
*
* Special Issues: None
*
* Test Method:    None
*
*****************************************************************************/

void flashHWProgramWord ( arch_sFlash * FiuBase, tpMemCopy pMemCopy,
                                 UWord16 Address, UWord16 * pData )
{
   UWord16 TmpWord;
   UWord16 TmpWord2;

   /* Check flash mode - it shoud be Standby or Read */
   while ( FiuBase->ControlReg & ~FLASH_FIU_CNTL_IFREN )
   {
//      assert (FALSE) /* Flash busy. */
   }

   /* clear all interrupts */
   FiuBase->IntSourceReg = 0;

   /* disable all interrupts */
   FiuBase->IntReg = 0;

   /* Enable programming by setting IPE and raw number in FIU_PE register */
   FiuBase->ProgramReg = FLASH_FIU_PE_IPE | ((Address) >> FLASH_RAW_SHIFT);

   /* Write the data */

   (*pMemCopy)((UWord16 *)Address, pData, sizeof(UWord16));

   while ( FiuBase->ControlReg & FLASH_FIU_CNTL_BUSY)
   {
   }

#if 0
   /* Check errors */
   TmpWord = FiuBase->IntSourceReg;

   if (TmpWord & 0x0005)   /* IS[1] has been set in this moment  */
                           /* Illegal read/write access to flash during programming */
   {
      assert (FALSE) /* Illegal read/write access to flash during programming */
   }
#endif

   FiuBase->ProgramReg = 0;
   FiuBase->IntSourceReg = 0;

}


/*****************************************************************************
*
* Module:         flashmemCopyXtoP()
*
* Description:    Copy src words from X:src to P:dest memory location
*                 Register usage:
*                    R2 - dest
*                    R3 - src
*                    Y0 - count, tmp variable
*
* Returns:        (void *)(dest + count)
*
* Arguments:      dest - data destination
*                 scr  - data source
*                 count - data length in words
*
* Range Issues:   0 <= count < 8191
*
* Special Issues: Inline assembler used, "do" hardware cycle used
*                 No error checking for incorrect counter value.
*
* Test Method:    bootest.mcp
*
*****************************************************************************/

asm void * flashmemCopyXtoP ( void *dest, const void *src, size_t count )
{
			tstw    Y0
			beq     EndDo
			do      Y0,EndDo
			move    X:(R3)+,Y0
			move    Y0,P:(R2)+
EndDo:
			/* R2 - Contains *dest return value */
			rts
}


/*****************************************************************************
*
* Module:         flashmemCopyPtoX()
*
* Description:    Copy src words from P:src to X:dest memory location
*                 Register usage:
*                    R2 - dest
*                    R3 - src
*                    Y0 - count, tmp variable
*
* Returns:        (void *)(dest + count)
*
* Arguments:      dest - data destination
*                 scr  - data source
*                 count - data length in words
*
* Range Issues:   0 <= count < 8191
*
* Special Issues: Inline assembler used, "do" hardware cycle used
*                 No error checking for incorrect counter value.
*
* Test Method:    bootest.mcp
*
*****************************************************************************/

asm void * flashmemCopyPtoX ( void *dest, const void *src, size_t count )
{
			tstw    Y0
			beq     EndDo
			do      Y0,EndDo
			move    P:(R3)+,Y0
			move    Y0,X:(R2)+
EndDo:
			/*  R2 - Contains *dest return value */
			rts
}

/*****************************************************************************/




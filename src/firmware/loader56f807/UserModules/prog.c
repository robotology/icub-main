/*****************************************************************************
*
* File Name:         prog.c
*
* Description:       Flash programming and received data writing
*
* Modules Included:  progPlaceData()
*                    progWriteFlashPage()
*                    progSaveData()
*                    progGetMemDescription()
*                    flashHWErasePage()
*                    flashHWProgramWord()
*
*
*****************************************************************************/

#include "bootloader.h"
#include "flashprog.h"
#include "comCAN.h"
#include "sparser.h"
#include "prog.h"

extern void * archStartDelayAddress;


/* P memory map */
static Area progPBaundary[] =
   {
     /* Program flash */
      {
         /* StartAddress    */      0x0004,
         /* EndAddress      */      0x7fff,
         /* Description     */      AREA_P_MEMORY | AREA_FLASH,
         /* FIUBaseAddress  */      (arch_sFlash *)(UWord16 *)&PFIU_CNTL,
      },
      /* Program flash 2 */
      {
         /* StartAddress    */      0x8000,
         /* EndAddress      */      0xefff,
         /* Description     */      AREA_P_MEMORY | AREA_FLASH,
         /* FIUBaseAddress  */      (arch_sFlash *)(UWord16 *)&PFIU2_CNTL,
      },
      /* Program RAM */
      {
         /* StartAddress    */      0xf000,
         /* EndAddress      */      0xf7ff,
         /* Description     */      AREA_P_MEMORY | AREA_RAM,
         /* FIUBaseAddress  */      0,
      },
      /* Boot flash (Reset and COP vectors) */
      {
         /* StartAddress    */      0xf800,
         /* EndAddress      */      0xf803,
         /* Description     */      AREA_P_MEMORY | AREA_FLASH | AREA_DISABLED,
         /* FIUBaseAddress  */      0,
      },
      /* Boot flash, bootloader code location */
      {
         /* StartAddress    */      0xf804,
         /* EndAddress      */      0xffff,
         /* Description     */      AREA_P_MEMORY | AREA_FLASH | AREA_DISABLED | AREA_ERROR,
         /* FIUBaseAddress  */      0,
      }
   };


/* X memory map */
static Area progXBaundary[] =
   {
      /* Internal data RAM, used for bootloader stack and buffers */
      {
         /* StartAddress    */      0x0000,
         /* EndAddress      */      0x0fff,
         /* Description     */      AREA_X_MEMORY | AREA_RAM | AREA_DISABLED,
         /* FIUBaseAddress  */      0,
      },
      /* Peripheral registers */
      {
         /* StartAddress    */      0x1000,
         /* EndAddress      */      0x17ff,
         /* Description     */      AREA_X_MEMORY | AREA_RAM | AREA_DISABLED,
         /* FIUBaseAddress  */      0,
      },
      /* Reserved 1 */
      {
         /* StartAddress    */      0x1800,
         /* EndAddress      */      0x1fff,
         /* Description     */      AREA_X_MEMORY | AREA_RAM | AREA_DISABLED,
         /* FIUBaseAddress  */      0,
      },
      /* Data Flash */
      {
         /* StartAddress    */      0x2000,
         /* EndAddress      */      0x3fff,
         /* Description     */      AREA_X_MEMORY | AREA_FLASH,
         /* FIUBaseAddress  */      (arch_sFlash *)(UWord16 *)&DFIU_CNTL,
      },
      /* External Data Memory */
      {
         /* StartAddress    */      0x4000,
         /* EndAddress      */      0xff7f,
         /* Description     */      AREA_X_MEMORY | AREA_RAM,
         /* FIUBaseAddress  */      0,
      },
      /* Core Configuration Registers */
      {
         /* StartAddress    */      0xff80,
         /* EndAddress      */      0xffff,
         /* Description     */      AREA_X_MEMORY | AREA_RAM | AREA_DISABLED,
         /* FIUBaseAddress  */      0,
      }
   };

#define P_BAUNDARY_NUMBER  (sizeof(progPBaundary) / sizeof(Area))
#define X_BAUNDARY_NUMBER  (sizeof(progXBaundary) / sizeof(Area))

//typedef void * (*tpMemCopy)( void *, const void *, size_t  );
//typedef int    (*tpMemCmp)( const void *, const void *, size_t );

static UWord16           progData    [SPRS_BUFFER_LEN];
static UWord16         * progDataPointer = progData;
static UWord16           progAddress;
static UWord16           progLength;
static bool              progMoreData;
static mem_eMemoryType   progMemoryType = XData;
static tpMemCopy         pToMemCopy;
static tpMemCopy         pFromMemCopy;

static UWord16           progFlashBuffer  [FLASH_PAGE_LENGTH];
static Area            * progFlashArea;        /* if NULL no current area */
static UWord16           progFlashPageAddress; /* current flash page address */

UWord16                  progProgCounter;
UWord16                  progDataCounter;
UWord16                  progIndicatorCounter;

UWord16                  progDelayFlag;
UWord16                  progDelayValue;

/* Local functions prototypes */

static Area * progGetMemDescription  ( Area * AreaArray,
                                       UWord16 AreaNumber,
                                       UWord16 Address );

static void progWriteFlashPage      ( void );


/*****************************************************************************
*
* Module:         progPlaceData()
*
* Description:    Place binary data into appropriate memory location
*
* Returns:        None
*
* Arguments:      ProgEnableFlag - true enables flash programming
*
* Range Issues:   None
*
* Special Issues: Use a lot of global variables
*
* Test Method:    boottest.mcp
*
*****************************************************************************/
void progPlaceData(bool ProgEnableFlag)
{
   UWord16        i;
   UWord16        CurrentLength;
   Area        *  TmpProgArea;


   if (progMoreData)
   {
      /* check for bootloader delay variable */
      if ((progMemoryType == PData) &&
         ((progAddress <= (UInt16)&archStartDelayAddress) && ((UInt16)&archStartDelayAddress < (progAddress + progLength))))
      {
         /* set flag, save value and replace to new temporary value for Delay variable */
         progDelayFlag     = TRUE;
         progDelayValue    =  *(progDataPointer + ((UInt16)&archStartDelayAddress - progAddress ));
         *(progDataPointer + ((UInt16)&archStartDelayAddress - progAddress)) = 0xfffe;
      }
      while (progLength)
      {
         if (progMemoryType == PData)
         {
            TmpProgArea = progGetMemDescription ( progPBaundary, P_BAUNDARY_NUMBER, progAddress );
         }
         else
         {
            TmpProgArea = progGetMemDescription ( progXBaundary, X_BAUNDARY_NUMBER, progAddress );
         }

         if ((( progFlashArea != NULL ) && ( progFlashArea != TmpProgArea )) ||
               ( ( progFlashArea == TmpProgArea ) &&
               ( (progAddress ^ progFlashPageAddress) & ~FLASH_PAGE_MASK )))
         {
            if ( ProgEnableFlag )
            {  /* save flash previus data */
               progWriteFlashPage();

               progFlashArea = NULL;
            }
            else
            {
               break;
            }
         }
         else
         {
            CurrentLength = (TmpProgArea->EndAddress - progAddress) + 1;

            CurrentLength = (CurrentLength > progLength) ? progLength: CurrentLength;

            if ((TmpProgArea->Description & AREA_DISABLED) == 0)
            {
               if (TmpProgArea->Description & AREA_P_MEMORY )
               {
                  pFromMemCopy  = flashmemCopyPtoX;
                  pToMemCopy    = flashmemCopyXtoP;
               }
               else
               {
                  pFromMemCopy  = bootmemCopyXtoX;
                  pToMemCopy    = bootmemCopyXtoX;
               }
               if (TmpProgArea->Description & AREA_FLASH)
               {  /* Flash */
                  if (progFlashArea == NULL)
                  {  /* save data from flash */

                     pFromMemCopy(progFlashBuffer, (void *)(progAddress & ~FLASH_PAGE_MASK), FLASH_PAGE_LENGTH);

                     pFromMemCopy = NULL;

                     progFlashPageAddress = progAddress & ~FLASH_PAGE_MASK;
                     if (((progFlashPageAddress ^ TmpProgArea->StartAddress) & ~FLASH_PAGE_MASK) == 0)
                     {
                        progFlashPageAddress = TmpProgArea->StartAddress;
                     }
                     progFlashArea = TmpProgArea;
                  }
                  if ( CurrentLength > ( FLASH_PAGE_LENGTH - ( progAddress & FLASH_PAGE_MASK )))
                  {
                     CurrentLength =  FLASH_PAGE_LENGTH - (progAddress & FLASH_PAGE_MASK);
                  }
                  /* save data into flash page buffer */
                  bootmemCopyXtoX(&(progFlashBuffer[progAddress & FLASH_PAGE_MASK]), progDataPointer, CurrentLength);

               }
               else
               {  /* area RAM */
                  pToMemCopy((void *)progAddress, (void *)progDataPointer, CurrentLength);
                  progFlashArea = NULL;
               }
            }
            else
            {
               if ( TmpProgArea->Description & AREA_ERROR )
               {
                  userError(INDICATE_ERROR_PROTECTED_BOOT_SECTION);
               }
            }
            progLength        -= CurrentLength;
            progAddress       += CurrentLength;
            progDataPointer   += CurrentLength;

         }
      }
      if (progLength == 0)
      {
         progMoreData = FALSE;
      }
   }
   else
   {
      if (ProgEnableFlag && (progFlashArea != NULL) )
      {
         /* save flash buffer */
         progWriteFlashPage();

         progFlashArea = NULL;
      }
   }
}

/*****************************************************************************
*
* Module:         progPlaceDelayValue()
*
* Description:    Write stored value of Delay timeout
*
* Returns:        None
*
* Arguments:      None
*
* Range Issues:   None
*
* Special Issues: a lot of global vars
*
* Test Method:    boottest.mcp
*
*****************************************************************************/
void progPlaceDelayValue ( void )
{
   if (progDelayFlag)
   {
#if defined (PEcfg_56F827BootLoader)
      flashHWProgramWord ( (arch_sFlash *)(UWord16 *)&PFIU_CNTL, flashmemCopyXtoP, (UInt16)&archStartDelayAddress, &progDelayValue );
#else
      flashHWProgramWord ( (arch_sFlash *)(UWord16 *)&PFIU_CNTL, flashmemCopyXtoP, (UInt16)&archStartDelayAddress, &progDelayValue );
#endif
   }
}


/*****************************************************************************
*
* Module:         progWriteFlashPage()
*
* Description:    Write one page into flash
*
* Returns:        None
*
* Arguments:      None
*
* Range Issues:   None
*
* Special Issues: a lot of global vars
*
* Test Method:    boottest.mcp
*
*****************************************************************************/

void progWriteFlashPage(void)
{
   int        i;
   UWord16    TmpWord;
   UWord16  * TmpFlashAddress;

   TmpFlashAddress = (UWord16 *)(progFlashPageAddress - (progFlashPageAddress & FLASH_PAGE_MASK));

   if (progFlashArea->Description & AREA_P_MEMORY )
   {
      pToMemCopy    = flashmemCopyXtoP;
      pFromMemCopy  = flashmemCopyPtoX;
   }
   else
   {
      pToMemCopy    = bootmemCopyXtoX;
      pFromMemCopy  = bootmemCopyXtoX;
   }

   /* Erase page */
   for (i= progFlashPageAddress & FLASH_PAGE_MASK; i < FLASH_PAGE_LENGTH; i++)
   {
      *(pFromMemCopy)( &TmpWord, (TmpFlashAddress + i), 1);
      if (TmpWord != 0xffff)
      {
#if !defined(NO_FLASH_PROGRAM)
         flashHWErasePage( progFlashArea->FIUBaseAddress, pToMemCopy, progFlashPageAddress);
#endif /* !defined(NO_FLASH_PROGRAM) */
         break;
      }
   }

   for (i= progFlashPageAddress & FLASH_PAGE_MASK; i < FLASH_PAGE_LENGTH; i++)
   {
      *(pFromMemCopy)( &TmpWord, (TmpFlashAddress + i), 1);
      if (TmpWord != 0xffff)
      {
         userError(INDICATE_ERROR_FLASH);
      }
   }

   /* program flash */
   for (i= progFlashPageAddress & FLASH_PAGE_MASK; i < FLASH_PAGE_LENGTH; i++)
   {
#if !defined(NO_FLASH_PROGRAM)
      flashHWProgramWord ( progFlashArea->FIUBaseAddress, pToMemCopy,
                           (UWord16)(TmpFlashAddress + i),
                            progFlashBuffer + i );
#endif /* !defined(NO_FLASH_PROGRAM) */
   }

   /* verify flash */
   for (i= progFlashPageAddress & FLASH_PAGE_MASK; i < FLASH_PAGE_LENGTH; i++)
   {
      *(pFromMemCopy)( &TmpWord, (TmpFlashAddress + i), 1);
#if !defined(NO_FLASH_PROGRAM)
      if (TmpWord != progFlashBuffer[i] )
      {
         userError(INDICATE_ERROR_FLASH);
      }
#endif /* !defined(NO_FLASH_PROGRAM) */
   }

   pToMemCopy    = NULL;
   pFromMemCopy  = NULL;
}

/*****************************************************************************
*
* Module:         progSaveData()
*
* Description:    Save data after S-Record parser into progData buffer, call
*                 progPlaceData()
*
* Returns:        None
*
* Arguments:      pData - pointer to data
*                 Length - data length
*                 Address - start address to put data
*                 MemoryType - memory space to put data
*
* Range Issues:   None
*
* Special Issues: None
*
* Test Method:    boottest.mcp
*
*****************************************************************************/
void progSaveData ( UWord16 * pData, UWord16 Length, UWord16 Address, mem_eMemoryType MemoryType)
{

   if (progMoreData)
   {
      userError(INDICATE_ERROR_OVERRUN);
   }

   bootmemCopyXtoX(progData,pData,SPRS_BUFFER_LEN);

   progDataPointer   = progData;
   progLength        = Length;
   progAddress       = Address;
   progMemoryType    = MemoryType;

   if (MemoryType == PData)
   {
      progProgCounter  += Length;   /* 0xffff overflow does not detected */
   }
   else
   {
      progDataCounter  += Length;
   }

   progMoreData      = TRUE;

   progPlaceData(TRUE);

}

/*****************************************************************************
*
* Module:         progGetMemDescription()
*
* Description:    Search through AreaArray and find memory area that contains
*                 Address
*
* Returns:        pointer to found memory area description
*
* Arguments:      AreaArray - pointer to memory array
*                 AreaNumber - number of items in AreaArray
*                 Address  - address to search
*
* Range Issues:   None
*
* Special Issues: None
*
* Test Method:    boottest.mcp
*
*****************************************************************************/

Area * progGetMemDescription(Area * AreaArray, UWord16 AreaNumber, UWord16 Address)
{
   int i;
   for (i = 0; i < AreaNumber; i++)
   {
      if (Address <= AreaArray[i].EndAddress )
      {
         return &(AreaArray[i]);
      }
   }
}


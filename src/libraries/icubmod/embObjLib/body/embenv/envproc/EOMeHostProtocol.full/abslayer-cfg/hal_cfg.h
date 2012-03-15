
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _HAL_CFG_H_
#define _HAL_CFG_H_

// --------------------------------------------------------------------------------------------------------------------
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------

// <h> Configuration of HAL
// <i> It holds configuration for objects used in OSAl


// <h> Target 
// <i> Define the target 

//   <o> CPU family         <0=>   Cortex M3
//   <i> Only Cortex M3 is now supported.
#ifndef HAL_CPUFAM
 #define HAL_CPUFAM      0
#endif

//   <o> CPU type  <0x00=>   STM32F1x
//   <i> Only STM32F1x is now supported.
#ifndef HAL_CPUTYPE
 #define HAL_CPUTYPE       0x00
#endif


//   <o> CPU frequency  <72000000=>   72 MHz
//   <i> Only 72 MHz is supported for now.
#ifndef HAL_CPUFREQ
 #define HAL_CPUFREQ       72000000
#endif

// </h>Target


// <h> System 
// <i> sssssssss

//   <o> stack size         <0x0-0xFFFFFFFF:8>
//   <i> define how much stack you want.
#ifndef HAL_SYS_STACKSIZE
 #define HAL_SYS_STACKSIZE      0x00002000
#endif

//   <o> heap size         <0x0-0xFFFFFFFF:8>
//   <i> define how much heap you want.
#ifndef HAL_SYS_HEAPSIZE
 #define HAL_SYS_HEAPSIZE      0x0000B000
#endif


// </h>System

// <h> GPIO 
// <i> sssssssss

//   <h> Nothing to configure in GPIO part
//   </h>Nothing to configure in GPIO part 


// </h>GPIO


// <h> FLASH 
// <i> sssssssss

//   <h> Nothing to configure in FLASH part
//   </h>Nothing to configure in FLASH part 


// </h>FLASH


// <h> Display 
// <i> it si important disable display on ems001, because there isn't

//   <o> SPI Display 320x240  <0=>   Disable  <1=> Enable
//   <i> only SPI is supported for now.
#ifndef HAL_SPIDISPLAY_ENABLE
 #define HAL_SPIDISPLAY_ENABLE  0     
#endif


// </h>Display


// <h> Ethernet 

// <o> ETH mode         <0=> ETH Disabled  <1=> ETH w/ ISR + DMA 
// <i> sssssssss

#ifndef HAL_ETH_ENABLE
 #define HAL_ETH_ENABLE  1     
#endif

//   <o> Size of DMA TX buffers  <1=> One frame  <2=> Two frames <3=> Three frames <4=> Four frames
//   <i> One ETH frame is 1536 bytes
#ifndef HAL_ETH_DMA_TX_BUF
 #define HAL_ETH_DMA_TX_BUF  2     
#endif


//   <o> Size of DMA RX buffers  <1=> One frame  <2=> Two frames <3=> Three frames <4=> Four frames
//   <i> One ETH frame is 1536 bytes
#ifndef HAL_ETH_DMA_RX_BUF
 #define HAL_ETH_DMA_RX_BUF  2     
#endif

// </h>Ethernet


// <e> CAN 1          
// <i> sssssssss

#ifndef HAL_CAN1_ENABLE
 #define HAL_CAN1_ENABLE  0     
#endif


//   <o> Max can frames in input buffer <1-16>
//   <i> Default: 4
#define HAL_CAN1_INPBUFFCAPACITY     2

//   <o> Max can frames in output buffer <1-16>
//   <i> Default: 4
#define HAL_CAN1_OUTBUFFCAPACITY     30


// </e> CAN 0  



// <e> CAN 2          
// <i> sssssssss

#ifndef HAL_CAN2_ENABLE
 #define HAL_CAN2_ENABLE  0     
#endif


//   <o> Max can frames in input buffer <1-16>
//   <i> Default: 4
#define HAL_CAN2_INPBUFFCAPACITY     2

//   <o> Max can frames in output buffer <1-16>
//   <i> Default: 4
#define HAL_CAN2_OUTBUFFCAPACITY     1


// </e> CAN 2  


// <e> SPI 1          
// <i> SPI1 peripheral

#ifndef HAL_SPI1_ENABLE
 #define HAL_SPI1_ENABLE  0     
#endif

//   <h> Nothing to configure in SPI1 part
//   </h>Nothing to configure in SPI1 part 

// </e> SPI 1    


// <e> SPI 2          
// <i> SPI2 peripheral

#ifndef HAL_SPI2_ENABLE
 #define HAL_SPI2_ENABLE  0     
#endif

//   <h> Nothing to configure in SPI2 part
//   </h>Nothing to configure in SPI2 part 

// </e> SPI 2    


// <e> SPI 3          
// <i> SPI3 peripheral

#ifndef HAL_SPI3_ENABLE
 #define HAL_SPI3_ENABLE  0     
#endif

//   <h> Nothing to configure in SPI3 part
//   </h>Nothing to configure in SPI3 part 

// </e> SPI 3    


// <h> Timers          
// <i> Timer peripheral

//   <o> Max number of timers <0-7>
//   <i> Default: 2
#define HAL_TIMERS_NUMBER     2

// </h> Timers    


// </h>






// --------------------------------------------------------------------------------------------------------------------
//------------- <<< end of configuration section >>> ------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



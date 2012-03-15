
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _OSAL_CFG_H_
#define _OSAL_CFG_H_

// --------------------------------------------------------------------------------------------------------------------
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// <h> Configuration of OSAL
// <i> It holds configuration for objects used in OSAL


// <h> Porting specifics 
// <i> sssssssss
//   <o> RTOS type         <0=>   IITmodified-RTXARM
//   <i> Only IITmodified-RTXARM is now supported.
#ifndef OSAL_RTOSTYPE
 #define OSAL_RTOSTYPE      0
#endif


//   <o> Memory model         <0=>   static allocation
//   <i> Only static allocation is now supported.
#ifndef OSAL_MEMMODEL
 #define OSAL_MEMMODEL      0
#endif


// </h>Porting specifics

// <h> Embedded System 
// <i> sssssssss

//   <o> CPU family         <0=>   Cortex M3
//   <i> Only Cortex M3 is now supported.
#ifndef OSAL_CPUFAM
 #define OSAL_CPUFAM      0
#endif

//   <o> CPU Frequency [Hz] <1-1000000000>
//   <i> Specify CPU frequency.
//   <i> Default: 72000000  (72MHz on STM32F107)
#ifndef OSAL_CPUFREQ
 #define OSAL_CPUFREQ       72000000
#endif


// </h>Embedded System


// <h> Scheduler 
// <i> sssssssss


//   <o> Priority of the tick handler <1-15>
//   <i> Set the priority of teh tick handler. Lower number is higher priority.
//   <i> Default: 15
#ifndef OSAL_PRIO
 #define OSAL_PRIO       15
#endif

//   <o> Timer tick value [us] <1-1000000>
//   <i> Set the timer tick, the base of time for OSAL.
//   <i> Default: 1000  (1ms)
#ifndef OSAL_TICK
 #define OSAL_TICK        1000
#endif



//   <o> Stack size for launcher task [bytes] <200-4096:8>
//   <i> Set the stack size for launcher task.
//   <i> Default: 128
#ifndef OSAL_LAUNSTKSIZE
 #define OSAL_LAUNSTKSIZE     1024
#endif

//   <o> Stack size for idle task [bytes] <200-4096:8>
//   <i> Set the stack size for idle task.
//   <i> Default: 128
#ifndef OSAL_IDLESTKSIZE
 #define OSAL_IDLESTKSIZE     512
#endif

//   <o> Total stack size for all the other tasks [bytes] <256-16384:8>
//   <i> Define max. size in bytes of the global stack.
//   <i> Default: 256  (only one task for instance)
#ifndef OSAL_GLOBSTKSIZE
 #define OSAL_GLOBSTKSIZE    20000
#endif



// <e>Round-Robin Task switching
// <i> Enable Round-Robin
// <i> Default: Disabled
#ifndef OSAL_RROBIN
 #define OSAL_RROBIN       0
#endif

// <o> Round-Robin tick time [ticks] <1-1000>
// <i> Define how long a task will execute before a task switch.
// <i> Default: 5
#ifndef OSAL_RROBINTICK
 #define OSAL_RROBINTICK   10
#endif

// </e>



// </h>Scheduler





// <h> OSAL objects

//   <o> Number of user tasks <0-250>
//   <i> Maximum number of tasks that will run at the same time.
//   <i> Default: 6
#ifndef OSAL_TASKNUM
 #define OSAL_TASKNUM     10
#endif

//   <o> Number of timers <0-250>
//   <i> Define max number of timers.
//   <i> Default: 0  (User timers disabled)
#ifndef OSAL_TIMERNUM
 #define OSAL_TIMERNUM    7
#endif


//   <o> Number of mutexes <0-250>
//   <i> Define max. number of mutexes that will run at the same time.
//   <i> Default: 0  (Mutexes not enabled)
#ifndef OSAL_MUTEXNUM
 #define OSAL_MUTEXNUM    8
#endif


//   <o> Number of semaphores <0-250>
//   <i> Define max. number of semaphores that will run at the same time.
//   <i> Default: 0  (Semaphores not enabled)
#ifndef OSAL_SEMAPHORENUM
 #define OSAL_SEMAPHORENUM    4
#endif


//   <o> Number of message queues <0-250>
//   <i> Define max. number of message queues that that will run at the same time.
//   <i> Default: 0  (Message queues not enabled)
#ifndef OSAL_MQUEUENUM
 #define OSAL_MQUEUENUM    9
#endif

//   <o> Total number of messages in message queues <0-1000>
//   <i> Define max. number of messages that can be contained in all the message queues.
//   <i> Default: 0  (Message queues not enabled)
#ifndef OSAL_MQUEUEELEMNUM
 #define OSAL_MQUEUEELEMNUM    64
#endif



// </h>OSAL objects



// </h>


// --------------------------------------------------------------------------------------------------------------------
//------------- <<< end of configuration section >>> ------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// - some controls ----------------------------------------------------------------------------------------------------

#if(0 != OSAL_RTOSTYPE)
    #error only arm-rtx modified by iit is supported so far
#endif


#if(OSAL_MQUEUEELEMNUM < OSAL_MQUEUENUM)
    #warning more messagequeues than messages ...
#endif

// - end of controls --------------------------------------------------------------------------------------------------

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _DSPAL_CFG_H_
#define _DSPAL_CFG_H_

// --------------------------------------------------------------------------------------------------------------------
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------


// <h> Configuration of DSPAL
// <i> It holds configuration of DSPAL

// <h> Porting specifics 
// <i> sssssssss
//   <o> DSP library type         <0=>   CMSISDSP
//   <i> Only CMSISDSP is now supported.
#ifndef DSPAL_DSPLIBTYPE
 #define DSPAL_DSPLIBTYPE      0
#endif


//   <o> CPU family         <0=>   Cortex M3
//   <i> Only Cortex M3 is now supported.
#ifndef DSPAL_CPUFAM
 #define DSPAL_CPUFAM      0
#endif

// </h>Porting specifics

// <h> There is nothing else to configure 
// <i> sssssssss


// </h>There is nothing else to configure 


// </h>






// --------------------------------------------------------------------------------------------------------------------
//------------- <<< end of configuration section >>> ------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



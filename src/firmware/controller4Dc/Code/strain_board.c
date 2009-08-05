#include "strain_board.h"
#include "ad.h"

Int16 _strain[6] = INIT_ARRAY (0);
Int16 _strain_init[6] = INIT_ARRAY (0);
Int16 _strain_old[6] = INIT_ARRAY (0);
Int32 _Feq = 0;

byte  _strain_wtd = STRAIN_SAFE;

/*************************************************************************** 
 * Initialization
 ***************************************************************************/
void init_strain (void)
{
	#if VERSION == 0x0170
		//do nothing
	#elif VERSION == 0x0171
		AD_init ();
		AD_enableIntTriggerA ();
		AD_enableIntTriggerB ();		
	#endif
}

/*************************************************************************** 
 * strain gauges sampling
 ***************************************************************************/
word read_strain(byte jnt, bool sign)
{
	word temp;
	Int32 temporary;	

	#if VERSION == 0x0170
		return _strain[jnt];
	#else if VERSION == 0x0171
		switch (jnt)
		{
			case 0: AD_getChannel16A (2, &temp); break;
			case 1: AD_getChannel16B (2, &temp); break;			
		}

		temporary = (Int32) temp;
		if (!sign)	temporary = -temporary;
		
		return temporary;
	#endif
		
	
}
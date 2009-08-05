#ifndef __2dc_options_h__
#define __2dc_options_h__

#define BOARD_TYPE_2DC  0
#define BOARD_TYPE_4DC  1
#define BOARD_TYPE_2BLL 2
#define BOARD_TYPE_EVM  3

#define BOARD_TYPE 		BOARD_TYPE_2DC

#define JN 				2

#if (JN == 2)
	#define INIT_ARRAY(x) {x,x}
#elif (JN == 4)
	#define INIT_ARRAY(x) {x,x,x,x}
#else
	#error unable to init array for JN != 2,4
#endif
    
#endif
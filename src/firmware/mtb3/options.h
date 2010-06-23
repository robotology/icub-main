#ifndef __options_h__
#define __options_h__

//TRIANGLE OFFSET is used to calculate the Triangle number in the CAN message. It could be 0, 32, 64, 96
#define TRIANGLE_OFFSET 0

#define SIXTEEN_BITS    0
#define EIGHT_BITS      1
#define TEN_BITS        2
#define CALIB           3

#define CONFIG_SINGLE 0
#define CONFIG_THREE 1
#define CONFIG_ALL 2

#define TIMER_SINGLE_256dec   0x3700          //timer value default 3A00
#define TIMER_SINGLE_128dec   0x2D00          //timer value default
#define TIMER_SINGLE_64dec   0x1A00          //timer value default
#define TIMER_THREE    0x2A00
#define TIMER_ALL      0x1A00    


#define _version  0x0200
#define _build_number  7      //changed the I2C speed


#endif

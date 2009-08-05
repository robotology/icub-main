
#ifndef DEFN_H
#define DEFN_H

#define TRUE	1
#define FALSE  0

#define IMG_SCALE 2 

#if (IMG_SCALE==1)
	#define PIC_W	640
	#define PIC_H  480
#elif (IMG_SCALE == 2)
	#define PIC_W 320
	#define PIC_H 240
#elif (IMG_SCALE == 3)
	#define PIC_W 160
	#define PIC_H 120
#elif (IMG_SCALE == 4)
	#define PIC_W 80
	#define PIC_H 60
#endif

//I am too lazy to change these in Flow.cpp
#define PIC_X PIC_W
#define PIC_Y PIC_H


#endif

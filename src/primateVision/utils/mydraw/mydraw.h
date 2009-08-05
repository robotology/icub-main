/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_mydraw MyDraw
 *
 * 
 * Basic functions to draw in an image.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/mydraw/mydraw.cc
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#ifndef __MYDRW__

#include <ipp.h>


namespace iCub {
  namespace contrib {
    namespace primateVision {
      
   
      /** A simple function to draw a cross within an image at a specified location with a specified greyscale intensity.
       * @param imsize Image width and height.
       * @param im Pointer to the image to draw the line in. 
       * @param psb Step in bytes through the input image.
       * @param px Horizontal position of cross.
       * @param py Vertical position of cross.
       * @param col The cross intensity.
       */
      void MyDrawLine(Ipp8u*im,int psb,IppiSize imsize,int px,int py,Ipp32f col);
      
    }
  }
}
#endif

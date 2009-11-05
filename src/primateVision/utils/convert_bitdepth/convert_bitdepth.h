/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_convert_bitdepth Convert_BitDepth
 *
 * 
 * Functions to convert between image depths, and for image normalisation.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/convert_bitdepth/convert_bitdepth.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */

#ifndef CONVERT_BITDEPTH_H
#define CONVERT_BITDEPTH_H

#include <ipp.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


      /** Conversion from 32f to 8u with prior normalisation.
       * @param im_32f Pointer to 32f input image location.
       * @param psb_32f Step in bytes through the 32f input image.
       * @param im_8u Pointer to 8u output image location.
       * @param psb_8u Step in bytes through the 8u output image.
       * @param imsize Image width and height.
       */
      void conv_32f_to_8u_nn(Ipp32f*im_32f,int psb_32f,Ipp8u* im_8u,int psb_8u,IppiSize imsize);

      /** Conversion from 32f to 8u without normalisation. Ensure range is within 0.0 - 255.0.
       * @param im_32f Pointer to 32f input image location.
       * @param psb_32f Step in bytes through the 32f input image.
       * @param im_8u Pointer to 8u output image location.
       * @param psb_8u Step in bytes through the 8u output image.
       * @param imsize Image width and height.
       */
      void conv_32f_to_8u(Ipp32f*im_32f,int psb_32f,Ipp8u* im_8u,int psb_8u,IppiSize imsize);
         
      /** Conversion from 8u to 32f.
       * @param im_8u Pointer to 8u input image location.
       * @param psb_8u Step in bytes through the 8u input image.
       * @param im_32f Pointer to 32f output image location.
       * @param psb_out_32f Step in bytes through the 32f output image.
       * @param imsize Image width and height.
       */
      void conv_8u_to_32f(Ipp8u* im_8u,int psb_8u,Ipp32f*im_32f,int psb_out_32f,IppiSize imsize);

    }
  }
}
#endif

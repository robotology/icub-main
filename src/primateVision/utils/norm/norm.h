/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_norm Norm
 *
 * 
 * Functions to normalise greyscale images.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/norm/norm.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 */

#ifndef __NORM_H
#define __NORM_H

#include <ipp.h>

using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {


       /** Normalisation of a 32f image to the range 0.0 - Val.
       * @param val Maximum entry in normalised output (minimum is 0.0).
       * @param im_in_32f Pointer to 32f input image location.
       * @param psb_in_32f Step in bytes through the 32f input image.
       * @param im_out_32f Pointer to 32f output image location.
       * @param psb_out_32f Step in bytes through the 32f output image.
       * @param imsize Image width and height.
       */
      void normVal_32f(double val,Ipp32f*im_in_32f,int psb_in_32f,Ipp32f*im_out_32f,int psb_out_32f,IppiSize imsize);

     /** Normalisation of a 32f image to the range 0.0 - Val, IN PLACE.
       * @param val Maximum entry in normalised output (minimum is 0.0).
       * @param im_32f Pointer to 32f input image location.
       * @param psb_32f Step in bytes through the 32f input image.
       * @param imsize Image width and height.
       */
      void normValI_32f(double val,Ipp32f*im_32f,int psb_32f,IppiSize imsize);

      /** Normalisation of a 8u image to the range 0 - Val.
       * @param val Maximum entry in normalised output (min is 0, max 255).
       * @param im_in_8u Pointer to 32f input image location.
       * @param psb_in_8u Step in bytes through the 32f input image.
       * @param im_out_8u Pointer to 32f output image location.
       * @param psb_out_8u Step in bytes through the 32f output image.
       * @param imsize Image width and height.
       */
      void normVal_8u(int val, Ipp8u*im_in_8u,int psb_in_8u,Ipp8u*im_out_8u,int psb_out_8u,IppiSize imsize);

     /** Normalisation of a 8u image to the range 0 - Val, IN PLACE.
       * @param val Maximum entry in normalised output (min is 0, max 255).
       * @param im_8u Pointer to 32f input image location.
       * @param psb_8u Step in bytes through the 32f input image.
       * @param imsize Image width and height.
       */
      void normValI_8u(int val,Ipp8u*im_8u,int psb_8u,IppiSize imsize);

    }
  }
}
#endif

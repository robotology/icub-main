/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrew Dankers, maintainer Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __DOG_H
#define __DOG_H

#include <ipp.h>

#define PAD_BORD 8

/** 
  * A processing class that constructs on- and off-centre difference-of-Gaussian maps.
  */
class DoG{
	
public:

    /** Constructor.
     * @param imsize Input image width and height for memory allocation.
     */
    DoG(IppiSize imsize);

    /** Destructor.
     */
    ~DoG();

    /** Processing initiator.
     * @param im Pointer to input image.
     * @param psb_8u Step in bytes through the input image.
     */
    void proc(Ipp8u* im, int psb_8u);

    /** Access to the on-centre output.
     * @return Pointer to the on-centre output image.
     */
    Ipp8u* get_dog_on(){return out_dog_on;}   //on-centre

    /** Access to the off-centre output.
     * @return Pointer to the off-centre output image.
     */
    Ipp8u* get_dog_off(){return out_dog_off;} //off-centre

    /** Access to the magnitude output.
     * @return Pointer to the on/off-centre output image.
     */
    Ipp8u* get_dog_onoff(){return out_dog_onoff;} //absolute difference

    /** Memory width return function.
     * @return Step in bytes through the output image.
     */
    int get_psb(){return psb_o;}

    /** 
     * Convert from 32f precision back to 8u
     */
    void conv_32f_to_8u( Ipp32f* im_i, int p4_, Ipp8u*im_o, int p1_, IppiSize srcsize_);

private:
    Ipp32f *dog;
    Ipp32f *dog_on;
    Ipp32f *dog_off;
    Ipp32f *dog_onoff;
    Ipp32f *tmp1;
    Ipp32f *tmp2;
    Ipp32f *tmp3;
    Ipp32f *in_pad;
    Ipp8u  *in_pad_8u;

    Ipp8u *out_dog_on;
    Ipp8u *out_dog_off;
    Ipp8u *out_dog_onoff;

    int width,height;
    int psb_o,psb_pad,psb_pad_8u;
    IppiSize srcsize,psize;
	
};
#endif

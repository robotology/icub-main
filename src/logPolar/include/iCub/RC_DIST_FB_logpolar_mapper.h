/*
 *  logpolar mapper library. subsamples rectangular images into logpolar.
 *
 *  Copyright (C) 2005 Fabio Berton, LIRA-Lab
 *  RobotCub Consortium, European Commission FP6 Project IST-004370
 *  email:   fberton@dist.unige.it
 *  website: www.robotcub.org
 *
 *  Permission is granted to copy, distribute, and/or modify this program 
 *  under the terms of the GNU General Public License, version 2 or any later
 *  version published by the Free Software Foundation. A copy of the license can be 
 *  found at http://www.robotcub.org/icub/license/gpl.txt
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *  PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 *  $Id: RC_DIST_FB_logpolar_mapper.h,v 1.8 2006/07/25 16:51:05 fberton Exp $
 */

#ifndef RC_DIST_FB_logpolar_mapper_h
#define RC_DIST_FB_logpolar_mapper_h

/**
 * \file rc_dist_fb_logpolar_mapper.h \brief The Log Polar library contains all the needed functions
 * for both the color reconstruction starting from a Bayer pattern image and 
 * the trasformations between cartesian and log polar images.
 */

/**
 * \def RADIAL Each receptive field in fovea will be tangent to a receptive field in the previous ring and one in the next ring.
 */

/**
 * \def TANGENTIAL Each receptive field in fovea will be tangent to the previous and the next receptive fields on the same ring.
 */

/**
 * \def ELLIPTICAL Each receptive field in fovea will be tangent to all its neighbors, having then an elliptical shape.
 */

#define RADIAL  0
#define TANGENTIAL 1
#define ELLIPTICAL 2

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

/**
 * \struct cart2LpPixel 
 * \brief It contains the look-up table for the creation of a log polar image. 
 *
 */
struct cart2LpPixel
{
    int divisor;    /**< Number of cartesian pixels corresponding to the current log polar one.*/
    int *iweight;   /**< Array containing the weight of each cartesian pixel.*/
    int *position;  /**< Array containing the position of each cartesian pixel. \n
                         Note that the plane information is included in this field 
                         (i.e. when only one plane is present it contains
                         the value  (\p y*xSize+x), while in case of three planes, 
                         the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * \struct lp2CartPixel 
 * \brief It contains the look-up table for the remapping of a log polar image into a cartesian one. 
 *
 */
struct lp2CartPixel
{
    int iweight;    /**< Array containing the weight of the log polar pixel 
                         corresponding to the current cartesian one. */
    int *position;  /**< Array containing the cartesian position of each log 
                         polar pixel. \n
                         Note that the plane information is included in this 
                         field (i.e. the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * \brief Allocates the memory for the look-up table for the transformation from 
   a cartesian image to a log polar one 
 * and loads it.
 * @param Table is a pointer to the LUT
 * @param nEcc is the number of rings
 * @param nAng is the number of pixels per ring 
 * @param bayerImg when \a true a bayer pattern image is generated, 
   a color image is generated otherways
 * @param path is the path where the table will be loaded from
 * @return 0 if succeed, 1 if the function can't read the table data, 
   2 if allocation fails
 */
int RCallocateC2LTable (cart2LpPixel *
                        Table, int nEcc, int nAng, bool bayerImg, char *path);

/**
 * \brief Allocates the memory for the look-up table \
   for the transformation from a log polar image to a cartesian one 
 * and loads it.
 * @param Table is a pointer to the LUT
 * @param xSize is the horizontal size of the image
 * @param ySize is the vertical size of the image
 * @param path is the path where the table will be loaded from
 * @return 0 if succeed, 1 if the function can't read the table data, 
   2 if allocation fails
 */
int RCallocateL2CTable (lp2CartPixel * Table, int xSize, int ySize,
                        char *path);

/**
 * \brief Frees the memory from the look-up table.
 * @param Table is a pointer to the LUT
 */
void RCdeAllocateC2LTable (cart2LpPixel * Table);

/**
 * \brief Frees the memory from the look-up table. 
 * @param Table is a pointer to the LUT
 */
void RCdeAllocateL2CTable (lp2CartPixel * Table);

/**
 * \brief Reconstructs the color information from a Bayer pattern image. 
 * @param color is a pointer to a color image
 * @param grey is a pointer to Bayer pattern image
 * @param width is the horizontal size of the image
 * @param height is the vertical size of the image
 */
void RCreconstructColor (unsigned char
                         *color, unsigned char *grey, int width, int height);

/**
 * \brief Generates the look-up table for the transformation from a cartesian 
   image to a log polar one, both images have 
 * a Bayer Pattern structure
 * @param nEcc is number of rings
 * @param nAng is the number of pixels per ring 
 * @param xSize is the width of the cartesian image
 * @param ySize is the height of the cartesian image
 * @param overlap is the overlap amount between receptive fields.
 * \arg \b 0 All the RF's are tangent each other. 
 * \arg \b -1 All their sizes are 1 pixel
 * \arg \b 1 The edges of the RF's pass through the centers of neighboring RF's
 * Allowed values: float numbers between -1 and +infinity
 * @param scaleFact the ratio between the size of the smallest logpolar pixel 
   and the cartesian ones
 * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
 * @param path is the path where the table will be stored
 * @return 0 when there are no errors
 * @return 1 in case of wrong parameters
 * @return 2 in case of allocation problems
 * @return 3 in case of file problems
 */
int RCbuildC2LMapBayer (int nEcc,
                        int nAng,
					    int xSize,
					    int ySize,
                        double overlap, double scaleFact, int mode,
                        char *path);

/**
 * \brief Generates the look-up table for the transformation from a cartesian 
   image to a log polar one, both images are 
 * color images
 * @param nEcc is number of rings
 * @param nAng is the number of pixels per ring 
 * @param xSize is the width of the cartesian image
 * @param ySize is the height of the cartesian image
 * @param overlap is the overlap amount between receptive fields.
 * \arg \b 0 All the RF's are tangent each other. 
 * \arg \b -1 All their sizes are 1 pixel
 * \arg \b 1 The edges of the RF's pass through the centers of neighboring RF's
 * Allowed values: float numbers between -1 and +infinity
 * @param scaleFact the ratio between the size of the smallest logpolar pixel 
   and the cartesian ones
 * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
 * @param path is the path where the table will be stored
 * @return 0 when there are no errors
 * @return 1 in case of wrong parameters
 * @return 2 in case of allocation problems
 * @return 3 in case of file problems
 */
int RCbuildC2LMap (int nEcc, int nAng,
                   int xSize,
                   int ySize,
                   double overlap, double scaleFact, int mode, char *path);

/**
 * \brief Generates the look-up table for the transformation from a log polar 
   image to a cartesian one.
 * @param nEcc is number of rings
 * @param nAng is the number of pixels per ring 
 * @param xSize is the width of the cartesian image
 * @param ySize is the height of the cartesian image
 * @param overlap is the overlap amount between receptive fields.
 * \arg \b 0 All the RF's are tangent each other. 
 * \arg \b -1 All their sizes are 1 pixel
 * \arg \b 1 The edges of the RF's pass through the centers of neighboring RF's
 * Allowed values: float numbers between -1 and +infinity
 * @param scaleFact the ratio between the size of the smallest logpolar pixel and 
   the cartesian ones
 * @param hOffset is the horizontal shift in pixels
 * @param vOffset is the vertical shift in pixels
 * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
 * @param path is the path where the table will be stored
 * @return 0 when there are no errors
 * @return 1 in case of wrong parameters
 * @return 2 in case of allocation problems
 * @return 3 in case of file problems
 */
int RCbuildL2CMap (int nEcc, int nAng,
                   int xSize,
                   int ySize,
                   double overlap,
                   double scaleFact,
                   int hOffset, int vOffset, int mode, char *path);

/**
 * \brief Generates a log polar image from a cartesian one
 * @param lpImg is the output LogPolar image
 * @param cartImg is the input Cartesian image
 * @param Table is the LUT used for the transformation
 * @param lpSize is the size of the log polar image
 * @param bayerImg when true a bayer pattern image is generated, a color image 
   is generated otherways
 */
void RCgetLpImg (unsigned char *lpImg,
                 unsigned char *cartImg,
                 cart2LpPixel * Table, int lpSize, bool bayerImg);

/**
 * \brief Remaps a log polar image to a cartesian one
 * @param cartImg is the output Cartesian image
 * @param lpImg is the input LogPolar image
 * @param Table is the LUT used for the transformation
 * @param cartSize is the size of the log polar image
 */
void RCgetCartImg (unsigned char
                   *cartImg,
                   unsigned char *lpImg, lp2CartPixel * Table, int cartSize);

/**
 * \brief Computes the logarithm index
 * @param nAng is the number of pixels per ring 
 * @return the logarithm index.
 */
double RCgetLogIndex (int nAng);

/**
 * \brief Computes the ratio between the size of the smallest logpolar pixel and 
   the cartesian ones
 * @param nEcc is number of rings
 * @param nAng is the number of pixels per ring 
 * @param xSize is the width of the cartesian image
 * @param ySize is the height of the cartesian image
 * @param overlap is the overlapping amount between pixels (0 = tangent pixels, 1 = ???)
 * @return the scale factor.
 */
double RCcomputeScaleFactor (int nEcc, int nAng, int xSize, int ySize, double overlap);

#endif

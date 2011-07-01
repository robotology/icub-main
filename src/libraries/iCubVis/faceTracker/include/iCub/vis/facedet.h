/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __FACEDET_H__
#define __FACEDET_H__

#include <cv.h>

class faceDetector
{
protected:
    CvHaarClassifierCascade *pCascade;
    CvMemStorage            *pStorage;    

public:
    faceDetector();
    bool init(const char *haarCascadePath);    
    CvSeq *detect(IplImage *pImg);
    ~faceDetector();
};

#endif


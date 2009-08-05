// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: 2008 Dario Figueira
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#ifndef COMMON_FUNCTIONS_HPP
#define COMMON_FUNCTIONS_HPP

extern string itos(int i);

extern void SaveImage( IplImage* image, const char* rootname ); 

extern IplImage* crop(IplImage* img, int MaxWidth, int MinWidth, int MaxHeight, int MinHeight);

extern Keypoint feature2keypoint(struct feature* feat);

#endif //COMMON_FUNCTIONS_HPP

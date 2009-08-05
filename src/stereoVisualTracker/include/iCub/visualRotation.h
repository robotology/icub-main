// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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


#ifndef __VISUAL_ROTATION_H__
#define __VISUAL_ROTATION_H__


class VisualRotation{
protected:
    FrameGrabber *grabber;
    IplImage* frame;
    QuaternionFlow *qFlow;
    double t0;
public:
    VisualRotation(){};
    //    VisualRotation(int nb_points=500);
    virtual ~VisualRotation(){Free();};
    virtual bool InitGrabber();
    void InitInterface();
    void Init(int nb_points=400);
    virtual int Loop();
    virtual void Free();
    void Run();
    void ShowImages();
    virtual int ExecuteCommand(int key);
};


// class VisualRotation{

// protected:
//     FrameGrabber *grabber_left;
//     FrameGrabber *grabber_right;
//     IplImage *frame_left;
//     IplImage *frame_right;
//     DistortionCorrector *right_distortion;
//     DistortionCorrector *left_distortion;
//     Locator3D *stereo;
  
//     StereoQuaternionFlow *flow;
//     bool frame_by_frame;

// public:
//     VisualRotation();
//     ~VisualRotation();
//     void InitGrabbers();
//     void InitVision(const char *paramdir, double chessboard[3]);
//     void InitInterface();
//     int Loop();
//     int ExecuteCommand(int key);
//     void Free();
//     void Run();
//     void ShowImages();
//     void Calibrate3d();
//     void Calibrate2d(int i);
// };



#endif

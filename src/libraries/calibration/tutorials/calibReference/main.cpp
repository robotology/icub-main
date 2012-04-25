/*
 * Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//
// A tutorial on how to use the calibration procedure to find out
// the roto-translation matrix of two sets of matching 3D points.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <iostream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/math.h>
#include <iCub/calibration/calibReference.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::calibration;


int main()
{
    // define our working bounding box
    Vector min(3),max(3);
    min[0]=-5.0; max[0]=5.0;
    min[1]=-5.0; max[1]=5.0;
    min[2]=-5.0; max[2]=5.0;

    // define the "unknown" transformation    
    // translation
    Vector p(3);
    p[0]=0.3; p[1]=-2.0; p[2]=3.1;
    // rotation
    Vector euler(3);
    euler[0]=M_PI/4.0; euler[1]=M_PI/6.0; euler[2]=M_PI/3.0;

    Matrix H=euler2dcm(euler);
    H(0,3)=p[0]; H(1,3)=p[1]; H(2,3)=p[2];

    // create the calibration
    CalibReferenceWithMatchedPoints calibrator;

    // set the working bounding box where the solution is seeked
    calibrator.setBounds(min,max);

    // produce the two clouds of matching 3D points
    // and feed the calibrator
    for (int i=0; i<10; i++)
    {
        Vector p0=Rand::vector(min,max);
        p0.push_back(1.0);

        Vector p1=H*p0;

        calibrator.addPoints(p0,p1);
    }

    // carry out the calibration
    Matrix Hcap;
    double error;
    double t0=Time::now();
    calibrator.calibrate(Hcap,error);
    double dt=Time::now()-t0;

    cout<<endl;
    cout<<"H"<<endl<<H.toString(3,3).c_str()<<endl;
    cout<<"Hcap"<<endl<<Hcap.toString(3,3).c_str()<<endl;
    cout<<endl;
    cout<<"residual = "<<error<<" [m]"<<endl;
    cout<<"calibration performed in "<<dt<<" [s]"<<endl;
    cout<<endl;

    return 0;
}



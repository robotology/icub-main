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

    // define the "unknown" transformation:
    // translation
    Vector p(3);
    p[0]=0.3; p[1]=-2.0; p[2]=3.1;
    // rotation
    Vector euler(3);
    euler[0]=M_PI/4.0; euler[1]=M_PI/6.0; euler[2]=M_PI/3.0;

    Matrix H=euler2dcm(euler);
    H(0,3)=p[0]; H(1,3)=p[1]; H(2,3)=p[2];

    // ansisotropic scale
    Vector scale(3);
    scale[0]=0.9; scale[1]=0.8; scale[2]=1.1;
    Vector h_scale=scale;
    h_scale.push_back(1.0);

    // create the calibrator
    CalibReferenceWithMatchedPoints calibrator;

    // generate randomly the two clouds of matching 3D points
    for (int i=0; i<10; i++)
    {
        Vector p0=Rand::vector(min,max);
        p0.push_back(1.0);

        // make data a bit dirty (add up noise in the range (-1,1) [cm])
        Vector eps=Rand::vector(Vector(3,-0.01),Vector(3,0.01));
        eps.push_back(0.0);
        Vector p1=h_scale*(H*p0)+eps;

        // feed the calibrator with the matching pair
        calibrator.addPoints(p0,p1);
    }

    // set the working bounding box where the solution is seeked
    calibrator.setBounds(min,max);

    // carry out the calibration
    Matrix Hcap;
    Vector scalecap;
    double error;
    double t0=Time::now();
    calibrator.calibrate(Hcap,scalecap,error);
    double dt=Time::now()-t0;

    // the final report
    cout<<endl;
    cout<<"H"<<endl<<H.toString(3,3).c_str()<<endl;
    cout<<"scale = "<<scale.toString(3,3).c_str()<<endl;
    cout<<endl;
    cout<<"Hcap"<<endl<<Hcap.toString(3,3).c_str()<<endl;
    cout<<"scalecap = "<<scalecap.toString(3,3).c_str()<<endl;
    cout<<endl;
    cout<<"residual error = "<<error<<" [m]"<<endl;
    cout<<"calibration performed in "<<dt<<" [s]"<<endl;
    cout<<endl;

    return 0;
}



#ifndef __RFWR__
#define __RFWR__

/**
*
@ingroup icub_libraries
\defgroup icub_rfwr Rfwr

Receptive field weighted regression.

\section intro_sec Description
Implement RFWR neural network.

\author Alessandra Sciutti and Lorenzo Natale

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/rfwr/rfwr.h
**/

// A wrapper for RFWR for YARP2
// nat, May 07

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

class RFNet; //forward dcl

class RFWR
{
private:
        RFNet *rfnet;

public:
    RFWR();
    ~RFWR();

    void initialize(int n_in,int n_out, 
	     bool diag_only, 
	     bool meta, 
	     double meta_rate,	
	     double penalty,	
	     double init_alpha,
         const yarp::sig::Vector &norm,
         const yarp::sig::Vector &norm_out);

    double train(const yarp::sig::Vector &x, const yarp::sig::Vector &y);
    double train(const yarp::sig::Vector &x, const yarp::sig::Vector &y, yarp::sig::Vector &yp);
    double simulate(const yarp::sig::Vector &x, yarp::sig::Vector &yp, double cutoff=0.001);
    void jacobian(const yarp::sig::Vector &x, yarp::sig::Matrix &J, double cutoff=0.001);

    int getInSize();
    int getOutSize();

    void save(const char * file_ini);
    int load(const char * file_ini); 
};

#endif

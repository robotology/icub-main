/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <iomanip>
#include <string>
#include "iCub/skinDynLib/common.h"

using namespace std;
using namespace yarp::sig;
using namespace iCub::skinDynLib;

// print a matrix nicely
void iCub::skinDynLib::printMatrix(Matrix &m, std::string description, unsigned int precision)
{
    if(description.compare("") != 0)
        cout<<description<<endl;
    for(int i=0;i<m.rows();i++){
	    for(int j=0;j<m.cols();j++)
		    cout<< setiosflags(ios::fixed)<< setprecision(precision)<< setw(precision+3)<<m(i,j)<<"\t";
	    cout<<endl;
    }
}

// print a vector nicely
void iCub::skinDynLib::printVector(yarp::sig::Vector &v, std::string description, unsigned int precision)
{
    if(description.compare("") != 0)
        cout<<description<<endl;
    for(int j=0;j<v.length();j++)
	    cout<< setiosflags(ios::fixed)<< setprecision(precision)<< setw(precision+3)<<v(j)<<"\t";
    cout<<endl;
}



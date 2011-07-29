/*
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
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

#include "iCub/learningMachine/Serialization.h"

namespace iCub {
namespace learningmachine {
namespace serialization {

yarp::os::Bottle& operator<<(yarp::os::Bottle &out, int val) {
    out.addInt(val);
    return out;
}

yarp::os::Bottle& operator<<(yarp::os::Bottle &out, double val) {
    out.addDouble(val);
    return out;
}

yarp::os::Bottle& operator<<(yarp::os::Bottle &out, const yarp::sig::Vector& v) {
    for(int i = 0; i < v.size(); i++) {
        out << v(i);
    }
    out << v.size();
    return out;
}

yarp::os::Bottle& operator<<(yarp::os::Bottle &out, const yarp::sig::Matrix& M) {
    for(int r = 0; r < M.rows(); r++) {
        for(int c = 0; c < M.cols(); c++) {
            out << M(r,c);
        }
    }
    out << M.rows() << M.cols();
    return out;
}

yarp::os::Bottle& operator>>(yarp::os::Bottle &in, int& val) {
    val = in.pop().asInt();
    return in;
}

yarp::os::Bottle& operator>>(yarp::os::Bottle &in, double& val) {
    val = in.pop().asDouble();
    return in;
}

yarp::os::Bottle& operator>>(yarp::os::Bottle &in, yarp::sig::Vector& v) {
    int len;
    in >> len;
    v.resize(len);
    for(int i = v.size() - 1; i >= 0; i--) {
        in >> v(i);
    }
    return in;
}

yarp::os::Bottle& operator>>(yarp::os::Bottle &in, yarp::sig::Matrix& M) {
    int rows, cols;
    in >> cols >> rows;
    M.resize(rows, cols);
    for(int r = M.rows() - 1; r >= 0; r--) {
        for(int c = M.cols() - 1; c >= 0; c--) {
            in >> M(r, c);
        }
    }
    return in;
}



} // serialization
} // learningmachine
} // iCub


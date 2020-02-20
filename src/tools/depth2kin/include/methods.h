/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __DEPTH2KIN_METHODS_H__
#define __DEPTH2KIN_METHODS_H__

#include <string>
#include <deque>

#include <yarp/os/Property.h>
#include <yarp/sig/all.h>

#include <iCub/optimization/matrixTransformation.h>
#include <iCub/learningMachine/IMachineLearner.h>


/************************************************************************/
class Calibrator
{
protected:
    std::string type;
    std::deque<yarp::sig::Vector> in,out;

    struct SpatialCompetence
    {
        yarp::sig::Matrix A;
        yarp::sig::Vector c;
        double scale;
        bool extrapolation;
        yarp::sig::Matrix R;
        yarp::sig::Vector radii;
        SpatialCompetence() : extrapolation(true), scale(1.0)
        {
            A=yarp::math::eye(3,3);
            c=yarp::sig::Vector(3,0.0);
            R=yarp::math::eye(4,4);
            radii=yarp::sig::Vector(3,1.0);
        }
    } spatialCompetence;

    virtual bool computeSpatialTransformation();
    virtual bool computeSpatialCompetence(const std::deque<yarp::sig::Vector> &points);
    void copySuperClassData(const Calibrator &src);

public:
    virtual std::string getType() const { return type; }
    virtual bool getExtrapolation() const { return spatialCompetence.extrapolation; }
    virtual void setExtrapolation(const bool extrapolation) { spatialCompetence.extrapolation=extrapolation; }
    virtual bool addPoints(const yarp::sig::Vector &in, const yarp::sig::Vector &out)=0;
    virtual bool clearPoints()=0;
    virtual bool getPoints(std::deque<yarp::sig::Vector> &in, std::deque<yarp::sig::Vector> &out) const=0;
    virtual size_t getNumPoints() const { return in.size(); }
    virtual bool calibrate(double &error)=0;
    virtual bool retrieve(const yarp::sig::Vector &in, yarp::sig::Vector &out)=0;
    virtual double getSpatialCompetence(const yarp::sig::Vector &point);
    virtual bool toProperty(yarp::os::Property &info) const;
    virtual bool fromProperty(const yarp::os::Property &info);
    virtual ~Calibrator() { }
};


/************************************************************************/
class MatrixCalibrator : public Calibrator
{
protected:
    iCub::optimization::MatrixTransformationWithMatchedPoints *impl;
    yarp::sig::Matrix H;
    double scale;

public:
    MatrixCalibrator(const std::string &type="se3");
    MatrixCalibrator(const MatrixCalibrator &calibrator);
    virtual bool addPoints(const yarp::sig::Vector &in, const yarp::sig::Vector &out);
    virtual bool clearPoints();
    virtual bool getPoints(std::deque<yarp::sig::Vector> &in, std::deque<yarp::sig::Vector> &out) const;
    virtual bool calibrate(double &error);
    virtual bool retrieve(const yarp::sig::Vector &in, yarp::sig::Vector &out);
    virtual bool toProperty(yarp::os::Property &info) const;
    virtual bool fromProperty(const yarp::os::Property &info);
    virtual ~MatrixCalibrator();
};


/************************************************************************/
class LSSVMCalibrator : public Calibrator
{
protected:
    iCub::learningmachine::IMachineLearner *impl;

public:
    LSSVMCalibrator(const std::string &type="lssvm");
    LSSVMCalibrator(const LSSVMCalibrator &calibrator);
    virtual bool addPoints(const yarp::sig::Vector &in, const yarp::sig::Vector &out);
    virtual bool clearPoints();
    virtual bool getPoints(std::deque<yarp::sig::Vector> &in, std::deque<yarp::sig::Vector> &out) const;
    virtual bool calibrate(double &error);
    virtual bool retrieve(const yarp::sig::Vector &in, yarp::sig::Vector &out);
    virtual bool toProperty(yarp::os::Property &info) const;
    virtual bool fromProperty(const yarp::os::Property &info);
    virtual ~LSSVMCalibrator();
};


/************************************************************************/
class LocallyWeightedExperts
{
protected:
    std::deque<Calibrator*> models;

public:
    virtual size_t size() const { return models.size(); }
    virtual Calibrator *operator[](const size_t i);
    virtual LocallyWeightedExperts &operator<<(Calibrator &c);
    virtual bool retrieve(const yarp::sig::Vector &in, yarp::sig::Vector &out);
    virtual void clear();
    virtual ~LocallyWeightedExperts();
};


#endif



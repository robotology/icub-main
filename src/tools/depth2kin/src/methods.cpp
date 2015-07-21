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

#include <cmath>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/math.h>
#include <iCub/optimization/algorithms.h>
#include <iCub/optimization/affinity.h>
#include <iCub/optimization/calibReference.h>
#include <iCub/learningMachine/LSSVMLearner.h>

#include "methods.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::optimization;
using namespace iCub::learningmachine;


/************************************************************************/
bool Calibrator::computeSpatialTransformation()
{
    Matrix U,V; Vector S;
    SVD(spatialCompetence.A,U,S,V);

    spatialCompetence.radii.resize(3);
    spatialCompetence.radii[0]=(S[0]!=0.0)?(1.0/sqrt(fabs(S[0]))):0.0;
    spatialCompetence.radii[1]=(S[1]!=0.0)?(1.0/sqrt(fabs(S[1]))):0.0;
    spatialCompetence.radii[2]=(S[2]!=0.0)?(1.0/sqrt(fabs(S[2]))):0.0;

    // check if we have a right-hand basis
    Vector x=V.getCol(0);
    Vector y=V.getCol(1);
    Vector z=V.getCol(2);
    if (dot(cross(x,y),z)<0.0)
        V.setCol(2,-1.0*z);

    Matrix R(4,4); R.zero(); R(3,3)=1.0;
    R.setSubmatrix(V,0,0);
    spatialCompetence.R=SE3inv(R);

    return true;
}


/************************************************************************/
bool Calibrator::computeSpatialCompetence(const deque<Vector> &points)
{
    if (minVolumeEllipsoid(points,0.001,spatialCompetence.A,spatialCompetence.c))
        if (computeSpatialTransformation())
            return true;

    return false;
}


/************************************************************************/
void Calibrator::copySuperClassData(const Calibrator &src)
{
    type=src.type;
    in=src.in;
    out=src.out;
    spatialCompetence=src.spatialCompetence;
}


/************************************************************************/
double Calibrator::getSpatialCompetence(const Vector &point)
{
    if (point.length()<3)
        return 0.0;

    Vector x=(point.subVector(0,2)-spatialCompetence.c)/spatialCompetence.scale;
    if (dot(x,spatialCompetence.A*x)<=1.0)
        return 1.0;
    else if (spatialCompetence.extrapolation)
    {
        // getting cartesian coordinates wrt (A,c) frame
        x.push_back(1.0);
        x=spatialCompetence.R*x;
        x.pop_back();

        // distance approximated as ||x-xp||,
        // where xp is the projection of x over
        // the ellipsoid along the line connecting
        // x with the origin

        // switch to spherical coordinates
        double cos_theta=x[2]/norm(x);
        double theta=acos(cos_theta);
        double phi=atan2(x[1],x[0]);

        Vector xp(3);
        double cos_phi=cos(phi);
        xp[0]=spatialCompetence.radii[0]*cos_theta*cos_phi;
        xp[1]=spatialCompetence.radii[1]*sin(theta)*cos_phi;
        xp[2]=spatialCompetence.radii[2]*sin(phi);

        double d=norm(x-xp);
        return exp(-40.0*d*d);  // competence(0.2 [m])=0.2
    }
    else
        return 0.0;
}


/************************************************************************/
bool Calibrator::toProperty(Property &info) const
{
    Bottle data;
    Bottle &values=data.addList();
    values.addString(spatialCompetence.extrapolation?"true":"false");
    values.addDouble(spatialCompetence.scale);
    values.addDouble(spatialCompetence.c[0]);
    values.addDouble(spatialCompetence.c[1]);
    values.addDouble(spatialCompetence.c[2]);
    for (int r=0; r<spatialCompetence.A.rows(); r++)
        for (int c=0; c<spatialCompetence.A.cols(); c++)
            values.addDouble(spatialCompetence.A(r,c));

    info.put("spatial_competence",data.get(0));
    return true;
}


/************************************************************************/
bool Calibrator::fromProperty(const Property &info)
{
    if (!info.check("spatial_competence"))
        return false;

    spatialCompetence.A=eye(3,3);
    spatialCompetence.c=Vector(3,0.0);
    spatialCompetence.scale=1.0;
    spatialCompetence.extrapolation=true;
    if (Bottle *values=info.find("spatial_competence").asList())
    {
        spatialCompetence.extrapolation=(values->get(0).asString()=="true");
        spatialCompetence.scale=values->get(1).asDouble();
        spatialCompetence.c[0]=values->get(2).asDouble();
        spatialCompetence.c[1]=values->get(3).asDouble();
        spatialCompetence.c[2]=values->get(4).asDouble();
        int r=0; int c=0;
        for (int i=5; i<values->size(); i++)
        {
            spatialCompetence.A(r,c)=values->get(i).asDouble();
            if (++c>=spatialCompetence.A.cols())
            {
                c=0;
                if (++r>=spatialCompetence.A.rows())
                    break;
            }
        }        
    }

    computeSpatialTransformation();
    return true;
}


/************************************************************************/
MatrixCalibrator::MatrixCalibrator(const string &type)
{
    H=eye(4,4);
    scale=1.0;
    if (type=="affine")
    {
        impl=new AffinityWithMatchedPoints;
        this->type="affine";
    }
    else
    {
        impl=new CalibReferenceWithMatchedPoints;
        this->type=type;
        if ((this->type!="se3") && (this->type!="se3+scale"))
            this->type="se3";
    }
}


/************************************************************************/
MatrixCalibrator::MatrixCalibrator(const MatrixCalibrator &calibrator)
{
    copySuperClassData(calibrator);
    H=calibrator.H;
    scale=calibrator.scale;
    if (type=="affine")
        impl=new AffinityWithMatchedPoints(*dynamic_cast<AffinityWithMatchedPoints*>(calibrator.impl));
    else
        impl=new CalibReferenceWithMatchedPoints(*dynamic_cast<CalibReferenceWithMatchedPoints*>(calibrator.impl));
}


/************************************************************************/
MatrixCalibrator::~MatrixCalibrator()
{
    delete impl;
}


/************************************************************************/
bool MatrixCalibrator::addPoints(const Vector &in, const Vector &out)
{
    if ((in.length()>=3) && (out.length()>=3))
    {
        Vector _in=in.subVector(0,2);
        Vector _out=out.subVector(0,2);

        this->in.push_back(_in);
        this->out.push_back(_out);
        return impl->addPoints(_in,_out);
    }
    else
        return false;
}


/************************************************************************/
bool MatrixCalibrator::clearPoints()
{
    impl->clearPoints();
    in.clear();
    out.clear();    
    return true;
}


/************************************************************************/
bool MatrixCalibrator::getPoints(deque<Vector> &in, deque<Vector> &out) const
{
    in=this->in;
    out=this->out;
    return true; 
}


/************************************************************************/
bool MatrixCalibrator::calibrate(double &error)
{
    (type=="se3+scale")?dynamic_cast<CalibReferenceWithMatchedPoints*>(impl)->calibrate(H,scale,error):
                        impl->calibrate(H,error);

    if (computeSpatialCompetence(in))
    {
        spatialCompetence.scale=1.2;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool MatrixCalibrator::retrieve(const Vector &in, Vector &out)
{
    if (in.length()>=3)
    {
        Vector _in=in.subVector(0,2);
        _in.push_back(1.0);
        out=H*_in;
        out.pop_back();
        out*=scale;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool MatrixCalibrator::toProperty(Property &info) const
{
    Bottle data;
    Bottle &values=data.addList();
    values.addDouble(scale);
    for (int r=0; r<H.rows(); r++)
        for (int c=0; c<H.cols(); c++)
            values.addDouble(H(r,c));

    info.clear();
    info.put("type",type.c_str());
    info.put("calibration_data",data.get(0));
    return Calibrator::toProperty(info);
}


/************************************************************************/
bool MatrixCalibrator::fromProperty(const Property &info)
{
    if (!info.check("type"))
        return false;

    string type=info.find("type").asString().c_str();
    if ((type=="se3") || (type=="se3+scale"))
    {
        delete impl;
        impl=new CalibReferenceWithMatchedPoints;
    }
    else if (type=="affine")
    {
        delete impl;
        impl=new AffinityWithMatchedPoints;
    }
    else
        return false;

    H=eye(4,4);
    scale=1.0;
    if (Bottle *values=info.find("calibration_data").asList())
    {
        scale=values->get(0).asDouble();

        int r=0; int c=0;        
        for (int i=1; i<values->size(); i++)
        {
            H(r,c)=values->get(i).asDouble();
            if (++c>=H.cols())
            {
                c=0;
                if (++r>=H.rows())
                    break;
            }
        }
    }

    return Calibrator::fromProperty(info);
}


/************************************************************************/
LSSVMCalibrator::LSSVMCalibrator(const string &type)
{
    impl=new LSSVMLearner;
    LSSVMLearner *lssvm=dynamic_cast<LSSVMLearner*>(impl);
    lssvm->setDomainSize(3);
    lssvm->setCoDomainSize(3);
    lssvm->setC(100.0);
    lssvm->getKernel()->setGamma(10.0);

    this->type="lssvm";
}


/************************************************************************/
LSSVMCalibrator::LSSVMCalibrator(const LSSVMCalibrator &calibrator)
{
    copySuperClassData(calibrator);
    impl=new LSSVMLearner(*dynamic_cast<LSSVMLearner*>(calibrator.impl));
}


/************************************************************************/
bool LSSVMCalibrator::addPoints(const Vector &in, const Vector &out)
{
    if ((in.length()>=3) && (out.length()>=3))
    {
        Vector _in=in.subVector(0,2);
        Vector _out=out.subVector(0,2);
        impl->feedSample(in,out);

        this->in.push_back(_in);
        this->out.push_back(_out);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool LSSVMCalibrator::clearPoints()
{
    impl->reset();
    in.clear();
    out.clear();
    return true;
}


/************************************************************************/
bool LSSVMCalibrator::getPoints(deque<Vector> &in, deque<Vector> &out) const
{
    in=this->in;
    out=this->out;
    return true;
}


/************************************************************************/
bool LSSVMCalibrator::calibrate(double &error)
{
    if (getNumPoints()==0)
        return false;

    impl->train();

    error=0.0;
    for (size_t i=0; i<getNumPoints(); i++)
    {
        Vector p;
        retrieve(in[i],p);
        error+=norm(out[i].subVector(0,2)-p);
    }
    error/=getNumPoints();

    if (computeSpatialCompetence(in))
    {
        spatialCompetence.scale=1.0;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool LSSVMCalibrator::retrieve(const Vector &in, Vector &out)
{
    if (in.length()>=3)
    {
        Prediction prediction=impl->predict(in.subVector(0,2));
        out=prediction.getPrediction();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool LSSVMCalibrator::toProperty(Property &info) const
{
    Bottle data;
    Bottle &values=data.addList();
    values.addString(impl->toString().c_str());

    info.clear();
    info.put("type",type.c_str());
    info.put("calibration_data",data.get(0));
    return Calibrator::toProperty(info);
}


/************************************************************************/
bool LSSVMCalibrator::fromProperty(const Property &info)
{
    if (!info.check("type"))
        return false;

    string type=info.find("type").asString().c_str();
    if (type=="lssvm")
        clearPoints();
    else
        return false;

    bool ret=false;
    if (Bottle *values=info.find("calibration_data").asList())
        if (values->size()>=1)
            ret=impl->fromString(values->toString().c_str());

    ret&=Calibrator::fromProperty(info); 
    return ret;
}


/************************************************************************/
LSSVMCalibrator::~LSSVMCalibrator()
{
    delete impl;
    in.clear();
    out.clear();
}


/************************************************************************/
Calibrator *LocallyWeightedExperts::operator[](const size_t i)
{
    if (i<models.size())
        return models[i];
    else
        return NULL;
}


/************************************************************************/
LocallyWeightedExperts &LocallyWeightedExperts::operator<<(Calibrator &c)
{
    Calibrator *calibrator;
    if (c.getType()=="lssvm")
        calibrator=new LSSVMCalibrator(dynamic_cast<LSSVMCalibrator&>(c));
    else
        calibrator=new MatrixCalibrator(dynamic_cast<MatrixCalibrator&>(c));

    models.push_back(calibrator);
    return *this;
}


/************************************************************************/
bool LocallyWeightedExperts::retrieve(const Vector &in, Vector &out)
{
    if (in.length()<3)
        return false;

    Vector _in=in.subVector(0,2);
    Vector outExperts(_in.length(),0.0);
    Vector outExtrapolators(_in.length(),0.0);
    double sumExperts=0.0;
    double sumExtrapolators=0.0;

    // collect information over available models
    for (size_t i=0; i<models.size(); i++)
    {
        double competence=models[i]->getSpatialCompetence(_in);
        if (competence>0.0)
        {
            Vector pred;
            models[i]->retrieve(_in,pred);

            outExtrapolators+=competence*pred;
            sumExtrapolators+=competence;
            if (competence>=1.0)
            {                
                outExperts+=competence*pred;
                sumExperts+=competence;
            }
        }
    }

    // consider experts first
    if (sumExperts!=0.0)
    {
        out=outExperts/sumExperts;
        return true;
    }
    // then extrapolators
    else if (sumExtrapolators!=0.0)
    {
        out=outExtrapolators/sumExtrapolators;
        return true;
    }
    // no models found
    else
        return false; 
}


/************************************************************************/
void LocallyWeightedExperts::clear()
{
    for (size_t i=0; i<models.size(); i++)
        delete models[i];

    models.clear();
}


/************************************************************************/
LocallyWeightedExperts::~LocallyWeightedExperts()
{
    clear();
}



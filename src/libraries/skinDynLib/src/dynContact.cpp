/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include "stdio.h"

#include <yarp/math/Math.h>
#include "iCub/skinDynLib/dynContact.h"
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;


//~~~~~~~~~~~~~~~~~~~~~~
//   DYN CONTACT
//~~~~~~~~~~~~~~~~~~~~~~
unsigned long dynContact::ID = 1;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(){
    init(UNKNOWN_BODY_PART, 0, zeros(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, const unsigned int &_linkNumber, const Vector &_CoP){
    init(_bodyPart, _linkNumber, _CoP);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, const unsigned int &_linkNumber, const Vector &_CoP, const Vector &_Mu){
    init(_bodyPart, _linkNumber, _CoP, _Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, const unsigned int &_linkNumber, const Vector &_CoP, const Vector &_Mu, const Vector &_Fdir){
    init(_bodyPart, _linkNumber, _CoP, _Mu, _Fdir);    
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::init(const BodyPart &_bodyPart, const unsigned int &_linkNumber, const Vector &_CoP, const Vector &_Mu, const Vector &_Fdir){
    contactId = ID++;
    setBodyPart(_bodyPart);
    setLinkNumber(_linkNumber); 
    setCoP(_CoP);
    Mu.resize(3, 0.0);
    Fdir.resize(3, 0.0);
    F.resize(3, 0.0);
    Fmodule = 0.0;

    if(_Mu.size()==0)
        muKnown = false;
    else
        fixMoment(_Mu);

    if(_Fdir.size()==0)
        fDirKnown = false;
    else
        fixForceDirection(_Fdir);
}
//~~~~~~~~~~~~~~~~~~~~~~
//   GET methods
//~~~~~~~~~~~~~~~~~~~~~~
Vector dynContact::getForceMoment() const{ return cat(F, Mu); }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getForce() const{ return F;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getForceDirection() const{ return Fdir;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const double& dynContact::getForceModule() const{ return Fmodule;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getMoment() const{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getCoP() const{ return CoP;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const unsigned int& dynContact::getLinkNumber() const{ return linkNumber;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const BodyPart& dynContact::getBodyPart() const{ return bodyPart;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string dynContact::getBodyPartName() const{ return BodyPart_s[bodyPart];}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const unsigned long& dynContact::getId() const{ return contactId;}

//~~~~~~~~~~~~~~~~~~~~~~
//   IS methods
//~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::isMomentKnown() const{ return muKnown;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::isForceDirectionKnown() const{ return fDirKnown;}
//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~    
bool dynContact::setForce(const Vector &_F){
    if(!checkVectorDim(_F, 3, "force"))
        return false;
    F = _F;
    Fmodule = norm(_F);
    if(Fmodule!=0.0)
        Fdir = _F / Fmodule;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setForceModule(const double &_Fmodule){
    if(_Fmodule<0){
        if(verbose)
            fprintf(stderr, "Error in dynContact: negative force module, %f\n", _Fmodule);
        return false;
    }
    Fmodule = _Fmodule;
    F=Fmodule*Fdir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setForceDirection(const Vector &_Fdir){
    if(!checkVectorDim(_Fdir, 3, "force direction"))
        return false;
    double FdirNorm = norm(_Fdir);
    if(FdirNorm != 0.0)
        Fdir = _Fdir / FdirNorm;
    F=Fmodule*Fdir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setMoment(const Vector &_Mu){
    if(!checkVectorDim(_Mu, 3, "moment"))
        return false;
    Mu = _Mu;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setForceMoment(const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu){
    return setForce(_F) && setMoment(_Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setForceMoment(const yarp::sig::Vector &_FMu){
    if(!checkVectorDim(_FMu, 6, "force moment"))
        return false;
    bool res = setForce(_FMu.subVector(0,2));
    return res && setMoment(_FMu.subVector(3,5));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::setCoP(const Vector &_CoP){
    if(!checkVectorDim(_CoP, 3, "Center of pressure"))
        return false;
    CoP = _CoP;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setLinkNumber(const unsigned int &_linkNum){
    linkNumber = _linkNum;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setBodyPart(const BodyPart &_bodyPart){
    bodyPart = _bodyPart;
}
//~~~~~~~~~~~~~~~~~~~~~~
//   FIX/UNFIX methods
//~~~~~~~~~~~~~~~~~~~~~~ 
bool dynContact::fixForceDirection(const Vector &_Fdir){
    if(setForceDirection(_Fdir)){
        fDirKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::fixMoment(){
    return fixMoment(zeros(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::fixMoment(const Vector &_Mu){
    if(setMoment(_Mu)){
        muKnown = true;
        return true;
    }
    return false;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::unfixForceDirection(){ fDirKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::unfixMoment(){ muKnown=false;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::write(ConnectionWriter& connection){
    //connection.declareSizes();

    connection.appendInt(bodyPart);    // left_arm, right_arm, ...
    connection.appendInt(linkNumber);
    for(int i=0;i<3;i++) connection.appendDouble(CoP[i]);
    connection.appendDouble(Fmodule);
    for(int i=0;i<3;i++) connection.appendDouble(Fdir[i]);
    for(int i=0;i<3;i++) connection.appendDouble(Mu[i]);  

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();  
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::read(ConnectionReader& connection){
    // auto-convert text mode interaction
    connection.convertTextMode();
    // populate the object
    bodyPart    = (BodyPart)connection.expectInt();
    linkNumber  = connection.expectInt();
    for(int i=0;i<3;i++) CoP[i] = connection.expectDouble();
    Fmodule = connection.expectDouble();
    for(int i=0;i<3;i++) Fdir[i] = connection.expectDouble();
    for(int i=0;i<3;i++) Mu[i] = connection.expectDouble();
     
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string dynContact::toString(const int &precision) const{
    stringstream res;
    res<< "Contact id: "<< contactId<< "Body part: "<< BodyPart_s[bodyPart]<< ", link: "<< linkNumber<< ", CoP: "<< 
        CoP.toString(precision)<< ", F: "<< getForce().toString(precision)<< ", M: "<< Mu.toString(precision);
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setVerbose(const unsigned int &verb){
    verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::checkVectorDim(const Vector &v, const unsigned int &dim, const string &descr){
    if(v.length() != dim){
        if(verbose)
            fprintf(stderr, "Error in dynContact: unexpected dimension of vector %s, %d\n", descr.c_str(), (int)v.length());
        return false;
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



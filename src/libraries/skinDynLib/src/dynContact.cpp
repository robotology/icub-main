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
#include <yarp/os/LogStream.h>
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
    init(BODY_PART_UNKNOWN, 0, zeros(3));
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const Vector &_CoP){
    init(_bodyPart, _linkNumber, _CoP);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const Vector &_CoP, const Vector &_Mu){
    init(_bodyPart, _linkNumber, _CoP, _Mu);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
dynContact::dynContact(const BodyPart &_bodyPart, unsigned int _linkNumber, const Vector &_CoP, const Vector &_Mu, const Vector &_Fdir){
    init(_bodyPart, _linkNumber, _CoP, _Mu, _Fdir);    
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::init(const BodyPart &_bodyPart, unsigned int _linkNumber, const Vector &_CoP, const Vector &_Mu, const Vector &_Fdir){
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

    linkName = "";
    frameName = "";
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
double dynContact::getForceModule() const{ return Fmodule;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getMoment() const{ return Mu;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
const Vector& dynContact::getCoP() const{ return CoP;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned int dynContact::getLinkNumber() const{ return linkNumber;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BodyPart dynContact::getBodyPart() const{ return bodyPart;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string dynContact::getBodyPartName() const{ return BodyPart_s[bodyPart];}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
unsigned long dynContact::getId() const{ return contactId;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string dynContact::getLinkName() const{ return linkName;}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
std::string dynContact::getFrameName() const{ return frameName;}

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
bool dynContact::setForceModule(double _Fmodule){
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
void dynContact::setLinkNumber(unsigned int _linkNum){
    linkNumber = _linkNum;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setBodyPart(BodyPart _bodyPart){
    bodyPart = _bodyPart;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setLinkName(const std::string &_linkName){
    linkName = _linkName;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setFrameName(const std::string &_frameName){
    frameName = _frameName;
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
    // represent a dynContact as a list of 5 elements that are:
    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 2 string, i.e. linkName, frameName

    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(5);
    // list of 3 int, i.e. contactId, bodyPart, linkNumber
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_INT);
    connection.appendInt(3);
    connection.appendInt(contactId);
    connection.appendInt(bodyPart);    // left_arm, right_arm, ...
    connection.appendInt(linkNumber);
    // list of 3 double, i.e. the CoP
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(CoP[i]);
    // list of 3 double, i.e. the force
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(F[i]);
    // list of 3 double, i.e. the moment
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_DOUBLE);
    connection.appendInt(3);
    for(int i=0;i<3;i++) connection.appendDouble(Mu[i]);
    // list of 2 string, i.e. linkName, frameName
    connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_STRING);
    connection.appendInt(2);
    connection.appendInt(linkName.length());
    connection.appendExternalBlock((char*)linkName.c_str(), linkName.length());
    connection.appendInt(frameName.length());
    connection.appendExternalBlock((char*)frameName.c_str(), frameName.length());

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();  
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::read(ConnectionReader& connection){
    // auto-convert text mode interaction
    connection.convertTextMode();

    // represent a dynContact as a list of 5 elements that are:
    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 2 string, i.e. linkName, frameName
    if(connection.expectInt()!= BOTTLE_TAG_LIST)
        return false;

    int listSize = connection.expectInt();
    if(listSize != 5)
    {
        yError() << "skinDynLib: error reading skinContact, expecting a list of 10 elements, but "
                 << listSize << " found. You are probably using two different versions of icub-main across the network. "
                 << "See https://github.com/robotology/icub-main/pull/462 for more info.";
        return false;
    }

    // - a list of 3 int, i.e. contactId, bodyPart, linkNumber
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT || connection.expectInt()!=3)
        return false;
    contactId   = connection.expectInt();
    bodyPart    = (BodyPart)connection.expectInt();
    linkNumber  = connection.expectInt();
    // - a list of 3 double, i.e. the CoP
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) CoP[i] = connection.expectDouble();
    // - a list of 3 double, i.e. the force
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) F[i] = connection.expectDouble();
    setForce(F);
    // - a list of 3 double, i.e. the moment
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_DOUBLE || connection.expectInt()!=3)
        return false;
    for(int i=0;i<3;i++) Mu[i] = connection.expectDouble();
    // - a list of 2 string, i.e. linkName, frameName
    if(connection.expectInt()!=BOTTLE_TAG_LIST+BOTTLE_TAG_STRING || connection.expectInt()!=2)
        return false;
    int linkNameLen = connection.expectInt();
    linkName.resize(linkNameLen);
    if (!connection.expectBlock((char*)linkName.c_str(), linkNameLen))
        return false;
    int frameNameLen = connection.expectInt();
    frameName.resize(frameNameLen);
    if (!connection.expectBlock((char*)frameName.c_str(), frameNameLen))
        return false;

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string dynContact::toString(int precision) const{
    stringstream res;
    res<< "Contact id: "<< contactId<< ", Link name: "<< linkName << ", Frame name: " << frameName <<
          ", Body part: "<< BodyPart_s[bodyPart]<< ", Link number: "<< linkNumber<< ", CoP: "<<
        CoP.toString(precision)<< ", F: "<< F.toString(precision)<< ", M: "<< Mu.toString(precision);
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void dynContact::setVerbose(unsigned int verb){
    verbose = verb;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool dynContact::checkVectorDim(const Vector &v, unsigned int dim, const string &descr){
    if(v.length() != dim){
        if(verbose)
            fprintf(stderr, "Error in dynContact: unexpected dimension of vector %s, %d\n", descr.c_str(), (int)v.length());
        return false;
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <iomanip>
#include "iCub/skinDynLib/skinContact.h"

using namespace iCub::skinDynLib;
using namespace yarp::sig;
using namespace yarp::os;

//~~~~~~~~~~~~~~~~~~~~~~
//   CONSTRUCTORS
//~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(BodyPart _bodyPart, SkinPart _skinPart, unsigned int _linkNumber, const Vector &_CoP)
:dynContact(_bodyPart, _linkNumber, _CoP){
    skinPart = _skinPart;
    pressure = 0.0;
    geoCenter.resize(3); geoCenter.zero();
    activeTaxels = 1;
}

skinContact::skinContact(const Bottle& b){
    fromBottle(b);
}

//~~~~~~~~~~~~~~~~~~~~~~
//   BOTTLE methods
//~~~~~~~~~~~~~~~~~~~~~~
yarp::os::Bottle skinContact::toBottle() const{
    Bottle b;
    b.addInt(bodyPart);    // left_arm, right_arm, ...
    b.addInt(skinPart);    // left_forearm_upper, left_forearm_lower, ...
    b.addInt(linkNumber);
    b.addDouble(CoP[0]);
    b.addDouble(CoP[1]);
    b.addDouble(CoP[2]);

    // all this data are useless at the moment, but may be used in the future
    /*b.addDouble(pressure);
    b.addDouble(geoCenter[0]);
    b.addDouble(geoCenter[1]);
    b.addDouble(geoCenter[2]);
    b.addInt(activeTaxels);
    // also the force and the moment may be inserted in the Bottle
    */
    
    return b;
}

bool skinContact::fromBottle(const Bottle& b){
    // check the Bottle data
    if(b.isNull() || b.size() != 6)
        return false;
    if(!b.get(0).isInt() || !b.get(1).isInt() || !b.get(2).isInt() ||
        !b.get(3).isDouble() || !b.get(4).isDouble() || !b.get(4).isDouble())
        return false;
    if(b.get(0).asInt() >= BODY_PART_SIZE)
        return false;
    if(b.get(1).asInt() >= SKIN_PART_SIZE)
        return false;
    if(b.get(2).asInt() < 0)
        return false;

    // populate the object
    bodyPart    = (BodyPart)b.get(0).asInt();
    skinPart    = (SkinPart)b.get(1).asInt();
    linkNumber  = b.get(2).asInt();
    CoP[0]      = b.get(3).asDouble();
    CoP[1]      = b.get(4).asDouble();
    CoP[2]      = b.get(5).asDouble();

    return true;
}


//~~~~~~~~~~~~~~~~~~~~~~
//   GET methods
//~~~~~~~~~~~~~~~~~~~~~~
yarp::sig::Vector skinContact::getGeoCenter() const{ return geoCenter; }

double skinContact::getPressure() const{ return pressure; }

unsigned int skinContact::getActiveTaxels() const{ return activeTaxels; }

SkinPart skinContact::getSkinPart() const{ return skinPart; }

std::string skinContact::getSkinPartName() const{ return SkinPart_s[skinPart]; }


//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~    
bool skinContact::setGeoCenter(const yarp::sig::Vector &_geoCenter){
    if(!checkVectorDim(_geoCenter, 3, "Geometric center"))
        return false;
    geoCenter = _geoCenter;
    return true;
}

bool skinContact::setPressure(double _pressure){
    if(pressure<=0)
        return false;
    pressure = _pressure;
    return true;
}

bool skinContact::setActiveTaxels(unsigned int _activeTaxels){
    if(_activeTaxels==0)
        return false;
    activeTaxels = _activeTaxels;
    return true;
}

void skinContact::setSkinPart(SkinPart _skinPart){
    skinPart = _skinPart;
}

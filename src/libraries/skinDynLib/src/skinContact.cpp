/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include "iCub/skinDynLib/skinContact.h"

using namespace iCub::skinDynLib;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

//~~~~~~~~~~~~~~~~~~~~~~
//   CONSTRUCTORS
//~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(BodyPart _bodyPart, SkinPart _skinPart, unsigned int _linkNumber, const Vector &_CoP, 
                         const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), activeTaxels(_activeTaxels), pressure(_pressure){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact()
: skinPart(UNKNOWN_SKIN_PART), pressure(0.0), activeTaxels(0) {
    geoCenter.resize(3); 
    geoCenter.zero();
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
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setPressure(double _pressure){
    if(pressure<=0)
        return false;
    pressure = _pressure;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setActiveTaxels(unsigned int _activeTaxels){
    if(_activeTaxels==0)
        return false;
    activeTaxels = _activeTaxels;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setSkinPart(SkinPart _skinPart){
    skinPart = _skinPart;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::write(ConnectionWriter& connection){
    dynContact::write(connection);

    connection.appendInt(skinPart);
    for(int i=0;i<3;i++) connection.appendDouble(geoCenter[i]);
    connection.appendInt(activeTaxels);
    connection.appendDouble(pressure);
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::read(ConnectionReader& connection){
    dynContact::read(connection);

    skinPart                            = (SkinPart) connection.expectInt();
    for(int i=0;i<3;i++) geoCenter[i]   = connection.expectDouble();
    activeTaxels                        = connection.expectInt();
    pressure                            = connection.expectDouble();
     
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string skinContact::toString() const{
    stringstream res;
    res<< dynContact::toString()<< ", Skin part: "<< SkinPart_s[skinPart]<< ", geometric center: "<< 
        geoCenter.toString().c_str()<< ", active taxels: "<< activeTaxels<< ", pressure: "<< pressure;
    //res<< "Skin part: "<< SkinPart_s[skinPart]<< ", geometric center: "<< geoCenter.toString().c_str()<< ", active taxels: "<< activeTaxels;
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
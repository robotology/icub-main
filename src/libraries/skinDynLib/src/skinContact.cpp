/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <yarp/math/Math.h>
#include "iCub/skinDynLib/skinContact.h"

using namespace iCub::skinDynLib;
using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

//~~~~~~~~~~~~~~~~~~~~~~
//   CONSTRUCTORS
//~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const dynContact &c)
    :dynContact(c), skinPart(UNKNOWN_SKIN_PART), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), activeTaxels(_activeTaxels), taxelList(vector<unsigned int>(activeTaxels, 0)), pressure(_pressure), normalDir(zeros(3)){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, unsigned int _activeTaxels, double _pressure, const Vector &_normalDir)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), activeTaxels(_activeTaxels), taxelList(vector<unsigned int>(activeTaxels, 0)), pressure(_pressure), normalDir(_normalDir){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, vector<unsigned int> _taxelList, double _pressure)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), taxelList(_taxelList), activeTaxels(_taxelList.size()), pressure(_pressure), normalDir(zeros(3)){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, vector<unsigned int> _taxelList, double _pressure, const Vector &_normalDir)
:dynContact(_bodyPart, _linkNumber, _CoP), skinPart(_skinPart), 
geoCenter(_geoCenter), taxelList(_taxelList), activeTaxels(_taxelList.size()), pressure(_pressure), normalDir(_normalDir){}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact()
: dynContact(), skinPart(UNKNOWN_SKIN_PART), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
//~~~~~~~~~~~~~~~~~~~~~~
//   SET methods
//~~~~~~~~~~~~~~~~~~~~~~    
bool skinContact::setGeoCenter(const Vector &_geoCenter){
    if(!checkVectorDim(_geoCenter, 3, "Geometric center"))
        return false;
    geoCenter = _geoCenter;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setNormalDir(const Vector &_normalDir){
    if(!checkVectorDim(_normalDir, 3, "Normal direction"))
        return false;
    normalDir = _normalDir;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::setPressure(double _pressure){
    if(pressure<=0.0)
        return false;
    pressure = _pressure;
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setActiveTaxels(unsigned int _activeTaxels){
    activeTaxels = _activeTaxels;
    taxelList.resize(activeTaxels);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setSkinPart(SkinPart _skinPart){
    skinPart = _skinPart;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void skinContact::setTaxelList(const vector<unsigned int> &list){
    taxelList = list;
    activeTaxels = list.size();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//   SERIALIZATION methods
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::write(ConnectionWriter& connection){
    dynContact::write(connection);

    connection.appendInt(skinPart);
    for(int i=0;i<3;i++) connection.appendDouble(geoCenter[i]);
    for(int i=0;i<3;i++) connection.appendDouble(normalDir[i]);
    connection.appendInt(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) connection.appendInt(taxelList[i]);
    connection.appendDouble(pressure);
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::read(ConnectionReader& connection){
    dynContact::read(connection);

    skinPart                            = (SkinPart) connection.expectInt();
    for(int i=0;i<3;i++) geoCenter[i]   = connection.expectDouble();
    for(int i=0;i<3;i++) normalDir[i]   = connection.expectDouble();
    activeTaxels                        = connection.expectInt();
    taxelList.resize(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) taxelList[i] = connection.expectInt();
    pressure                            = connection.expectDouble();
     
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
string skinContact::toString(int precision) const{
    stringstream res;
    res<< dynContact::toString(precision)<< ", Skin part: "<< SkinPart_s[skinPart]<< ", geometric center: "<< 
        geoCenter.toString(precision)<< ", normal direction: "<< normalDir.toString(precision)<< 
        ", active taxels: "<< activeTaxels<< ", pressure: "<< pressure;
    return res.str();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



/*
 * Copyright (C) 2010-2011 RobotCub Consortium
 * Author: Andrea Del Prete
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iostream>
#include <sstream>
#include <iomanip>
#include <yarp/os/ConnectionReader.h>
#include <yarp/os/ConnectionWriter.h>
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
    :dynContact(c), skinPart(SKIN_PART_UNKNOWN), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
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
 skinContact::skinContact(const BodyPart &_bodyPart, const SkinPart &_skinPart, unsigned int _linkNumber, const yarp::sig::Vector &_CoP, 
        const yarp::sig::Vector &_geoCenter, std::vector<unsigned int> _taxelList, double _pressure, const yarp::sig::Vector &_normalDir,
        const yarp::sig::Vector &_F, const yarp::sig::Vector &_Mu)
 :dynContact(_bodyPart,_linkNumber,_CoP,_Mu), skinPart(_skinPart),
 geoCenter(_geoCenter), taxelList(_taxelList), activeTaxels(_taxelList.size()), pressure(_pressure), normalDir(_normalDir){
   this->setForce(_F); //note that dynContact constructor sets Fmodule to 0; here setForce() overwrites the init with the proper force vector and sets 
   //also Fmodule and Fdir appropriately  
 }
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
skinContact::skinContact()
: dynContact(), skinPart(SKIN_PART_UNKNOWN), geoCenter(zeros(3)), pressure(0.0), activeTaxels(0), normalDir(zeros(3)) {}
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
    if(_pressure<0.0)
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
bool skinContact::write(ConnectionWriter& connection) const
{
    // represent a skinContact as a list of 8 elements that are:
    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 3 double, i.e. the geometric center
    // - a list of 3 double, i.e. the normal direction
    // - a list of N int, i.e. the active taxel ids
    // - a double, i.e. the pressure

    connection.appendInt32(BOTTLE_TAG_LIST);
    connection.appendInt32(8);
    // list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_INT32);
    connection.appendInt32(4);
    connection.appendInt32(contactId);
    connection.appendInt32(bodyPart);    // left_arm, right_arm, ...
    connection.appendInt32(linkNumber);
    connection.appendInt32(skinPart);
    // list of 3 double, i.e. the CoP
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64);
    connection.appendInt32(3);
    for(int i=0;i<3;i++) connection.appendFloat64(CoP[i]);
    // list of 3 double, i.e. the force
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64);
    connection.appendInt32(3);
    for(int i=0;i<3;i++) connection.appendFloat64(F[i]);
    // list of 3 double, i.e. the moment
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64);
    connection.appendInt32(3);
    for(int i=0;i<3;i++) connection.appendFloat64(Mu[i]);
    // - a list of 3 double, i.e. the geometric center
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64);
    connection.appendInt32(3);
    for(int i=0;i<3;i++) connection.appendFloat64(geoCenter[i]);
    // - a list of 3 double, i.e. the normal direction
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_FLOAT64);
    connection.appendInt32(3);
    for(int i=0;i<3;i++) connection.appendFloat64(normalDir[i]);
    // - a list of N int, i.e. the active taxel ids
    connection.appendInt32(BOTTLE_TAG_LIST + BOTTLE_TAG_INT32);
    connection.appendInt32(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) connection.appendInt32(taxelList[i]);
    // - a double, i.e. the pressure
    connection.appendInt32(BOTTLE_TAG_FLOAT64);
    connection.appendFloat64(pressure);

    // if someone is foolish enough to connect in text mode,
    // let them see something readable.
    connection.convertTextMode();
    
    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::read(ConnectionReader& connection){
    // auto-convert text mode interaction
    connection.convertTextMode();

    // represent a skinContact as a list of 8 elements that are:
    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    // - a list of 3 double, i.e. the CoP
    // - a list of 3 double, i.e. the force
    // - a list of 3 double, i.e. the moment
    // - a list of 3 double, i.e. the geometric center
    // - a list of 3 double, i.e. the normal direction
    // - a list of N int, i.e. the active taxel ids
    // - a double, i.e. the pressure
    if(connection.expectInt32() != BOTTLE_TAG_LIST || connection.expectInt32() != 8)
        return false;

    // - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT32 || connection.expectInt32()!=4)
        return false;
    contactId   = connection.expectInt32();
    bodyPart    = (BodyPart)connection.expectInt32();
    linkNumber  = connection.expectInt32();
    skinPart    = (SkinPart) connection.expectInt32();

    // - a list of 3 double, i.e. the CoP
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_FLOAT64 || connection.expectInt32()!=3)
        return false;
    for(int i=0;i<3;i++) CoP[i] = connection.expectFloat64();

    // - a list of 3 double, i.e. the force
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_FLOAT64 || connection.expectInt32()!=3)
        return false;
    for(int i=0;i<3;i++) F[i] = connection.expectFloat64();
    setForce(F);

    // - a list of 3 double, i.e. the moment
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_FLOAT64 || connection.expectInt32()!=3)
        return false;
    for(int i=0;i<3;i++) Mu[i] = connection.expectFloat64();

    // - a list of 3 double, i.e. the geometric center
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_FLOAT64 || connection.expectInt32()!=3)
        return false;
    for(int i=0;i<3;i++) geoCenter[i]   = connection.expectFloat64();

    // - a list of 3 double, i.e. the normal direction
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_FLOAT64 || connection.expectInt32()!=3)
        return false;
    for(int i=0;i<3;i++) normalDir[i]   = connection.expectFloat64();

    // - a list of N int, i.e. the active taxel ids
    if(connection.expectInt32()!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT32)
        return false;
    activeTaxels                        = connection.expectInt32();
    taxelList.resize(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++) taxelList[i] = connection.expectInt32();

    // - a double, i.e. the pressure
    if(connection.expectInt32()!=BOTTLE_TAG_FLOAT64)
        return false;
    pressure                            = connection.expectFloat64();

    return !connection.isError();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector skinContact::toVector() const{
    Vector v(activeTaxels+21);
    unsigned int index = 0;
    v[index++] = contactId;
    v[index++] = bodyPart;
    v[index++] = linkNumber;
    v[index++] = skinPart;
    v.setSubvector(index, CoP);         index+=3;
    v.setSubvector(index, F);           index+=3;
    v.setSubvector(index, Mu);          index+=3;
    v.setSubvector(index, geoCenter);   index+=3;
    v.setSubvector(index, normalDir);   index+=3;
    v[index++] = activeTaxels;
    for(unsigned int i=0;i<activeTaxels;i++)
        v[index++] = taxelList[i];
    v[index++] = pressure;

    return v;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool skinContact::fromVector(const Vector &v){
    if(v.size()<21)
        return false;
    unsigned int index = 0;
    contactId   = (unsigned long)(v[index++]);
    bodyPart    = (BodyPart) int(v[index++]);
    linkNumber  = (unsigned int)(v[index++]);
    skinPart    = (SkinPart) int(v[index++]);
    CoP         = v.subVector(index, index+2);  index+=3;
    F           = v.subVector(index, index+2);  index+=3;
    Mu          = v.subVector(index, index+2);  index+=3;
    geoCenter   = v.subVector(index, index+2);  index+=3;
    normalDir   = v.subVector(index, index+2);  index+=3;
    activeTaxels = (unsigned int)(v[index++]);
    if(v.size()!=21+activeTaxels)
        return false;
    taxelList.resize(activeTaxels);
    for(unsigned int i=0;i<activeTaxels;i++)
        taxelList[i] = (unsigned int)(v[index++]);
    pressure    = v[index++];

    return true;
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



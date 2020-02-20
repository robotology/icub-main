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

vector<SkinPart> iCub::skinDynLib::getSkinParts(BodyPart b)
{
    vector<SkinPart> res;
    for(unsigned int i=0; i<SKIN_PART_SIZE; i++)
        if(SkinPart_2_BodyPart[i].body==b)
            res.push_back(SkinPart_2_BodyPart[i].skin);
    return res;
}

BodyPart iCub::skinDynLib::getBodyPart(SkinPart s)
{
    for(unsigned int i=0; i<SKIN_PART_SIZE; i++)
        if(SkinPart_2_BodyPart[i].skin==s)
            return SkinPart_2_BodyPart[i].body;
    return BODY_PART_UNKNOWN;
}

int iCub::skinDynLib::getLinkNum(SkinPart s)
{
    for(unsigned int i=0; i<SKIN_PART_SIZE; i++)
        if(SkinPart_2_LinkNum[i].skin==s)
            return SkinPart_2_LinkNum[i].linkNum;
    return -1;
}

SkinPart iCub::skinDynLib::getSkinPartFromString(const std::string skinPartString)
{

   if (skinPartString == SkinPart_s[SKIN_LEFT_HAND])
        return SKIN_LEFT_HAND;
   else if(skinPartString == SkinPart_s[SKIN_LEFT_FOREARM])
        return SKIN_LEFT_FOREARM;
   else if(skinPartString == SkinPart_s[SKIN_LEFT_UPPER_ARM])
        return SKIN_LEFT_UPPER_ARM;
   else if(skinPartString == SkinPart_s[SKIN_RIGHT_HAND])
        return SKIN_RIGHT_HAND;
   else if(skinPartString == SkinPart_s[SKIN_RIGHT_FOREARM])
        return SKIN_RIGHT_FOREARM;
   else if(skinPartString == SkinPart_s[SKIN_RIGHT_UPPER_ARM])
        return SKIN_RIGHT_UPPER_ARM;
   else if(skinPartString == SkinPart_s[SKIN_FRONT_TORSO])
        return SKIN_FRONT_TORSO;
   else if(skinPartString == SkinPart_s[LEFT_LEG_UPPER])
        return LEFT_LEG_UPPER;
   else if(skinPartString == SkinPart_s[LEFT_LEG_LOWER])
        return LEFT_LEG_LOWER;
   else if(skinPartString == SkinPart_s[LEFT_FOOT])
        return LEFT_FOOT;
   else if(skinPartString == SkinPart_s[RIGHT_LEG_UPPER])
        return RIGHT_LEG_UPPER;
   else if(skinPartString == SkinPart_s[RIGHT_LEG_LOWER])
        return RIGHT_LEG_LOWER;
   else if(skinPartString == SkinPart_s[RIGHT_FOOT])
       return RIGHT_FOOT;
   else
       return SKIN_PART_UNKNOWN;


}

yarp::sig::Vector iCub::skinDynLib::toVector(yarp::sig::Matrix m)
{
    Vector res(m.rows()*m.cols(),0.0);

    for (int r = 0; r < m.rows(); r++)
    {
        res.setSubvector(r*m.cols(),m.getRow(r));
    }

    return res;
}

yarp::sig::Vector iCub::skinDynLib::vectorFromBottle(const yarp::os::Bottle b, int in, const int size)
{
    yarp::sig::Vector v(size,0.0);

    for (int i = 0; i < size; i++)
    {
        v[i] = b.get(in).asDouble();
        in++;
    }
    return v;
}

void iCub::skinDynLib::vectorIntoBottle(const yarp::sig::Vector v, yarp::os::Bottle &b)
{
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

yarp::sig::Matrix iCub::skinDynLib::matrixFromBottle(const yarp::os::Bottle b, int in, const int r, const int c)
{
    yarp::sig::Matrix m(r,c);
    m.zero();

    for (int i = 0; i<r; i++)
    {
        for (int j = 0; j<c; j++)
        {
            m(i,j) = b.get(in).asDouble();
            in++;
        }
    }

    return m;
}

void iCub::skinDynLib::matrixIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b)
{
    Vector v = toVector(m);

    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

#include "iCubAddKinChain.h"
#include <string>
using namespace std;
using namespace iKin;

/************************************************************************/
iCubWrist::iCubWrist()
{
    allocate("right");
}


/************************************************************************/
iCubWrist::iCubWrist(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iCubWrist::iCubWrist(const iCubWrist &wrist)
{
    clone(wrist);
}


/************************************************************************/
void iCubWrist::allocate(const string &_type)
{
    iKinLimb::allocate(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);

    if (type=="right")
    {
        linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0, -0.10774,  M_PI/2.0,         -M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0, -M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0, -0.15228, -M_PI/2.0, -105.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(     0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,  -0.0773,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
    }
    else
    {
        linkList[0]=new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2]=new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3]=new iKinLink(       0.0,  0.10774, -M_PI/2.0,          M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4]=new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0,              0.0, 160.8*M_PI/180.0);
        linkList[5]=new iKinLink(       0.0,  0.15228, -M_PI/2.0,   75.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6]=new iKinLink(    -0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7]=new iKinLink(       0.0,   0.0773,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
    }

    for (unsigned int i=0; i<8; i++)
        *this << *linkList[i];

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
}


/************************************************************************/
iCubThirdEye::iCubThirdEye()
{
    allocate("");
}


/************************************************************************/
iCubThirdEye::iCubThirdEye(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iCubThirdEye::iCubThirdEye(const iCubThirdEye &eye)
{
    clone(eye);
}


/************************************************************************/
void iCubThirdEye::allocate(const string &_type)
{
    iKinLimb::allocate(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);

    linkList[0]=new iKinLink(   0.032,    0.0,  M_PI/2.0,       0.0, -22.0*M_PI/180.0, 84.0*M_PI/180.0);
    linkList[1]=new iKinLink(     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -39.0*M_PI/180.0, 39.0*M_PI/180.0);
    linkList[2]=new iKinLink( 0.00231,-0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*M_PI/180.0, 59.0*M_PI/180.0);
    linkList[3]=new iKinLink(   0.033,    0.0,  M_PI/2.0,  M_PI/2.0, -40.0*M_PI/180.0, 30.0*M_PI/180.0);
    linkList[4]=new iKinLink(     0.0,    0.0,  M_PI/2.0,  M_PI/2.0, -70.0*M_PI/180.0, 60.0*M_PI/180.0);
    linkList[5]=new iKinLink(  -0.054, 0.0825, -M_PI/2.0, -M_PI/2.0, -55.0*M_PI/180.0, 55.0*M_PI/180.0);
    linkList[6]=new iKinLink(     0.0,    0.0, -M_PI/2.0,       0.0, -35.0*M_PI/180.0, 15.0*M_PI/180.0);
    linkList[7]=new iKinLink(     0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -50.0*M_PI/180.0, 50.0*M_PI/180.0);

    for (unsigned int i=0; i<linkList.size(); i++)
        *this << *linkList[i];

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
}

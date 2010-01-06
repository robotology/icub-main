#include "ICubArmDrumStick.h"

void ICubArmDrumStick::_allocate_limb(const string &_type, double drumStickLength)
{
    iKinLimb::_allocate_limb(_type);

    H0.zero();
    H0(0,1)=-1;
    H0(1,2)=-1;
    H0(2,0)=1;
    H0(3,3)=1;

    linkList.resize(8);
    //linkList.resize(10);

	double totalForearm = 0.1373 + drumStickLength;
    if (type=="right")
    {
        // link 0-9 are the standard links for the right arm
        linkList[0] =new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1] =new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2] =new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3] =new iKinLink(       0.0, -0.10774,  M_PI/2.0,         -M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4] =new iKinLink(       0.0,      0.0, -M_PI/2.0,         -M_PI/2.0,              0.0, 45*M_PI/180.0);
        linkList[5] =new iKinLink(       0.0, -0.15228, -M_PI/2.0, -105.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6] =new iKinLink(     0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7] =new iKinLink(       0.0,  -0.1373,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[8]=new iKinLink(  drumStickLength, 0.0, M_PI/2.0, M_PI/2.0,     -10*M_PI/180.0,    10*M_PI/180.0);
       /* linkList[8] =new iKinLink(       0.0,      0.0,  M_PI/2.0,          M_PI/2.0, -90.0*M_PI/180.0,   0.0*M_PI/180.0);
        linkList[9] =new iKinLink(    0.0625,    0.016,       0.0,              M_PI, -20.0*M_PI/180.0,  40.0*M_PI/180.0);*/

        //link 10 is the drumming stick
        //linkList[10]=new iKinLink(  drumStickLength, 0.0, 0.0, M_PI/2.0,     M_PI/2,     M_PI/2);
    }
    else // the same for the left arm
    {
        linkList[0] =new iKinLink(     0.032,      0.0,  M_PI/2.0,               0.0, -22.0*M_PI/180.0,  84.0*M_PI/180.0);
        linkList[1] =new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0, -39.0*M_PI/180.0,  39.0*M_PI/180.0);
        linkList[2] =new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*M_PI/180.0, -59.0*M_PI/180.0,  59.0*M_PI/180.0);
        linkList[3] =new iKinLink(       0.0,  0.10774, -M_PI/2.0,          M_PI/2.0, -95.5*M_PI/180.0,   5.0*M_PI/180.0);
        linkList[4] =new iKinLink(       0.0,      0.0,  M_PI/2.0,         -M_PI/2.0,              0.0, 45*M_PI/180.0);
        linkList[5] =new iKinLink(       0.0,  0.15228, -M_PI/2.0,   75.0*M_PI/180.0, -37.0*M_PI/180.0,  90.0*M_PI/180.0);
        linkList[6] =new iKinLink(    -0.015,      0.0,  M_PI/2.0,               0.0,   5.5*M_PI/180.0, 106.0*M_PI/180.0);
        linkList[7] =new iKinLink(       0.0,   0.1373 + drumStickLength ,  M_PI/2.0,         -M_PI/2.0, -90.0*M_PI/180.0,  90.0*M_PI/180.0);
        //linkList[8]=new iKinLink(  drumStickLength, 0.0, M_PI/2.0, M_PI/2.0,     -10*M_PI/180.0,    10*M_PI/180.0);
        /*linkList[8] =new iKinLink(       0.0,      0.0,  M_PI/2.0,          M_PI/2.0, -90.0*M_PI/180.0,   0.0*M_PI/180.0);
        linkList[9] =new iKinLink(    0.0625,   -0.016,       0.0,               0.0, -20.0*M_PI/180.0,  40.0*M_PI/180.0);*/
        
		//drumming stick
        //linkList[10]=new iKinLink(  -drumStickLength, 0.0, 0.0, 0.0,     -10*M_PI/180.0,     10*M_PI/180.0);
    }

    // here the chain is actually built by pushing back the links in the list
    for (unsigned int i=0; i<linkList.size(); i++)
        *this << *(linkList[i]);

	// blocks the link for the stick since it's not a DoF 
	//blockLink(8);
	blockLink(0, 0.0);
	blockLink(1, 0.0);
	blockLink(2, 0.0);
	//blockLink(4, 0.0);
	/*blockLink(7, 0.0);
	blockLink(8, 0.0);
	blockLink(9, 0.0);*/
	//blockLink(8, 0.0);
}

ICubArmDrumStick::ICubArmDrumStick()
{
    _allocate_limb("left", 0);
}

ICubArmDrumStick::ICubArmDrumStick(const string &_type, double drumStickLength)
{
    _allocate_limb(_type, drumStickLength);
}

ICubArmDrumStick::ICubArmDrumStick(const ICubArmDrumStick &armDrumStick)
{
    _copy_limb(armDrumStick);
}
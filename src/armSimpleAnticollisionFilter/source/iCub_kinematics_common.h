/*
 * Common data for the iCub forward/reverse kinematics
 * Alexis Maldonado maldonad _at_ cs.tum.edu
 * Released under the GPLv3 or any later version (at your option)
 */


#ifndef icub_kine_h_
#define icub_kine_h_

#include <gnugraph.h>
#include <quaternion.h>
#include <robot.h>
#include <utils.h>

#ifdef use_namespace
using namespace ROBOOP;
#endif


//Position and rotation of the base of the arm in the world
const Real base_origin[] = {0.0 , 0.0, 0.63};  // in world coordinates
const Real rpy_arm2world[] = {-M_PI/2 , -M_PI/2, 0};
const Real rpy_world2arm[] = {M_PI/2, 0 , M_PI/2};


//Set up RoboOp
// Theta is the controlled joint var for rotational joints (type=0)
//type, theta, d , a ,alfa, mintetha, maxtetha, joint_offset(! gets added to whatever we set in tetha), ...
const Real RR_data_james_vvv[] = {
    0, 0, 0.0    ,  0.0,  M_PI/2,   -M_PI*170.0/180.0, M_PI*170.0/180.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.075  ,  0.0,  M_PI/2,   -M_PI*130.0/180.0, M_PI*130.0/180.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, -0.245 ,    0, -M_PI/2,   -M_PI*130.0/180.0, M_PI*130.0/180.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0      , 0.17,       0,   -M_PI*170.0/180.0, M_PI*170.0/180.0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0
};

// Theta is the controlled joint var for rotational joints (type=0)
//type, theta, d , a ,alfa, mintetha, maxtetha, joint_offset(! gets added to whatever we set in tetha), ...
const Real RR_data_iCubv3[] = {
    0, 0,-0.10980, 0.00000, M_PI/2, -M_PI*95.00/180.0,-M_PI*05.00/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.00000,-M_PI/2,  M_PI*020.0/180.0, M_PI*160.8/180.0,-M_PI/2, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0,-0.15146, 0.00000,-M_PI/2,  M_PI*10.00/180.0, M_PI*144.0/180.0, M_PI  , 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.01500, M_PI/2,  M_PI*05.50/180.0, M_PI*106.0/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0,-0.13750, 0.00000, M_PI/2, -M_PI*50.00/180.0, M_PI*50.00/180.0, M_PI  , 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.00000, M_PI/2, -M_PI*65.00/180.0, M_PI*10.00/180.0, M_PI/2, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000,-0.06250, 0.0000, -M_PI*25.00/180.0, M_PI*25.00/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
};

// Theta is the controlled joint var for rotational joints (type=0)
//type, theta, d , a ,alfa, mintetha, maxtetha, joint_offset(! gets added to whatever we set in tetha), ...
const Real RR_data_iCubv3_safelimits[] = {
    0, 0,-0.10980, 0.00000, M_PI/2, -M_PI*70.00/180.0,-M_PI*25.60/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.00000,-M_PI/2,  M_PI*020.0/180.0, M_PI*110.8/180.0,-M_PI/2, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0,-0.15146, 0.00000,-M_PI/2,  M_PI*45.00/180.0, M_PI*130.0/180.0, M_PI  , 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.01500, M_PI/2,  M_PI*30.00/180.0, M_PI*070.0/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0,-0.13750, 0.00000, M_PI/2, -M_PI*50.00/180.0, M_PI*50.00/180.0, M_PI  , 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000, 0.00000, M_PI/2, -M_PI*65.00/180.0, M_PI*10.00/180.0, M_PI/2, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
    0, 0, 0.00000,-0.06250, 0.0000, -M_PI*25.00/180.0, M_PI*25.00/180.0, 0.0000, 1.0, 0, 0, 0, 0, 0, 0, 0.1666666, 0, 0.1666666, 0, 0, 0, 0, 0,
};



//Gets values in degrees from the network
ColumnVector encoder2angle( const ColumnVector &encoders);
ColumnVector angle2command( const ColumnVector &angles);
ReturnMatrix coord_arm2world(const Matrix &acoord);
ReturnMatrix coord_world2arm(const Matrix &wcoord);

float deg2rad(float deg);


#endif // icub_kine_h_


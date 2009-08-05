/*
 * Common data for the iCub forward/reverse kinematics
 * Alexis Maldonado maldonad _at_ cs.tum.edu
 * Released under the GPLv3 or any later version (at your option)
 */

#include "iCub_kinematics_common.h"

//Gets values in degrees from the network
ColumnVector encoder2angle( const ColumnVector &encoders) {

    ColumnVector radianAngles(encoders.size());
    //right now, simply convert to degrees
    for (int i=1 ; i<=encoders.size() ; ++i ) {
        radianAngles(i)=deg2rad(encoders(i));
    }

    //If not already taken into account by the DH parameters and the offsets there (the right place to do it), then you can fix offsets or signs here:
    //Example:
    //encoders[0]=-encoders[0];
    //ColumnVector offset(7);
    //offset << 0 <<  -M_PI/2 << 0 <<0 << M_PI << M_PI/2,0 ;
    //ColumnVector angles(7);
    //angles=encoders+offset;
    //return angles;

    return radianAngles;
}

ColumnVector angle2command( const ColumnVector &angles) {
    //receives angles in rads, and prepares the commands to send to the robot
    //right now, this simply converts to degrees

    ColumnVector degAngles(angles.size());
    //right now, simply convert to degrees
    for (int i=1 ; i<=angles.size() ; ++i ) {
        degAngles(i)=rad2deg(angles(i));
    }

    return(degAngles);

}



//Needs a 4x4 Matrix to perform the homogeneous transformation
ReturnMatrix coord_arm2world(const Matrix &acoord) {
    //Let's test the matrixes for translation/rotation
    ColumnVector t1(3);
    ColumnVector r1(3);

    t1 << base_origin;
    r1 << rpy_arm2world;

    Matrix rotation= rpy(r1);
    Matrix translation= trans(t1);
    Matrix result;

    result=translation*(rotation*acoord); //We rotate the point (acord) to parallel to the world's coord, then we translate
    result.release();
    return result;

}


ReturnMatrix coord_world2arm(const Matrix &wcoord) {
    ColumnVector t1(3);
    ColumnVector r1(3);

    t1 << base_origin;
    t1(1)=-t1(1);  //invert the signs
    t1(2)=-t1(2);
    t1(3)=-t1(3);
    cout << "t1: " << t1 << "\n";

    r1 << rpy_world2arm;
    cout << "r1: " << r1 << "\n";

    Matrix rotation= rpy(r1);
    Matrix translation= trans(t1);
    Matrix result;

    result=rotation*(translation*wcoord);
    result.release();
    return result;

}



float deg2rad(float deg) {
    return (deg*M_PI/180.0);
}

float rad2deg(float rad) {
    return (rad*180.0/M_PI);
}


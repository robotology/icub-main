/*
 * Simple collision avoidance for the iCub. Done at the VVV'07 Camp in Genoa, Italy
 *  by Alexis Maldonado   maldonad _at_ cs.tum.edu
 *
 *  Receives the angles of the joints on a port and, if they are valid, forwards them to another port
 *  Also sends x,y,z, pose(quaternion) and pose(roll/pitch/yaw) on another port
 *  The frame of reference is at the base of the robot, with X going out the front of the robot, and Z going up
 *
 *  Uses Roboop, YARP, TClap , and ACE (for signals)
 *
 *  Released under the GPLv3 License or any later version (at your option)
 */


#include "anticollisionFilter.h"
#include "iCub_kinematics_common.h"



#include <unistd.h>
#include <iostream>
#include <string>
#include <math.h>

//The Command Line Parser
#include "tclap/CmdLine.h"
using namespace TCLAP;


//YARP includes
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Bottle.h>

using namespace yarp::os;
using namespace std;



//Global variables for the robot class (from RoboOp)
unsigned int ndof(0);
Robot robot;
Matrix initrob;


bool go_on;

int main(int argc, char *argv[]) {

    std::string portname("");
    std::string mainWindowTitle ("YARP Anticollision simple filter");
    try {
        // Define the command line object.
        CmdLine cmd("YARP Anticollision simple filter", ' ', "0.1");

        ValueArg<std::string> portnameArg("p","portprefix","Prefix of the name to register in the server",false,"/icub/right_arm/anticol","string");

        cmd.add( portnameArg );

        // Parse the args.
        cmd.parse( argc, argv );

        // Get the value parsed by each arg.
        portname = portnameArg.getValue();

    } catch (ArgException &e) { // catch any exceptions
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        return(-1);
    }



    // Set up signal handler to catch the interrupt signal.
    ACE_Select_Reactor reactor_impl;
    ACE_Reactor reactor (&reactor_impl);
    Sig_Handler handler (reactor);


    //Load up the model from the DH Parameters
    ndof=sizeof(RR_data_iCubv3)/(sizeof(Real))/23;
    cout << "DOF=" << ndof << "\n";

    initrob=Matrix(ndof,23);

    initrob << RR_data_iCubv3;
    robot = Robot(initrob);

    int dof_check = robot.get_dof();
    if (ndof != dof_check) {

        cout << "Something is wrong with the DH Parameters. The detect number of DOF is not consistent.\n";
    }


    //YARP Stuff
    Network::init();

    BufferedPort<Bottle> portIn;
    BufferedPort<Bottle> portOut;
    BufferedPort<Bottle> portPosOut;
    BufferedPort<Bottle> portWorkedOut;

    string inportname=portname + "/joints_in";
    portIn.open(inportname.c_str());

    string outportname=portname + "/joints_out";
    portOut.open(outportname.c_str());

    string posportname=portname + "/pospose_out";
    portPosOut.open(posportname.c_str());

    string workedportname=portname + "/worked";
    portWorkedOut.open(workedportname.c_str());


    Matrix lims(ndof,2);  //This matrix contains the joint limits

    for (unsigned int i=1  ; i<=ndof ; ++i) {  //get the joint limits from the DH Parameters
        lims(i,1)=initrob(i,6);
        lims(i,2)=initrob(i,7);

        cout << "Joint #" << i << "  minVal=" << rad2deg(lims(i,1)) << "  maxVal=" << rad2deg(lims(i,2)) << "\n";
    }


    go_on=true;

    while (go_on) {

        Bottle *input = portIn.read(false); //true -> Blocking read
        if (input!=NULL) {

            //std::cout << "Got a message!\n";

            //we expect at least the position from the first NDOF joints
            ColumnVector joint_angle(ndof);

            if (input->size() >= ndof) {
                for (int i=1; i<=ndof; ++i) {
                    joint_angle(i) = input->get(i-1).asDouble();  // get() begins in 0, and joint_angle() in 1
                }
            }

            ColumnVector angles_internal(ndof);
            angles_internal = encoder2angle ( joint_angle );
            //std::cout << "The joints have internal angles: \n";
            //std::cout << angles_internal;

            //setting the measured angles in the model of the robot (DH Parameters)
            robot.set_q(angles_internal);

            Matrix arm_coords=robot.kine();
            Matrix world_coord=coord_arm2world(arm_coords);

            //cout << "Arm Pos: " << arm_coords.Column(4) << "\n";
            cout << "In World coordinates:\n";
            cout << setw(7) << setprecision(3) << world_coord.Column(4);

            //cout << "Back to arm  coordinates:\n";
            //cout << setw(7) << setprecision(3) << coord_world2arm(world_coord);


            Matrix hand_world=coord_arm2world(robot.kine());
            Matrix elbow_world=coord_arm2world(robot.kine(4));


            //let's first check for the limits
            bool inside_limits(true);

            //if any limit is not respected, limitsOK will be false
            for (unsigned int j=1; j<=ndof ; ++j) {
                inside_limits = inside_limits && ( (angles_internal(j) > lims(j,1) ) && (angles_internal(j) < lims(j,2) )  );
            }

            //check if inside the safe volumne
            bool inside_safepos(false);
            inside_safepos =  positionInBox(hand_world.column(4)) && positionOk(hand_world.column(4)) && positionOk_withBack(elbow_world.column(4));


            if ( inside_limits && inside_safepos ) {

                //Actually send the info on the net:

                //forward the desired joint angles (they are hopefully safe)
                Bottle& jointsoutput = portOut.prepare();
                jointsoutput.clear();

                //simply put the received angles into a bottle
                for (unsigned int j=1 ; j <= ndof ; ++j) {
                    jointsoutput.addDouble(joint_angle(j));
                }
                portOut.write();


                //Send X,Y,Z, Quaternion, RPY information out through PosOut port
                Bottle& coordoutput = portPosOut.prepare();
                coordoutput.clear();

                ColumnVector worldpos=hand_world.Column(4);

                coordoutput.addDouble(worldpos(1));
                coordoutput.addDouble(worldpos(2));
                coordoutput.addDouble(worldpos(3));

                //let's also send the quaternion of the hand in the world's frame of reference
                //quaternion in the format s v_  (s v1 v2 v3)
                Quaternion qhand(hand_world);
                coordoutput.addDouble(qhand.s());
                Matrix tmpV=qhand.v();
                coordoutput.addDouble(tmpV(1,1));
                coordoutput.addDouble(tmpV(2,1));
                coordoutput.addDouble(tmpV(3,1));


                //Let's also send the Roll/Pitch/Yaw angles
                ColumnVector rpyVector(3);
                rpyVector=irpy(hand_world);
                coordoutput.addDouble(rpyVector(1));
                coordoutput.addDouble(rpyVector(2));
                coordoutput.addDouble(rpyVector(3));

                portPosOut.write();


                //forward the desired joint angles (they are hopefully safe)
                Bottle& workedoutput = portWorkedOut.prepare();
                workedoutput.clear();

                //1 if it was a safe position
                workedoutput.addInt(1);
                portWorkedOut.write();




            } else {
                //The desired position is not safe
                cout << "the desired angles are not safe. Sorry!.\n";

                //forward the desired joint angles (they are hopefully safe)
                Bottle& workedoutput = portWorkedOut.prepare();
                workedoutput.clear();

                //0 means it did not like the position
                workedoutput.addInt(0);
                portWorkedOut.write();



            }




        } else { //no packet received
            //std::cout << "." << std::endl;
            yarp::os::Time::delay(0.0001);

        }


    }



    return(0);
}



bool inCylinder(const ColumnVector &Pos) {

    //cylinder going up Z, centered in x=a, y=b
    float a(0.0);
    float b(0.0);
    float radiuscyl(0.2);
    float zhigh(1.5);
    float zlow(-0.3);


    if ( (Pos(3)<zhigh) && (Pos(3)>zlow)  ) {
        //not too high or too low  -> we have to check the radius
        float radius=sqrt( (Pos(1)-a)*(Pos(1)-a) + (Pos(2)-b)*(Pos(2)-b) );
        if (radius < radiuscyl) {
            cout << "INCYLINDER\n";
            return(true);
        }
    }


    return(false);


}


bool behindBack(const ColumnVector &Pos) {

    //The back is a plane with X~ normal  -> x<0 means on the back


    if (Pos(1)<0.0) {
        return(true);
    }

    return(false);


}

bool wrongSide(const ColumnVector &Pos) {
    //get right of the points on the left side of the robot
    //these are points with a positive y coordinate)
    if (Pos(2) > 0.1) {
        return(true);
    }

    return(false);

}


bool positionOk(const ColumnVector &Pos) {


    return ( !(inCylinder(Pos) || behindBack(Pos) || wrongSide(Pos)) );

}

bool positionOk_withBack(const ColumnVector &Pos) {


    return ( !( wrongSide(Pos)) || inCylinder(Pos)  );

}

bool positionInBox(const ColumnVector &Pos) {

    float corner0[]={0.00,-0.60,0.0};
    float corner1[]={0.60,0.1,1.5};

    if (  ( Pos(1) > corner0[0] ) && ( Pos(1) < corner1[0] ) && ( Pos(2) > corner0[1] ) && ( Pos(2) < corner1[1] ) && ( Pos(3) > corner0[2] ) && ( Pos(3) < corner1[2] ) ) {
        return(true);
    }
    cout << "OUT OF BOX\n";

    return(false);
}

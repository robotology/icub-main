/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Carlo Ciliberto, Vadim Tikhanoff
* email:   carlo.ciliberto@iit.it vadim.tikhanoff@iit.it
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


#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>

#include <gsl/gsl_math.h>

#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <iCub/MotorThread.h>

using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace iCub::perception;


bool MotorThread::checkOptions(Bottle &options, const string &parameter)
{
    bool found=false;
    for (int i=0; i<options.size(); i++)
    {
        if(options.get(i).asString()==parameter.c_str())
        {
            found=true;
            break;
        }
    }

    return found;
}


//Dragger initializer
void Dragger::init(Bottle &initializer, double thread_rate)
{
    extForceThresh=initializer.check("external_force_thresh",Value(1e9)).asDouble();
    samplingRate=initializer.check("subsampling_rate",Value(500.0)).asDouble();

    damping=initializer.check("damping",Value(1e9)).asDouble();
    inertia=initializer.check("inertia",Value(1e9)).asDouble();

    Vector zeros3d(3);
    zeros3d=0.0;
    I=new Integrator(thread_rate/1000.0,zeros3d);

    ctrl=NULL;
    actionName="";
}
//-------------------



int MotorThread::checkArm(int arm)
{
    if(arm!=ARM_IN_USE && arm!=ARM_MOST_SUITED && action[arm]!=NULL)
        armInUse=arm;

    return armInUse;
}


int MotorThread::checkArm(int arm, Vector &xd, const bool applyOffset)
{
    if(arm==ARM_MOST_SUITED)
        arm=xd[1]<0.0?LEFT:RIGHT;

    if(arm!=ARM_IN_USE && arm!=ARM_MOST_SUITED && action[arm]!=NULL)
        armInUse=arm;

    if (applyOffset)
    {
        xd[0]+=currentKinematicOffset[armInUse][0]; 
        xd[1]+=currentKinematicOffset[armInUse][1];
        xd[2]+=currentKinematicOffset[armInUse][2];
    }

    return checkArm(arm);
}


bool MotorThread::setImpedance(bool turn_on)
{
    bool done=true;
    if(turn_on)
    {
        for(int i=0; i<5; i++)
        {
            if(action[LEFT]!=NULL)
                done=done && int_mode_arm[LEFT]->setInteractionMode(i,VOCAB_IM_COMPLIANT);
            if(action[RIGHT]!=NULL)
                done=done && int_mode_arm[RIGHT]->setInteractionMode(i,VOCAB_IM_COMPLIANT);
        }
        
        for(int i=0; i<3; i++)
            done=done && int_mode_torso->setInteractionMode(i,VOCAB_IM_STIFF);

        status_impedance_on=done;
    }
    else
    {
        for(int i=0; i<5; i++)
        {
            if(action[LEFT]!=NULL)
                done=done && int_mode_arm[LEFT]->setInteractionMode(i,VOCAB_IM_STIFF);
            if(action[RIGHT]!=NULL)
                done=done && int_mode_arm[RIGHT]->setInteractionMode(i,VOCAB_IM_STIFF);
        }

        for(int i=0; i<3; i++)
            done=done && int_mode_torso->setInteractionMode(i,VOCAB_IM_STIFF);

        status_impedance_on=!done;
    }

    return done;
}



bool MotorThread::setTorque(bool turn_on, int arm)
{
    if(action[arm]==NULL)
        return false;

    bool done=true;

    //if the system is asked to turn on impedance control
    if(turn_on)
    {
        for(int i=0; i<4; i++)
            done=done && ctrl_mode_arm[arm]->setTorqueMode(i);

        done=done && ctrl_mode_torso->setTorqueMode(0);
        done=done && int_mode_torso->setInteractionMode(2,VOCAB_IM_COMPLIANT);
    }
    else
        done=setImpedance(status_impedance_on);

    return done;
}


int MotorThread::setStereoToCartesianMode(const int &mode)
{
    Bottle reply;
    return setStereoToCartesianMode(mode,reply);
}

int MotorThread::setStereoToCartesianMode(const int &mode, Bottle &reply)
{
    switch(mode)
    {
        case S2C_DISPARITY:
        {
            if(Network::isConnected(disparityPort.getName().c_str(),"/stereoDisparity/rpc"))
            {
                modeS2C=S2C_DISPARITY;
                fprintf(stdout,"[Stereo -> Cartesian]: Disparity\n");
                reply.addString("[Stereo -> Cartesian]: Disparity");
            }
            else
            {
                modeS2C=S2C_HOMOGRAPHY;
                fprintf(stdout,"[Stereo -> Cartesian]: Homography\n");
                reply.addString("[Stereo -> Cartesian]: Homography");
            }
            break;
        }

        case S2C_NETWORK:
        {
            if(neuralNetworkAvailable)
            {
                modeS2C=S2C_NETWORK;
                fprintf(stdout,"[Stereo -> Cartesian]: Neural Net\n");
                reply.addString("[Stereo -> Cartesian]: Neural Net");
            }
            else
            {
                modeS2C=S2C_HOMOGRAPHY;
                fprintf(stdout,"[Stereo -> Cartesian]: Homography\n");
                reply.addString("[Stereo -> Cartesian]: Homography");
            }
            break;
        }

        default:
        {
            modeS2C=S2C_HOMOGRAPHY;
            fprintf(stdout,"[Stereo -> Cartesian]: Homography\n");
            reply.addString("[Stereo -> Cartesian]: Homography");
            break;
        }
    }
    

    //get left offsets
    if(!bKinOffsets.find("left").asList()->check(Vocab::decode(modeS2C).c_str()))
    {

        Bottle &bKinMode=bKinOffsets.find("left").asList()->addList();
        bKinMode.addString(Vocab::decode(modeS2C).c_str());
        Bottle &bKinVec=bKinMode.addList();
        for(int i=0; i<3; i++)
            bKinVec.addDouble(0.0);
    }

    Bottle *bKinLeft=bKinOffsets.find("left").asList()->find(Vocab::decode(modeS2C).c_str()).asList();
    for(int i=0; i<3; i++)
        defaultKinematicOffset[LEFT][i]=bKinLeft->get(i).asDouble();

    
    //get left offsets
    if(!bKinOffsets.find("right").asList()->check(Vocab::decode(modeS2C).c_str()))
    {

        Bottle &bKinMode=bKinOffsets.find("right").asList()->addList();
        bKinMode.addString(Vocab::decode(modeS2C).c_str());
        Bottle &bKinVec=bKinMode.addList();
        for(int i=0; i<3; i++)
            bKinVec.addDouble(0.0);
    }

    Bottle *bKinRight=bKinOffsets.find("right").asList()->find(Vocab::decode(modeS2C).c_str()).asList();
    for(int i=0; i<3; i++)
        defaultKinematicOffset[RIGHT][i]=bKinRight->get(i).asDouble();

    return modeS2C;
}


bool MotorThread::targetToCartesian(Bottle *bTarget, Vector &xd)
{
    bool found=false;

    // set the default current kinematic offsets for the arms
    currentKinematicOffset[LEFT]=defaultKinematicOffset[LEFT];
    currentKinematicOffset[RIGHT]=defaultKinematicOffset[RIGHT];

    // if the tartget's cartesian coordinates was specified, use them.
    if(!found && bTarget!=NULL && bTarget->check("cartesian") && bTarget->find("cartesian").asList()->size()>=3)
    {
        Bottle *bCartesian=bTarget->find("cartesian").asList();

        xd.clear();
        for(int i=0; i<bCartesian->size(); i++)
            xd.push_back(bCartesian->get(i).asDouble());

        found=true;
    }

    // if an object was specified check for its 3D position associated to the object
    if(!found && bTarget!=NULL && bTarget->check("name"))
        if(opcPort.getCartesianPosition(bTarget->find("name").asString().c_str(),xd))
            found=true;

    if(!found && bTarget!=NULL && bTarget->check("stereo"))
    {
        Bottle *bStereo=bTarget->find("stereo").asList();

        if(bStereo!=NULL)
        {
            Vector stereo;
            for(int i=0; i<bStereo->size(); i++)
                stereo.push_back(bStereo->get(i).asDouble());

            found=stereoToCartesian(stereo,xd);
        }
    }

    if(found && bTarget!=NULL && bTarget->check("name"))
        opcPort.getKinematicOffsets(bTarget->find("name").asString().c_str(),currentKinematicOffset);

    return found;
}


bool MotorThread::stereoToCartesian(const Vector &stereo, Vector &xd)
{
    if(stereo.size()!=4)
        return false;

    if(arm_mode!=ARM_MODE_IDLE)
    {
        fprintf(stdout,"System is busy!\n");
        return false;
    }

    bool ok=false;

    switch(modeS2C)
    {
        case S2C_HOMOGRAPHY:
        {
            ok=stereoToCartesianHomography(stereo,xd);
            break;
        }

        case S2C_DISPARITY:
        {
            ok=stereoToCartesianDisparity(stereo,xd);
            break;
        }

        case S2C_NETWORK:
        {
            ok=stereoToCartesianNetwork(stereo,xd);
            break;
        }
    }

    return ok;
}


bool MotorThread::loadExplorationPoses(const string &file_name)
{
    Property optExpl;
    optExpl.fromConfigFile(rf.findFile(file_name.c_str()));

    if(!optExpl.check("torso") ||!optExpl.check("hand") ||!optExpl.check("head"))
        return false;

    fprintf(stdout,"\nTORSO\n");
    Bottle &tmpTorso=optExpl.findGroup("torso");
    for(int i=1; i<tmpTorso.size(); i++)
    {
        Bottle *b=tmpTorso.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        pos_torsoes.push_back(v);
    }

    fprintf(stdout,"\nHAND\n");
    Bottle &tmpHand=optExpl.findGroup("hand");
    for(int i=1; i<tmpHand.size(); i++)
    {
        Bottle *b=tmpHand.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        handPoses.push_back(v);
    }
    
    fprintf(stdout,"\nHEAD\n");
    Bottle &tmpHead=optExpl.findGroup("head");
    for(int i=1; i<tmpHead.size(); i++)
    {
        Bottle *b=tmpHead.get(i).asList();
        Vector v(b->size());
        for(int j=0; j<b->size(); j++)
            v[j]=b->get(j).asDouble();
        fprintf(stdout,"%s\n",v.toString().c_str());
        headPoses.push_back(v);
    }

    return true;
}


Vector MotorThread::eye2root(const Vector &out, bool forehead)
{
    Vector xd;
    if(out.size()!=3)
        return xd;

    Vector out_hom(4);
    out_hom[0]=out[0];
    out_hom[1]=out[1];
    out_hom[2]=out[2];
    out_hom[3]=1.0;

    Vector eyePos,eyeOrient;
    if(forehead)
        ctrl_gaze->getHeadPose(eyePos,eyeOrient);
    else
        ctrl_gaze->getLeftEyePose(eyePos,eyeOrient);

    Matrix T=axis2dcm(eyeOrient);
    T(0,3)=eyePos[0];
    T(1,3)=eyePos[1];
    T(2,3)=eyePos[2];

    Vector root=T*out_hom;

    // safe thresholding
    if (root[0]>-0.15)
        root[0]=-0.15;

    if (root[2]<=table_height)
        root[2]=table_height;

    xd.resize(3);
    xd[0]=root[0];
    xd[1]=root[1];
    xd[2]=root[2];

    return xd;
}


bool MotorThread::stereoToCartesianHomography(const Vector &stereo, Vector &xd)
{
    int eye_in_use;
    if(stereo[2*dominant_eye]==0.0 && stereo[2*dominant_eye+1]==0.0)
    {
        if(stereo[2*(1-dominant_eye)]==0.0 && stereo[2*(1-dominant_eye)+1]==0.0)
            return false;

        eye_in_use=1-dominant_eye;
    }
    else
        eye_in_use=dominant_eye;

    Vector px(2);
    px[0]=stereo[2*eye_in_use];
    px[1]=stereo[2*eye_in_use+1];

    ctrl_gaze->get3DPointOnPlane(eye_in_use,px,table,xd);

    return true;
}


bool MotorThread::stereoToCartesianDisparity(const Vector &stereo, Vector &xd)
{
    Bottle bEye;
    bEye.addDouble(stereo[0]);
    bEye.addDouble(stereo[1]);

    Bottle bX;

    do
    {
        disparityPort.write(bEye,bX);
    }
    while(bX.size()==0 || bX.get(2).asDouble()<0.0 );

    if(bX.size()!=0 && bX.get(2).asDouble()>0.0 )
    {
        xd.resize(3);
        xd[0]=bX.get(0).asDouble();
        xd[1]=bX.get(1).asDouble();    
        xd[2]=bX.get(2).asDouble();
    }

    xd=eye2root(xd,false);

    return xd.size()==3;
}


bool MotorThread::stereoToCartesianNetwork(const Vector &stereo, Vector &xd)
{
    if(stereo.size()!=4 || stereo[0]==0.0 || stereo[1]==0.0 || stereo[2]==0.0 || stereo[3]==0.0)
        return false;

    Vector h(6);
    enc_head->getEncoders(h.data());

    Vector in(7);
    in[0]=stereo[0];
    in[1]=stereo[1];
    in[2]=stereo[2];
    in[3]=stereo[3];
    in[4]=h[3];
    in[5]=h[4];
    in[6]=h[5];

    xd=eye2root(net.predict(in),true);

    return xd.size()==3;
}


Vector MotorThread::randomDeployOffset()
{
    Vector offset(3);
    offset[0]=Rand::scalar(-0.01,0.0);
    offset[1]=Rand::scalar(-0.02,0.02);
    offset[2]=0.0;

    return offset;
}


bool MotorThread::loadKinematicOffsets(const string &_kinematics_path)
{
    kinematics_path=_kinematics_path;

    ifstream kin_fin(kinematics_path.c_str());

    if(!kin_fin.is_open())
    {
        fprintf(stdout,"!!! Error. Kinematics file not found.\n");
        return false;
    }

    stringstream strstr;
    strstr<<kin_fin.rdbuf();

    bKinOffsets.fromString(strstr.str().c_str());

    if(!bKinOffsets.check("table_height"))
    {
        Bottle &bKinList=bKinOffsets.addList();
        bKinList.addString("table_height");
        bKinList.addDouble(0.2);
    }

    table_height=bKinOffsets.find("table_height").asDouble();
    table.resize(4);
    table=0.0;
    table[2]=1.0;
    table[3]=-table_height;

    //adjust the table height accordingly to a specified tolerance
    table_height+=table_height_tolerance;

    if(!bKinOffsets.check("left"))
    {
        Bottle &bKinList=bKinOffsets.addList();
        bKinList.addString("left");

        bKinList.addList();
    }
    
    if(!bKinOffsets.check("right"))
    {
        Bottle &bKinList=bKinOffsets.addList();
        bKinList.addString("right");

        bKinList.addList();
    }

    kin_fin.close();

    defaultKinematicOffset[LEFT].resize(3);
    defaultKinematicOffset[RIGHT].resize(3);

    return true;
}


bool MotorThread::saveKinematicOffsets()
{
    ofstream kin_fout(kinematics_path.c_str());

    if(!kin_fout.is_open())
        return false;

    kin_fout << bKinOffsets.toString() << endl;

    kin_fout.close();

    return true;
}


bool MotorThread::getGeneralOptions(Bottle &b)
{
    if (Bottle *pB=b.find("home_fixation_point").asList())
    {
        homeFixCartType=true;
        Value v=pB->get(0);
        int offs=0;

        if (v.isString())
        {
            homeFixCartType=(v.asString()=="xyz");
            offs++;
        }

        homeFix.resize(pB->size()-offs);
        for (size_t i=0; i<homeFix.length(); i++)
            homeFix[i]=pB->get(offs+i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_displacement").asList())
    {
        reachAboveDisp.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveDisp[i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("grasp_above_relief").asList())
    {
        graspAboveRelief.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            graspAboveRelief[i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("push_above_relief").asList())
    {
        pushAboveRelief.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            pushAboveRelief[i]=pB->get(i).asDouble();
    }
    else
        return false;

    targetInRangeThresh=b.find("target_in_range_thresh").asDouble();

    table_height_tolerance=b.find("table_height_tolerance").asDouble();

    return true;
}


bool MotorThread::getArmOptions(Bottle &b, const int &arm)
{
    if (Bottle *pB=b.find("home_position").asList())
    {
        homePos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homePos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("home_orientation").asList())
    {
        homeOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homeOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_side_displacement").asList())
    {
        reachSideDisp[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachSideDisp[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_orientation").asList())
    {
        reachAboveOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_above_calib_table").asList())
    {
        reachAboveCata[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachAboveCata[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("reach_side_orientation").asList())
    {
        reachSideOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            reachSideOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("deploy_position").asList())
    {
        deployPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            deployPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("deploy_orientation").asList())
    {
        deployOrient[arm].resize(pB->size());
        for (int i=0; i<pB->size(); i++)
            deployOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        deployOrient[arm]=reachAboveOrient[arm];

    if (Bottle *pB=b.find("draw_near_position").asList())
    {
        drawNearPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            drawNearPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("draw_near_orientation").asList())
    {
        drawNearOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            drawNearOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("shift_position").asList())
    {
        shiftPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            shiftPos[arm][i]=pB->get(i).asDouble();
    }
    else
    {
        shiftPos[arm].resize(3);
        shiftPos[arm]=0.0;
    }
    
    if (Bottle *pB=b.find("expect_position").asList())
    {
        expectPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            expectPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("expect_orientation").asList())
    {
        expectOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            expectOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("tool_take_position").asList())
    {
        takeToolPos[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            takeToolPos[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    if (Bottle *pB=b.find("tool_take_orientation").asList())
    {
        takeToolOrient[arm].resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            takeToolOrient[arm][i]=pB->get(i).asDouble();
    }
    else
        return false;

    extForceThresh[arm]=b.check("external_forces_thresh",Value(0.0)).asDouble();

    if(b.check("grasp_model_file"))
    {
        string tmpGraspPath=rf.getHomeContextPath().c_str();
        graspPath[arm]=tmpGraspPath+"/"+b.find("grasp_model_file").asString().c_str();
    }

    return true;
}


void MotorThread::close()
{
    if (closed)
        return;

    //set the system back to velocity mode
    setImpedance(false);
    
    if(drv_head!=NULL)
    {
        delete drv_head;
        drv_head=NULL;
    }

    if(drv_torso!=NULL)
    {
        delete drv_torso;
        drv_torso=NULL;
    }

    if(drv_arm[LEFT]!=NULL)
    {
        delete drv_arm[LEFT];
        drv_arm[LEFT]=NULL;
    }
        
    if(drv_arm[RIGHT]!=NULL)
    {
        delete drv_arm[RIGHT];
        drv_arm[RIGHT]=NULL;
    }

    if(drv_ctrl_gaze!=NULL)
    {
        if(ctrl_gaze!=NULL)
            ctrl_gaze->restoreContext(initial_gaze_context);
        delete drv_ctrl_gaze;
        drv_ctrl_gaze=NULL;
    }

    if (action[LEFT]!=NULL)
    {
        delete action[LEFT];
        action[LEFT]=NULL;
    }

    if (action[RIGHT]!=NULL)
    {
        delete action[RIGHT];
        action[RIGHT]=NULL;
    }

    disparityPort.interrupt();
    disparityPort.close();

    wbdPort.interrupt();
    wbdPort.close();

    wrenchPort[LEFT].interrupt();
    wrenchPort[RIGHT].interrupt();
    wrenchPort[LEFT].close();
    wrenchPort[RIGHT].close();

    closed=true;
}


bool MotorThread::threadInit()
{
    Bottle bMotor=rf.findGroup("motor");

    if(bMotor.isNull())
    {
        fprintf(stdout,"Motor part is missing!\n");
        return false;
    }

    string name=rf.find("name").asString().c_str();
    string robot=bMotor.check("robot",Value("icub")).asString().c_str();
    string partUsed=bMotor.check("part_used",Value("both_arms")).asString().c_str();
    setRate(bMotor.check("thread_period",Value(100)).asInt());

    actions_path=bMotor.check("actions",Value("actions")).asString().c_str();

    double eyesTrajTime=bMotor.check("eyes_traj_time",Value(1.0)).asDouble();
    double neckTrajTime=bMotor.check("neck_traj_time",Value(2.0)).asDouble();

    double kp=bMotor.check("stereo_kp",Value(0.001)).asDouble();
    double ki=bMotor.check("stereo_ki",Value(0.001)).asDouble();
    double kd=bMotor.check("stereo_kd",Value(0.0)).asDouble();

    stereo_track=bMotor.check("stereo_track",Value("on")).asString()=="on";
    dominant_eye=(bMotor.check("dominant_eye",Value("left")).asString()=="left")?LEFT:RIGHT;

    Bottle *neckPitchRange=bMotor.find("neck_pitch_range").asList();
    Bottle *neckRollRange=bMotor.find("neck_roll_range").asList();

    // open ports
    disparityPort.open(("/"+name+"/disparity:io").c_str());
    wbdPort.open(("/"+name+"/wbd:rpc").c_str());
    wrenchPort[LEFT].open(("/"+name+"/left/wrench:o").c_str());
    wrenchPort[RIGHT].open(("/"+name+"/right/wrench:o").c_str());

    // open controllers
    Property optHead("(device remote_controlboard)");
    Property optLeftArm("(device remote_controlboard)");
    Property optRightArm("(device remote_controlboard)");
    Property optTorso("(device remote_controlboard)");
    Property optctrl_gaze("(device gazecontrollerclient)");

    optHead.put("remote",("/"+robot+"/head").c_str());
    optHead.put("local",("/"+name+"/head").c_str());

    optLeftArm.put("remote",("/"+robot+"/left_arm").c_str());
    optLeftArm.put("local",("/"+name+"/left_arm").c_str());

    optRightArm.put("remote",("/"+robot+"/right_arm").c_str());
    optRightArm.put("local",("/"+name+"/right_arm").c_str());

    optTorso.put("remote",("/"+robot+"/torso").c_str());
    optTorso.put("local",("/"+name+"/torso").c_str());

    optctrl_gaze.put("remote","/iKinGazeCtrl");
    optctrl_gaze.put("local",("/"+name+"/gaze").c_str());

    drv_head=new PolyDriver;
    drv_torso=new PolyDriver;
    drv_ctrl_gaze=new PolyDriver;
    if (!drv_head->open(optHead)             ||
        !drv_torso->open(optTorso)           ||
        !drv_ctrl_gaze->open(optctrl_gaze)       )
    {
        close();
        return false;
    }

    // open views
    drv_head->view(enc_head);
    drv_torso->view(enc_torso);
    drv_torso->view(pos_torso);
    drv_torso->view(vel_torso);
    drv_torso->view(ctrl_mode_torso);
    drv_torso->view(int_mode_torso);
    drv_torso->view(ctrl_impedance_torso);

    if(partUsed=="both_arms" || partUsed=="left_arm")
    {
        drv_arm[LEFT]=new PolyDriver;
        if(!drv_arm[LEFT]->open(optLeftArm))
        {   
            close();
            return false;
        }        

        drv_arm[LEFT]->view(ctrl_mode_arm[LEFT]);
        drv_arm[LEFT]->view(int_mode_arm[LEFT]);
        drv_arm[LEFT]->view(ctrl_impedance_arm[LEFT]);
        drv_arm[LEFT]->view(pos_arm[LEFT]);
        drv_arm[LEFT]->view(enc_arm[LEFT]);

        Vector vels(16),accs(16);
        vels=20.0; accs=6000.0;
        pos_arm[LEFT]->setRefSpeeds(vels.data());
        pos_arm[LEFT]->setRefAccelerations(accs.data());
    }

    if(partUsed=="both_arms" || partUsed=="right_arm")
    {
        drv_arm[RIGHT]=new PolyDriver;
        if(!drv_arm[RIGHT]->open(optRightArm))
        {   
            close();
            return false;
        }        

        drv_arm[RIGHT]->view(ctrl_mode_arm[RIGHT]);
        drv_arm[RIGHT]->view(int_mode_arm[RIGHT]);
        drv_arm[RIGHT]->view(ctrl_impedance_arm[RIGHT]);
        drv_arm[RIGHT]->view(pos_arm[RIGHT]);
        drv_arm[RIGHT]->view(enc_arm[RIGHT]);

        Vector vels(16),accs(16);
        vels=20.0; accs=6000.0;
        pos_arm[RIGHT]->setRefSpeeds(vels.data());
        pos_arm[RIGHT]->setRefAccelerations(accs.data());
    }

    drv_ctrl_gaze->view(ctrl_gaze);

    Vector vels(3),accs(3);
    vels=5.0; accs=6000.0;
    pos_torso->setRefSpeeds(vels.data());
    pos_torso->setRefAccelerations(accs.data());

    // initialize the gaze controller

    //store the current context
    ctrl_gaze->storeContext(&initial_gaze_context);

    ctrl_gaze->setTrackingMode(false);
    ctrl_gaze->setEyesTrajTime(eyesTrajTime);
    ctrl_gaze->setNeckTrajTime(neckTrajTime);


    // set the values for the stereo PID controller
    Bottle stereoOpt;
    Bottle &bKp=stereoOpt.addList();
    bKp.addString("Kp");
    Bottle &bKpVal=bKp.addList();
    bKpVal.addDouble(kp);

    Bottle &bKi=stereoOpt.addList();
    bKi.addString("Ki");
    Bottle &bKiVal=bKi.addList();
    bKiVal.addDouble(ki);

    Bottle &bKd=stereoOpt.addList();
    bKd.addString("Kd");
    Bottle &bKdVal=bKd.addList();
    bKdVal.addDouble(kd);

    Bottle &bTs=stereoOpt.addList();
    bTs.addString("Ts");
    bTs.addDouble(0.05); 

    Bottle &bDominantEye=stereoOpt.addList();
    bDominantEye.addString("dominantEye");
    dominant_eye==LEFT?bDominantEye.addString("left"):bDominantEye.addString("right");

    ctrl_gaze->setStereoOptions(stereoOpt);

    //bind neck pitch and roll;
    if(neckPitchRange!=NULL && neckPitchRange->size()==1)
    {
        double neckPitchBlock=neckPitchRange->get(0).asDouble();
        ctrl_gaze->blockNeckPitch(neckPitchBlock);
    }
    else if(neckPitchRange!=NULL && neckPitchRange->size()>1)
    {
        double neckPitchMin=neckPitchRange->get(0).asDouble();
        double neckPitchMax=neckPitchRange->get(1).asDouble();
        ctrl_gaze->bindNeckPitch(neckPitchMin,neckPitchMax);
    }
    if(neckRollRange!=NULL && neckRollRange->size()==1)
    {
        double neckRollBlock=neckRollRange->get(0).asDouble();
        ctrl_gaze->blockNeckRoll(neckRollBlock);
    }
    else if(neckRollRange!=NULL && neckRollRange->size()>1)
    {
        double neckRollMin=neckRollRange->get(0).asDouble();
        double neckRollMax=neckRollRange->get(1).asDouble();
        ctrl_gaze->bindNeckRoll(neckRollMin,neckRollMax);
    }

    //store the current context and restore the initial one
    ctrl_gaze->storeContext(&default_gaze_context);
    ctrl_gaze->restoreContext(initial_gaze_context);
    gazeUnderControl=false;

    //-------------------------------

    // extract the exploration poses from a .ini
    string exploration_name=bMotor.find("exploration_poses").asString().c_str();
    if(!loadExplorationPoses(exploration_name))
    {
        fprintf(stdout,"Error while loading exploration poses!\n");
        close();
        return false;
    }

    // init the NN
    Property netOptions;
    string net_name=bMotor.find("net").asString().c_str();
    netOptions.fromConfigFile(rf.findFile(net_name.c_str()).c_str());
    if(net.configure(netOptions))
    {
        fprintf(stdout,"Neural network configured successfully\n");
        neuralNetworkAvailable=true;
    }
    else
    {
        fprintf(stdout,"Error while loading neural network!\n");
        neuralNetworkAvailable=false;
    }

    // get the general options
    if(!getGeneralOptions(bMotor))
    {
        fprintf(stdout,"Error extracting general options!\n");
        close();
        return false;
    }

    Bottle bArm[2];
    bArm[LEFT]=rf.findGroup("left_arm");
    bArm[RIGHT]=rf.findGroup("right_arm");

    // parsing general arm config options
    Property option;
    for (int i=1; i<bMotor.size(); i++)
    {
        Bottle *pB=bMotor.get(i).asList();
        if (pB->size()==2)
        {
            if(pB->get(0).asString()=="hand_sequences_file")
            {
                string hand_seq_name=bMotor.find("hand_sequences_file").asString().c_str();
                option.put("hand_sequences_file",rf.findFile(hand_seq_name.c_str()).c_str());
            }
            else
                option.put(pB->get(0).asString().c_str(),pB->get(1));
        }
        else
        {
            fprintf(stdout,"Error: invalid option!\n");
            close();
            return false;
        }
    }

    //torso impedence values
    vector<double> torso_stiffness(0),torso_damping(0);
    Bottle *bImpedanceTorsoStiff=bMotor.find("impedence_torso_stiffness").asList();
    Bottle *bImpedanceTorsoDamp=bMotor.find("impedence_torso_damping").asList();
    
    for(int i=0; i<bImpedanceTorsoStiff->size(); i++)
    {
        torso_stiffness.push_back(bImpedanceTorsoStiff->get(i).asDouble());
        torso_damping.push_back(bImpedanceTorsoDamp->get(i).asDouble());
    }

    for(int i=0; i<bImpedanceTorsoStiff->size(); i++)
        ctrl_impedance_torso->setImpedance(i,torso_stiffness[i],torso_damping[i]);


    //arm impedence values
    vector<double> arm_stiffness(0),arm_damping(0);
    Bottle *bImpedanceArmStiff=bMotor.find("impedence_arm_stiffness").asList();
    Bottle *bImpedanceArmDamp=bMotor.find("impedence_arm_damping").asList();
    
    for(int i=0; i<bImpedanceArmStiff->size(); i++)
    {
        arm_stiffness.push_back(bImpedanceArmStiff->get(i).asDouble());
        arm_damping.push_back(bImpedanceArmDamp->get(i).asDouble());
    }


    option.put("local",name.c_str());

    default_exec_time=option.check("default_exec_time",Value("3.0")).asDouble();
    reachingTimeout=2.0*default_exec_time;

    string arm_name[]={"left_arm","right_arm"};
    for (int arm=0; arm<2; arm++)
    {
        if (partUsed=="both_arms" || (partUsed=="left_arm" && arm==LEFT)
                                  || (partUsed=="right_arm" && arm==RIGHT))
        {
            // parsing arm-dependent config options
            if (bArm[arm].isNull())
            {
                fprintf(stdout,"Missing %s parameter list!\n",arm_name[arm].c_str());
                close();
                return false;
            }
            else if(!getArmOptions(bArm[arm],arm))
            {
                fprintf(stdout,"Error extracting %s options!\n",arm_name[arm].c_str());
                close();
                return false;
            }

            Property option_tmp(option);
            option_tmp.put("part",arm_name[arm].c_str());

            string tmpGraspFile=rf.findFile(bArm[arm].find("grasp_model_file").asString().c_str());
            option_tmp.put("grasp_model_file",tmpGraspFile.c_str());

            fprintf(stdout,"***** Instantiating primitives for %s\n",arm_name[arm].c_str());
            action[arm]=new ActionPrimitivesLayer2(option_tmp);
            if (!action[arm]->isValid())
            {
                close();
                return false;
            }

            action[arm]->getGraspModel(graspModel[arm]);
            action[arm]->setExtForceThres(extForceThresh[arm]);
            action[arm]->enableReachingTimeout(reachingTimeout);

            deque<string> q=action[arm]->getHandSeqList();
            fprintf(stdout,"***** List of available %s hand sequence keys:\n",arm_name[arm].c_str());
            for (size_t i=0; i<q.size(); i++)
            {
                Bottle sequence;
                action[arm]->getHandSequence(q[i],sequence);

                fprintf(stdout,"***** %s:\n",q[i].c_str());
                fprintf(stdout,"%s\n",sequence.toString().c_str());
            }

            ICartesianControl *tmpCtrl;
            action[arm]->getCartesianIF(tmpCtrl);

            double armTargetTol=bMotor.check("arm_target_tol",Value(0.01)).asDouble();
            tmpCtrl->setInTargetTol(armTargetTol);

            double tmpTargetTol;
            tmpCtrl->getInTargetTol(&tmpTargetTol);
            fprintf(stdout,"new arm target tol %g\n",tmpTargetTol);

            // set elbow parameters
            if (Bottle *pB=bArm[arm].find("elbow_height").asList())
            {
                if (pB->size()>=2)
                {
                    double height=pB->get(0).asDouble();
                    double weight=pB->get(1).asDouble();
                    changeElbowHeight(arm,height,weight);
                }
            }

            for(int i=0; i<bImpedanceArmStiff->size(); i++)
                ctrl_impedance_arm[arm]->setImpedance(i,arm_stiffness[i],arm_damping[i]);

            armInUse=arm;
        }
    }

    //set impedance on or off
    status_impedance_on=false;
    bool impedance_from_start=bMotor.check("impedance",Value("off")).asString()=="on";
    setImpedance(impedance_from_start);
    fprintf(stdout,"Impedance set %s\n",(status_impedance_on?"on":"off"));

    //init the kinematics offsets and table height
    string kinematics_file=bMotor.find("kinematics_file").asString().c_str();
    if(!loadKinematicOffsets(rf.findFile(kinematics_file.c_str()).c_str()))
        return false;

    //set the initial stereo2cartesian mode
    int starting_modeS2C=bMotor.check("stereoTocartesian_mode",Value("homography")).asVocab();
    setStereoToCartesianMode(starting_modeS2C);

    // initializer dragger
    dragger.init(rf.findGroup("dragger"),this->getRate());

    this->avoidTable(false);

    grasp_state=GRASP_STATE_IDLE;

    Rand::init();

    
    head_mode=HEAD_MODE_IDLE;
    arm_mode=ARM_MODE_IDLE;

    random_pos_y=bMotor.check("random_pos_y",Value(0.1)).asDouble();

    closed=false;
    interrupted=false;

    this->setWaveing(bMotor.check("waveing",Value("off")).asString()=="on");

    return true;
}


void MotorThread::run()
{
    //check if the system needs to be updated
    update();

    switch(head_mode)
    {
        case(HEAD_MODE_GO_HOME):
        {
            if(!gazeUnderControl)
            {
                gazeUnderControl=true;
            }

            homeFixCartType?ctrl_gaze->lookAtFixationPoint(homeFix):
                            ctrl_gaze->lookAtAbsAngles(homeFix);
            break;
        }

        case(HEAD_MODE_TRACK_HAND):
        {
            if(!gazeUnderControl)
            {
                gazeUnderControl=true;
            }

            Vector x,o;
            action[armInUse]->getPose(x,o);
            ctrl_gaze->lookAtFixationPoint(x);
            break;
        }

        case(HEAD_MODE_TRACK_TEMP):
        {
            Vector stereo=stereo_target.get();
            if(stereo.size()==4)
            {
                if(stereo_track)
                {
                    Vector px[2];
                    px[LEFT].resize(2);
                    px[RIGHT].resize(2);

                    px[LEFT][0]=stereo[0];
                    px[LEFT][1]=stereo[1];
                    px[RIGHT][0]=stereo[2];
                    px[RIGHT][1]=stereo[3];

                    ctrl_gaze->lookAtStereoPixels(px[LEFT],px[RIGHT]);
                }
                else
                {
                    int eye_in_use;
                    if(stereo[2*dominant_eye]==0.0 && stereo[2*dominant_eye+1]==0.0)
                    {
                        if(stereo[2*(1-dominant_eye)]==0.0 && stereo[2*(1-dominant_eye)+1]==0.0)
                            break;

                        eye_in_use=1-dominant_eye;
                    }
                    else
                        eye_in_use=dominant_eye;

                    Vector px(2);
                    px[0]=stereo[2*eye_in_use];
                    px[1]=stereo[2*eye_in_use+1];

                    fprintf(stdout,"px[dominant_eye]=%s\n",px.toString().c_str());
                    ctrl_gaze->lookAtMonoPixel(dominant_eye,px,0.4);
                }
            }
            break;
        }


        case(HEAD_MODE_TRACK_FIX):
        {
            if(!gazeUnderControl)
            {
                gazeUnderControl=true;
                //ctrl_gaze->restoreContext(default_gaze_context);
                //ctrl_gaze->setTrackingMode(true);
                //ctrl_gaze->lookAtFixationPoint(gaze_fix_point);
            }
            break;
        }

        case(HEAD_MODE_LOOK):
        {
            bool done;
            ctrl_gaze->checkMotionDone(&done);
            if(done)
            {
                gazeUnderControl=false;
                ctrl_gaze->restoreContext(initial_gaze_context);
                head_mode=HEAD_MODE_IDLE;
            }
            break;
        }

        default:
            break;
    }

    switch(arm_mode)
    {
        case(ARM_MODE_LEARN_ACTION):
        {
            Vector wrench, force(3);

            // get the wrench at the end-effector
            // and filter out forces under threshold
            action[dragger.arm]->getExtWrench(wrench);
            force[0]=wrench[0];
            force[1]=wrench[1];
            force[2]=wrench[2];

            Vector x(3),o(4);
        
            dragger.ctrl->getPose(x,o);

            fprintf(stdout,"curr arm pos= %s \n",x.toString().c_str());

            if(Time::now()-dragger.t0 > dragger.samplingRate)
            {
                //add the new position to the list of actions
                Bottle &tmp_action=dragger.actions.addList();
                Vector tmp_x=dragger.x0 - x;

                for(size_t i=0; i<tmp_x.size(); i++)
                    tmp_action.addDouble(tmp_x[i]);

                for(size_t i=0; i<o.size(); i++)
                    tmp_action.addDouble(o[i]);

                //here add timestamp to the files
                //tmp_action.addDouble(dragger.t0);//temporary for the data collection

                dragger.t0=Time::now();
            }

            if(norm(force)>dragger.extForceThresh)
                x=x+0.1*(1.0/norm(force))*force;

            //dragger.ctrl->goToPositionSync(x,1.0);

            //dragCtrl->askForPosition(x,xd,od,qd);

            //fprintf(stdout,"desired torso positions = %f\t%f\t%f\n\n",qd[0],qd[1],qd[2]);

            //pos_torso->positionmove(0,qd[0]);
            //pos_torso->positionmove(1,qd[1]);
            //pos_torso->positionmove(2,qd[2]);


            break;
        }

        case ARM_MODE_LEARN_KINOFF:
        {
            if(!dragger.using_impedance)
            {
                Vector wrench, force(3);
                double D=dragger.damping;

                // get the wrench at the end-effector
                // and filter out forces under threshold
                action[dragger.arm]->getExtWrench(wrench);
                force[0]=wrench[0];
                force[1]=wrench[1];
                force[2]=wrench[2];

                fprintf(stdout,"force = %f. thresh = %f\n",norm(force),dragger.extForceThresh);

                if (norm(force)<dragger.extForceThresh)
                    force=0.0;
                else
                    D/=5.0;

                
                fprintf(stdout,"force= %s\n", force.toString().c_str());

                Vector b=(1.0/dragger.inertia)*force;
                fprintf(stdout,"(1.0/dragger.inertia)*force = %s\n", b.toString().c_str());


                Vector c=D*dragger.I->get();
                fprintf(stdout,"D*dragger.I->get() = %s\n",c.toString().c_str());

                Vector a=(1.0/dragger.inertia)*force-D*dragger.I->get();
                Vector zeros4d(4);
                zeros4d=0.0;

                Vector v=dragger.I->integrate(a);

                dragger.ctrl->setTaskVelocities(v,zeros4d);
            }

            break;
        }

        case ARM_MODE_FINE_REACHING:
        {
            bool done;
            action[armInUse]->checkActionsDone(done,false);
            if(done)
            {
                Vector stereoHand=stereo_target.get();
                if(stereoHand.size()==4)
                {
                    //move toward the target point
                    

                }

            }

            break;
        }


        default:
        {
            //if the robot is asked to avoid the table with its arms
            if(avoid_table)
            {
                for(int arm=0; arm<2; arm++)
                {
                    if(action[arm]!=NULL)
                    {
                        Vector x,o;
                        action[arm]->getPose(x,o);
                        
                        Vector head_x,head_o;
                        ctrl_gaze->getHeadPose(head_x,head_o);
                        
                        if(fabs(x[1]-head_x[1])<0.2)
                            x[1]=head_x[1]+sign(x[1]-head_x[1])*0.2;

                        //action[arm]->pushAction(x,o);
                        ICartesianControl           *tmp_ctrl;
                        action[arm]->getCartesianIF(tmp_ctrl);
                        x[2]=avoid_table_height[arm];
                        
                        tmp_ctrl->goToPosition(x);
                    }
                }
            }

            break;
        }
    }
}


void MotorThread::threadRelease()
{
    close();
}


// in case it is implemented....
void MotorThread::onStop()
{
    if(action[LEFT]!=NULL)
        action[LEFT]->syncCheckInterrupt(true);

    if(action[RIGHT]!=NULL)
        action[RIGHT]->syncCheckInterrupt(true);
}


bool MotorThread::preTakeHand(Bottle &options)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if(!targetToCartesian(bTarget,xd))
        return false;

    arm=checkArm(arm,xd);

    bool f;
    action[arm]->pushAction("pretake_hand");
    action[arm]->checkActionsDone(f,true);

    return true;
}


bool MotorThread::goUp(Bottle &options, const double h)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    Vector x,o;
    action[arm]->getPose(x,o);
    x[2]+=h;

    bool f;
    action[arm]->pushAction(x,o);
    action[arm]->checkActionsDone(f,true);

    return true;
}


bool MotorThread::reach(Bottle &options)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if(!targetToCartesian(bTarget,xd))
        return false;

    arm=checkArm(arm,xd);

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        keepFixation(options);
        look(options);
    }

    bool side=checkOptions(options,"side");
    Vector tmpOrient,tmpDisp;

    if(side)
    {
        tmpOrient=reachSideOrient[arm];
        tmpDisp=reachSideDisp[arm];
    }
    else
    {
        tmpOrient=reachAboveCata[arm];
        tmpDisp=reachAboveDisp;
    }
    
    wbdRecalibration();
    action[arm]->enableContactDetection();

    ActionPrimitivesWayPoint wp;
    wp.x=xd+tmpDisp; wp.o=tmpOrient; wp.oEnabled=true;
    deque<ActionPrimitivesWayPoint> wpList;
    wpList.push_back(wp);
    wp.x=xd;
    wpList.push_back(wp);
    action[arm]->enableReachingTimeout(3.0*reachingTimeout);
    action[arm]->pushAction(wpList);

    bool f;
    action[arm]->checkActionsDone(f,true);
    action[arm]->checkContact(f);
    action[arm]->disableContactDetection();

    action[arm]->enableReachingTimeout(reachingTimeout);

    // go up straightaway
    if (f)
    {
        Vector x,o;
        action[arm]->getPose(x,o);

        if(!side)
            action[arm]->pushAction(x+graspAboveRelief,o);

        action[arm]->checkActionsDone(f,true);
    }

    setGraspState(side);

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    return true;
}


bool MotorThread::powerGrasp(Bottle &options)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if (!targetToCartesian(bTarget,xd))
        return false;
    if (xd.length()<7)
        return false;

    arm=checkArm(arm,xd,false);
    if (!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        keepFixation(options);
        look(options);
    }

    Vector approach_data(4,0.0);
    if (options.check("approach"))
    {
        if (Bottle *bApproach=options.find("approach").asList())
        {
            size_t sz=std::min(approach_data.size(),(size_t)bApproach->size()); 
            for (size_t i=0; i<sz; i++)
                approach_data[i]=bApproach->get(i).asDouble();
        }
    }

    Vector x=xd.subVector(0,2);
    Vector o=xd.subVector(3,6);

    Matrix R=axis2dcm(o);
    Vector y=R.getCol(1);
    y[3]=CTRL_DEG2RAD*approach_data[3];

    Vector dx=approach_data[0]*R.getCol(0).subVector(0,2);
    Vector dy=approach_data[1]*R.getCol(1).subVector(0,2);
    Vector dz=approach_data[2]*R.getCol(2).subVector(0,2);

    Vector approach_x=x+dx+dy+dz;
    Vector approach_o=dcm2axis(axis2dcm(y)*R);

    wbdRecalibration();
    action[arm]->enableContactDetection();
    action[arm]->pushAction(approach_x,approach_o,"pregrasp_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    // increase reaching precision
    ICartesianControl *ctrl; double tol;
    action[arm]->getCartesianIF(ctrl);
    ctrl->getInTargetTol(&tol);
    ctrl->setInTargetTol(0.002);

    // give time for precise reaching
    action[arm]->enableReachingTimeout(2.0*reachingTimeout);

    action[arm]->pushAction(x,o);
    action[arm]->checkActionsDone(f,true);
    action[arm]->disableContactDetection();

    ctrl->setInTargetTol(tol);
    action[arm]->enableReachingTimeout(reachingTimeout);

    return grasp(options);
}


bool MotorThread::push(Bottle &options)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if(!targetToCartesian(bTarget,xd))
        return false;

    arm=checkArm(arm,xd);

    xd=xd+pushAboveRelief;

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        keepFixation(options);
        look(options);
    }

    ctrl_gaze->waitMotionDone(0.1,2.0);

    double push_direction=checkOptions(options,"away")?-1.0:1.0;
    Vector tmpDisp=push_direction*reachSideDisp[arm];
    Vector tmpOrient=reachSideOrient[arm];

    action[arm]->pushAction(xd+tmpDisp,tmpOrient);
    bool f;

    action[arm]->checkActionsDone(f,true);

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        keepFixation(options);
        lookAtHand(options);
    }

    wbdRecalibration();
    action[arm]->enableContactDetection();
    
    action[arm]->pushAction(xd-3*push_direction*reachSideDisp[arm],reachSideOrient[arm]);
    action[arm]->checkActionsDone(f,true);
    action[arm]->disableContactDetection();

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    return true;
}


bool MotorThread::point(Bottle &options)
{
    int arm=ARM_MOST_SUITED;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    Bottle *bTarget=options.find("target").asList();

    Vector target,xd;
    if(!targetToCartesian(bTarget,target))
        return false;

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        look(options);
        //keepFixation(options);
        //ctrl_gaze->setTrackingMode(true);
    }

    arm=checkArm(arm,target);

    Vector x,o;
    action[arm]->getPose(x,o);

    // set the new position
    Vector x_o=(target-x);
    x_o/=norm(x_o);
    xd=target-0.15*x_o;

    // set the new orientation
    Vector z_o(3,0.0);
    z_o[2]=(arm==LEFT)?1.0:-1.0;

    Vector y_o=cross(z_o,x_o);
    y_o/=norm(y_o);

    z_o=cross(x_o,y_o);

    Matrix R(3,3);
    R.setCol(0,x_o);
    R.setCol(1,y_o);
    R.setCol(2,z_o);

    Vector od=dcm2axis(R);
    action[arm]->pushAction(xd,od,"pointing_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);
    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        ctrl_gaze->setTrackingMode(false);
    }

    return true;
}


bool MotorThread::look(Bottle &options)
{
    if(checkOptions(options,"hand"))
    {
        setGazeIdle();
        ctrl_gaze->restoreContext(default_gaze_context);
        
        int arm=ARM_IN_USE;
        if(checkOptions(options,"left") || checkOptions(options,"right"))
            arm=checkOptions(options,"left")?LEFT:RIGHT;

        arm=checkArm(arm);
        
        lookAtHand(options);
        
        return true;
    }

    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if(!targetToCartesian(bTarget,xd))
        return false;

    setGazeIdle();
    ctrl_gaze->restoreContext(default_gaze_context);

    if (options.check("block_eyes"))
        ctrl_gaze->blockEyes(options.find("block_eyes").asDouble());

    if(checkOptions(options,"fixate"))
    {
        gaze_fix_point=xd;
        keepFixation(options);
    }
        
    ctrl_gaze->lookAtFixationPoint(xd);

    return true;
}


bool MotorThread::takeTool(Bottle &options)
{
    int arm=ARM_IN_USE;
    if (checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);
    if (!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        lookAtHand(options);
    }

    action[arm]->pushAction(takeToolPos[arm],takeToolOrient[arm],"open_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    double force_thresh;
    action[arm]->getExtForceThres(force_thresh);

    bool contact_detected=false;
    Vector wrench(6);
    double t=Time::now();
    
    while (!contact_detected && (Time::now()-t<5.0))
    {
        action[arm]->getExtWrench(wrench);
        if (norm(wrench)>force_thresh)
            contact_detected=true;

        Time::delay(0.1);
    }

    if (!contact_detected)
        fprintf(stdout,"damn!\n");

    if (!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    return true;
}



bool MotorThread::expect(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        lookAtHand(options);
    }

    action[arm]->pushAction(expectPos[arm],expectOrient[arm],"open_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);
    
    double force_thresh;
    action[arm]->getExtForceThres(force_thresh);
    
    bool contact_detected=false;
    Vector wrench(6);
    double t=Time::now();
    while(!contact_detected && Time::now()-t<5.0)
    {
        action[arm]->getExtWrench(wrench);
        if(norm(wrench)>force_thresh)
            contact_detected=true;
        Time::delay(0.1);
    }    
    
    if(!contact_detected)
        fprintf(stdout,"damn!\n");
    
    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    return true;
}


bool MotorThread::give(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();
        lookAtHand(options);
    }

    action[arm]->pushAction(expectPos[arm],expectOrient[arm]);

    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->pushAction("open_hand");

    wbdRecalibration();
    action[arm]->enableContactDetection();

    bool contact_detected=false;
    double t=Time::now();
    while(!contact_detected && Time::now()-t<5.0)
        action[arm]->checkContact(contact_detected); 

    action[arm]->disableContactDetection();

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    return true;
}

bool MotorThread::clearIt(Bottle &options)
{
    setGazeIdle();
    ctrl_gaze->blockEyes(5.0);
    ctrl_gaze->setSaccadesStatus(0);

    return true;
}

bool MotorThread::grasp(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    action[arm]->pushAction("close_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    return isHolding(options);
}

bool MotorThread::grasp_tool(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    action[arm]->pushAction("close_hand_tool");

    bool f;
    action[arm]->checkActionsDone(f,true);

    return isHolding(options);
}


bool MotorThread::release(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    action[arm]->pushAction("release_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    return true;
}


bool MotorThread::changeElbowHeight(const int arm, const double height, const double weight)
{
    if (action[arm]!=NULL)
    {
        Bottle tweakOptions;
        Bottle &optTask2=tweakOptions.addList();
        optTask2.addString("task_2");
        Bottle &plTask2=optTask2.addList();
        plTask2.addInt(6);
        Bottle &posPart=plTask2.addList();
        posPart.addDouble(0.0);
        posPart.addDouble(0.0);
        posPart.addDouble(height);
        Bottle &weightsPart=plTask2.addList();
        weightsPart.addDouble(0.0);
        weightsPart.addDouble(0.0);
        weightsPart.addDouble(weight);

        ICartesianControl *ctrl;
        action[arm]->getCartesianIF(ctrl);
        return ctrl->tweakSet(tweakOptions);
    }
    else
        return false;
}


void MotorThread::goHomeHelper(ActionPrimitives *action, const Vector &xin, const Vector &oin)
{
    ICartesianControl *ctrl;
    action->getCartesianIF(ctrl);

    int context;
    ctrl->storeContext(&context);

    Vector dof;
    ctrl->getDOF(dof); dof=1.0;
    ctrl->setDOF(dof,dof);

    ctrl->setLimits(0,0.0,0.0);
    ctrl->setLimits(1,0.0,0.0);
    ctrl->setLimits(2,0.0,0.0);

    ctrl->setTrajTime(default_exec_time);

    ctrl->goToPoseSync(xin,oin);
    ctrl->waitMotionDone();

    ctrl->restoreContext(context);
    ctrl->deleteContext(context);
}


bool MotorThread::goHome(Bottle &options)
{
    bool head_home=(checkOptions(options,"head") || checkOptions(options,"gaze"));
    bool arms_home=checkOptions(options,"arms") || checkOptions(options,"arm");
    bool hand_home=checkOptions(options,"fingers") || checkOptions(options,"hands") || checkOptions(options,"hand");

    //if none is specified then assume all are going home
    if(!head_home && !arms_home && !hand_home)
        head_home=arms_home=hand_home=true;

    //workaround
    head_home = head_home && !checkOptions(options,"no_head") && !checkOptions(options,"no_gaze");

    bool left_arm=checkOptions(options,"left") || checkOptions(options,"both");
    bool right_arm=checkOptions(options,"right") || checkOptions(options,"both");

    //if none is specified the assume both arms (or hands) are going home
    if(!left_arm && !right_arm)
        left_arm=right_arm=true;

    bool head_fixing=false;
    ctrl_gaze->getTrackingMode(&head_fixing);

    if(head_home)
    //if(!head_fixing && head_mode!=HEAD_MODE_TRACK_TEMP && head_home)
        head_mode=HEAD_MODE_GO_HOME;

    if(arms_home)
    {
        if(left_arm && action[LEFT]!=NULL)
        {          
            if(hand_home)
                action[LEFT]->pushAction("open_hand");

            goHomeHelper(action[LEFT],homePos[LEFT],homeOrient[LEFT]);
            bool f; action[LEFT]->checkActionsDone(f,true);
        }

        if(right_arm && action[RIGHT]!=NULL)
        {
            if(hand_home)
                action[RIGHT]->pushAction("open_hand");

            goHomeHelper(action[RIGHT],homePos[RIGHT],homeOrient[RIGHT]);
            bool f; action[RIGHT]->checkActionsDone(f,true);
        }

        if(right_arm && action[RIGHT]!=NULL)
        {
            if(waveing)
                action[RIGHT]->enableArmWaving(homePos[RIGHT]);
        }

        if(left_arm && action[LEFT]!=NULL)
        {
            if(waveing)
                action[LEFT]->enableArmWaving(homePos[LEFT]);
        }
    }

    if(hand_home)
    {
        if(left_arm)
            action[LEFT]->pushAction("open_hand");

        if(right_arm)
            action[RIGHT]->pushAction("open_hand");
    }

    if(head_home)
    {
        ctrl_gaze->waitMotionDone(0.1,2.0);

        if(!head_fixing && head_mode!=HEAD_MODE_TRACK_TEMP)
            setGazeIdle();
    }

    return true;
}


bool MotorThread::deploy(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    Vector x,o;
    action[arm]->getPose(x,o);

    Vector deployZone;

    Bottle *bTarget=options.find("target").asList();

    if(!targetToCartesian(bTarget,deployZone))
    {
        deployZone=deployPos[arm];

        if(checkOptions(options,"away"))
        {
            deployZone[0]=-0.15;
            deployZone[1]=(arm==LEFT)?deployZone[1]=-0.35:deployZone[1]=0.35;
        }

        deployZone=deployZone+randomDeployOffset();

        deployZone[2]=table_height;
    }
    else
    {
        arm=checkArm(arm,deployZone);
        if(checkOptions(options,"gently"))
            deployZone[2]=table_height;
    }

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        Vector deployFixZone=deployZone;
        deployFixZone[2]=table_height;
        setGazeIdle();
        keepFixation(options);
        ctrl_gaze->lookAtFixationPoint(deployFixZone);
    }

    Vector tmpOrient=(grasp_state==GRASP_STATE_SIDE?reachSideOrient[arm]:deployOrient[arm]);

    Vector preDeployZone=deployZone;
    preDeployZone[2]+=reachAboveDisp[2];

    // prepare hand for deployment
    action[arm]->pushAction(preDeployZone,tmpOrient);
    bool f;
    action[arm]->checkActionsDone(f,true);

    wbdRecalibration();
    action[arm]->enableContactDetection();

    action[arm]->pushAction(deployZone,tmpOrient);
    action[arm]->checkActionsDone(f,true);

    action[arm]->getPose(x,o);
    action[arm]->disableContactDetection();

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        ctrl_gaze->lookAtFixationPoint(x);

    release(options);

    return true;
}


bool MotorThread::drawNear(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    action[arm]->pushAction(drawNearPos[arm],drawNearOrient[arm]);

    bool f;
    action[arm]->checkActionsDone(f,true);

    return true;
}

bool MotorThread::shiftAndGrasp(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    Vector x,o;
    action[arm]->getPose(x,o);
    x=x+shiftPos[arm];

    action[arm]->pushAction(x,reachAboveOrient[arm],"close_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    return true;
}

bool MotorThread::getHandImagePosition(Bottle &hand_image_pos)
{
    int arm=ARM_IN_USE;
    arm=checkArm(arm);

    Vector x_hand,o_hand,px[2],x_head,o_head;
    action[arm]->getPose(x_hand,o_hand);
    ctrl_gaze->get2DPixel(LEFT,x_hand,px[LEFT]);
    ctrl_gaze->get2DPixel(RIGHT,x_hand,px[RIGHT]);

    ctrl_gaze->getHeadPose(x_head,o_head);

    hand_image_pos.clear();
    hand_image_pos.addDouble(px[LEFT][0]);
    hand_image_pos.addDouble(px[LEFT][1]);
    hand_image_pos.addDouble(px[RIGHT][0]);
    hand_image_pos.addDouble(px[RIGHT][1]);
    hand_image_pos.addDouble(norm(x_head-x_hand));

    return true;
}

bool MotorThread::isHolding(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    bool in_position;
    action[arm]->areFingersInPosition(in_position);

    return !in_position;
}


bool MotorThread::calibTable(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    Vector deployZone=deployPos[arm];
    deployZone=deployZone+randomDeployOffset();

    Vector deployPrepare,deployEnd;
    deployPrepare=deployEnd=deployZone;

    deployPrepare[2]=0.0;
    deployEnd[2]=-0.25;

    bool f=false;

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        //setGazeIdle();

        setGazeIdle();
        ctrl_gaze->restoreContext(default_gaze_context);
        //keepFixation(options);
        ctrl_gaze->lookAtFixationPoint(deployEnd);
    }

    if(isHolding(options))
        action[arm]->pushAction("open_hand");

    wbdRecalibration();
    action[arm]->enableContactDetection();

    ActionPrimitivesWayPoint wp;
    wp.x=deployPrepare; wp.o=reachAboveCata[arm];
    wp.oEnabled=true; wp.duration=1.0;
    deque<ActionPrimitivesWayPoint> wpList;
    wpList.push_back(wp);
    wp.x=deployEnd; wp.duration=3.0;
    wpList.push_back(wp);
    action[arm]->enableReachingTimeout(3.0*reachingTimeout);
    action[arm]->pushAction(wpList);

    action[arm]->checkActionsDone(f,true);
    action[arm]->pushWaitState(1.0);
    action[arm]->disableContactDetection();
    action[arm]->enableReachingTimeout(reachingTimeout);

    bool found;
    action[arm]->checkContact(found);

    if(found)
    {
        Vector x,o;
        action[arm]->getPose(x,o);

        if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
            ctrl_gaze->lookAtFixationPoint(x);

        table_height=x[2];
        table[3]=-table_height;

        bKinOffsets.find("table_height")=Value(table_height);

        //save the table height on file
        saveKinematicOffsets();

        //save the table height also in the object database
        opcPort.setTableHeight(table_height);

        fprintf(stdout,"########## Table height found: %f\n",table_height);
        
        //adjust the table height accordingly to a specified tolerance
        table_height+=table_height_tolerance;
    }
    else
        fprintf(stdout,"########## Table height not found.\n");

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    goHome(options);

    return found;
}


bool MotorThread::calibFingers(Bottle &options)
{
    int currentArm=armInUse;

    int handToCalibrate=-1;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        handToCalibrate=checkOptions(options,"left")?LEFT:RIGHT;

    bool no_head=(checkOptions(options,"no_head"));

    bool calibrated=true;
    for(int arm=0; arm<2; arm++)
    {
        if(action[arm]!=NULL && (handToCalibrate==-1 || handToCalibrate==arm))
        {
            setArmInUse(arm);

            if(!no_head)
                lookAtHand(options);

            Bottle b(arm==LEFT?"left":"right");
            drawNear(b);

            Property prop;
            prop.put("finger","all_parallel");
            graspModel[arm]->calibrate(prop);

            std::ofstream fout;
            fout.open(graspPath[arm].c_str());
            graspModel[arm]->toStream(fout);
            fout.close();

            calibrated=calibrated && graspModel[arm]->isCalibrated();

            bool f;
            action[arm]->pushAction(homePos[arm],homeOrient[arm],"open_hand");
            action[arm]->checkActionsDone(f,true);
        }
    }

    if(!no_head)
    {
        head_mode=HEAD_MODE_GO_HOME;
        ctrl_gaze->waitMotionDone(0.1,1.0);
    }

    setArmInUse(currentArm);

    return calibrated;
}


bool MotorThread::avoidTable(bool avoid)
{
    for(int arm=0; arm<2; arm++)
    {
        if(action[arm]!=NULL)
        {
            action[arm]->setTrackingMode(false);

            Vector x,o;
            action[arm]->getPose(x,o);
            avoid_table_height[arm]=x[2];
        }
    }

    avoid_table=avoid;
    return true;
}



//************************************************************************************************
bool MotorThread::exploreTorso(Bottle &options)
{
    int dbg_cnt=0;
    this->avoidTable(true);
    // avoid torso controlDisp

    Bottle info;
    ctrl_gaze->getInfo(info);
    double head_version=info.check("head_version",Value(1.0)).asDouble();

    ostringstream type;
    type<<(dominant_eye==LEFT?"left":"right");
    if (head_version==2.0)
        type<<"_v2";

    iCubEye iKinTorso=iCubEye(type.str());
    iKinTorso.releaseLink(0);   // pitch
    iKinTorso.releaseLink(1);   // roll
    for (size_t i=2; i<iKinTorso.getN(); i++)
        iKinTorso.blockLink(i,0.0);

    //get the torso initial position
    Vector torso_init_joints(3);
    enc_torso->getEncoders(torso_init_joints.data());

    //initialization for the random walker
    //the iKinTorso needs the {0,2} joints switched
    Vector tmp_joints(2,0.0);
    tmp_joints[0]=CTRL_DEG2RAD*torso_init_joints[2];
    tmp_joints[1]=CTRL_DEG2RAD*torso_init_joints[1];
    iKinTorso.setAng(tmp_joints);
    iKinTorso.setBlockingValue(2,CTRL_DEG2RAD*torso_init_joints[0]);

    Matrix H=iKinTorso.getH(3,true);
    Vector cart_init_pos(3);
    cart_init_pos[0]=H[0][3];
    cart_init_pos[1]=H[1][3];
    cart_init_pos[2]=H[2][3];
    //----------------

    fprintf(stdout,"cart init pos = %s\n",cart_init_pos.toString().c_str());
    
    //fixed "target" position
    Vector fixed_target(3);
    fixed_target[0]=-0.5;
    fixed_target[1]=0.0;
    fixed_target[2]=cart_init_pos[2];

    double walking_time=20.0;
    double step_time=2.0;
    double kp_pos_torso=0.6;

    VectorOf<int> modes(3);
    modes[0]=modes[1]=modes[2]=VOCAB_CM_VELOCITY;
    ctrl_mode_torso->setControlModes(modes.getFirst());

    double init_walking_time=Time::now();

    //random walk!
    while(this->isRunning() && !interrupted && Time::now()-init_walking_time<walking_time)
    {    
        //generate next random step
        Vector random_pos=cart_init_pos;
        if (Rand::scalar(0.0,3.0)>2.0)  // 1/3 => lean forward
            random_pos[0]+=Rand::scalar(-0.1,-0.05);
        else                            // 2/3 => move sideways
        {
            double tmp_rnd=Rand::scalar(-random_pos_y,random_pos_y);
            if ((tmp_rnd>-0.1) && (tmp_rnd<0.1))
                tmp_rnd=0.1*iCub::ctrl::sign(tmp_rnd);

            random_pos[1]+=tmp_rnd;
        }

        random_pos=(norm(cart_init_pos)/norm(random_pos))*random_pos;
        fprintf(stdout,"random pos = %s\n\n",random_pos.toString().c_str());

        //wait to reach that point
        double init_step_time=Time::now();
        while(this->isRunning() && !interrupted && Time::now()-init_step_time<step_time && Time::now()-init_walking_time<walking_time)
        {
            //set the current torso joints to the iKinTorso
            Vector torso_joints(3);
            enc_torso->getEncoders(torso_joints.data());

            Vector tmp_joints(2,0.0);
            tmp_joints[0]=CTRL_DEG2RAD*torso_joints[2];
            tmp_joints[1]=CTRL_DEG2RAD*torso_joints[1];
            iKinTorso.setAng(tmp_joints);
            iKinTorso.setBlockingValue(2,CTRL_DEG2RAD*torso_joints[0]);

            Matrix H=iKinTorso.getH(3,true);
            Vector curr_pos(3);
            curr_pos[0]=H[0][3];
            curr_pos[1]=H[1][3];
            curr_pos[2]=H[2][3];

            Vector x_dot=kp_pos_torso*(random_pos-curr_pos);
            Matrix J=iKinTorso.GeoJacobian(3).submatrix(0,2,0,1);
            Matrix J_pinv=pinv(J,1e-06);
            Vector q_dot=CTRL_RAD2DEG*(J_pinv*x_dot);
            double q_dot_mag=norm(q_dot);
            double q_dot_saturation=15.0;

            if (q_dot_mag<1.0)
            {
                vel_torso->stop();
                break;
            }
            else if (q_dot_mag>q_dot_saturation)
                q_dot=(q_dot_saturation/q_dot_mag)*q_dot;

            // account for specific order
            VectorOf<int> jnts(2);
            jnts[0]=2;
            jnts[1]=1;
            vel_torso->velocityMove(jnts.size(),jnts.getFirst(),q_dot.data());
            Time::delay(0.01);
        }
    }

    //go back to torso initial position
    modes[0]=modes[1]=modes[2]=VOCAB_CM_POSITION;
    ctrl_mode_torso->setControlModes(modes.getFirst());
    pos_torso->positionMove(torso_init_joints.data());
    bool done=false;
    while (isRunning() && !done)
    {
        Time::delay(0.1);
        pos_torso->checkMotionDone(&done);
    }

    this->avoidTable(false);
    return true;
}




//************************************************************************************************
bool MotorThread::exploreHand(Bottle &options)
{
    if(arm_mode!=ARM_MODE_IDLE)
    {
        fprintf(stdout,"Error! The requested arm is busy!\n");
        return false;
    }

    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    if(action[arm]==NULL)
    {
        fprintf(stdout,"Error! requested arm is not working!\n");
        return false;
    }

    //if it was not specified by the user to keep the other hand still, bring it back home
    int other_arm=1-arm;
    if(!checkOptions(options,"keep_other_hand_still") && action[other_arm]!=NULL)
    {
        action[other_arm]->pushAction(homePos[other_arm],homeOrient[other_arm]);

        bool f;
        action[other_arm]->checkActionsDone(f,true);
    }

    //completely disable the arm control
    Bottle bInterrupt("skip");
    suspendLearningModeAction(bInterrupt);
    suspendLearningModeKinOffset(bInterrupt);

    action[arm]->lockActions();
    action[arm]->syncCheckInterrupt(true);
    action[arm]->stopControl();
    //-----------------------------------

    if(!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
    {
        setGazeIdle();

        lookAtHand(options);
    }

    double max_step_time=2.0;

    int nJnts;
    enc_arm[arm]->getAxes(&nJnts);

    VectorOf<int> modes(nJnts);
    for (int i=0; i<nJnts; i++)
        modes[i]=VOCAB_CM_POSITION; 
    ctrl_mode_arm[arm]->setControlModes(modes.getFirst());

    //start exploration
    Vector destination(nJnts);
    for(unsigned int pose_idx=0; pose_idx<handPoses.size(); pose_idx++)
    {
        Vector current_position(nJnts);
        enc_arm[arm]->getEncoders(current_position.data());

        //copy the arm configuration in the destination vector
        for(unsigned int i=0; i<handPoses[pose_idx].size(); i++)
            destination[i]=handPoses[pose_idx][i];

        //do not change the fingers angles
        for(int i=handPoses[pose_idx].size(); i<nJnts; i++)
            destination[i]=current_position[i];

        pos_arm[arm]->positionMove(destination.data());

        double init_step_time=Time::now();
        while(!this->interrupted && Time::now()-init_step_time<max_step_time)
        {
            enc_arm[arm]->getEncoders(current_position.data());

            bool done=true;
            for(size_t i=0; i<handPoses[pose_idx].size(); i++)
                if(fabs(destination[i]-current_position[i])>3.0)
                    done=false;

            if(done)
                break;

            Time::delay(0.05);
        }
    }
    
    if (!checkOptions(options,"no_head") && !checkOptions(options,"no_gaze"))
        setGazeIdle();

    //re-enable the arm control
    if(action[arm]!=NULL)
    {
        action[arm]->unlockActions();
        action[arm]->syncCheckReinstate();
    }
    
    return true;
}


//Action Learning mode
bool MotorThread::startLearningModeAction(Bottle &options)
{
    if(arm_mode!=ARM_MODE_IDLE)
    {
        fprintf(stdout,"Error! The requested arm is busy!\n");
        return false;
    }

    string action_name=options.find("action_name").asString().c_str();

    if(action_name=="")
    {
        fprintf(stdout,"Error! action name not specified!\n");
        return false;
    }

    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    string arm_name=(arm==LEFT?"left":"right");

    string fileName=rf.findFile((actions_path+"/"+arm_name+"/"+action_name+".action").c_str());
    if(!fileName.empty())
    {
        fprintf(stdout,"Error! Action '%s' already learned... stopping\n",action_name.c_str());
        return false;
    }

    dragger.actionName=action_name;
    dragger.actions.clear();


    bool f;
    action[arm]->checkActionsDone(f,true);
    action[arm]->getCartesianIF(dragger.ctrl);

    if(dragger.ctrl==NULL)
    {
        fprintf(stdout,"Error! Could not find the cartesian arm interface!\n");
        return false;
    }

    dragger.arm=arm;

    Vector x(3),o(4);
    dragger.ctrl->getPose(x,o);
    dragger.x0=x;

    if(!setTorque(true,arm))
    {
        fprintf(stdout,"Error! Could not set torque control mode\n");
        return false;
    }

    dragger.t0=Time::now();

    arm_mode=ARM_MODE_LEARN_ACTION;

    return true;
}


bool MotorThread::suspendLearningModeAction(Bottle &options)
{
    if(arm_mode!=ARM_MODE_LEARN_ACTION)
        return false;

    string arm_name=(dragger.arm==LEFT?"left":"right");

    bool success=true;
    bool skip=checkOptions(options,"skip");

    if(!skip)
    {
        if (!actions_path.empty())
        {
            string fileName=rf.getHomeContextPath().c_str();
            fileName+="/"+actions_path+"/"+arm_name+"/"+dragger.actionName+".action";
            ofstream action_fout(fileName.c_str());
            if(!action_fout.is_open())
            {
                fprintf(stdout,"Error! Unable to open file '%s' for action %s\n",(actions_path+"/"+arm_name+"/"+dragger.actionName+".action").c_str(),dragger.actionName.c_str());
                success=false;
            }
            else
                action_fout << dragger.actions.toString();
        }
        else
            success = opcPort.setAction(dragger.actionName, &(dragger.actions));
    }

    // set back again to (impedance) velocity mode
    arm_mode=ARM_MODE_IDLE;
    Time::delay(0.1);

    setTorque(false);

    dragger.ctrl=NULL;

    return success;
}


bool MotorThread::imitateAction(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    string arm_name=arm==LEFT?"left":"right";

    string action_name=options.find("action_name").asString().c_str();

    Bottle actions;
    string fileName=rf.findFile((actions_path+"/"+arm_name+"/"+action_name+".action").c_str());
    if (!fileName.empty())
    {
        ifstream action_fin(fileName.c_str());
        if(!action_fin.is_open())
            return false;

        stringstream strstr;
        strstr << action_fin.rdbuf();

        actions.fromString(strstr.str().c_str());
    }
    else
    {
        if (!opcPort.getAction(action_name, &actions))
            return false;
    }

    ICartesianControl *ctrl;
    action[arm]->getCartesianIF(ctrl);

    double currTrajTime;
    ctrl->getTrajTime(&currTrajTime);
    ctrl->setTrajTime(0.75);
    
    Vector newPos;
    ctrl->getDOF(newPos);
    newPos[0]=0.0;
    newPos[1]=0.0;
    newPos[2]=1.0;
    ctrl->setDOF(newPos,newPos);

    ctrl->setInTargetTol(0.01);

    Vector init_x(3),init_o(4);
    ctrl->getPose(init_x,init_o);

    for(int i=0; i<actions.size(); i++)
    {
        Bottle *b=actions.get(i).asList();
        Vector x(3),o(4);
        for(int j=0; j<3; j++)
            x[j]=b->get(j).asDouble();

        for(int j=0; j<4; j++)
            o[j]=b->get(j+3).asDouble();

        //ctrl->goToPosition(init_x-x);
        ctrl->goToPose(init_x-x,o);

        //Time::delay(dragger.samplingRate);
        Time::delay(0.1);
    }

    ctrl->getDOF(newPos);
    newPos=1.0;
    newPos[1]=0.0;
    ctrl->setDOF(newPos,newPos);

    //tmpCtrl->setInTargetTol(armTargetTol);

    ctrl->setTrajTime(currTrajTime);

    return true;
}


//Action Learning mode
bool MotorThread::startLearningModeKinOffset(Bottle &options)
{
    //check if kinematic offset learning is already going on and if the system is not busy
    if(dragger.ctrl!=NULL || arm_mode!=ARM_MODE_IDLE)
        return false;

    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    dragger.arm=checkArm(arm);

    bool f;
    action[dragger.arm]->checkActionsDone(f,true);

    action[dragger.arm]->getCartesianIF(dragger.ctrl);
    if(dragger.ctrl==NULL)
    {
        fprintf(stdout,"Error! Could not find the arm controller\n");
        return false;
    }

    //if the impedance control is available use it otherwise employ adimttance control
    dragger.using_impedance=setTorque(true,dragger.arm);
    if (!dragger.using_impedance)
        fprintf(stdout,"!!! Impedance control not available. Using admittance control!\n");

    Vector x(3),o(4);
    dragger.ctrl->getPose(x,o);
    dragger.x0=x;

    dragger.t0=Time::now();

    arm_mode=ARM_MODE_LEARN_KINOFF;

    return true;
}


bool MotorThread::suspendLearningModeKinOffset(Bottle &options)
{
    if(arm_mode!=ARM_MODE_LEARN_KINOFF)
        return false;

    bool skip=checkOptions(options,"skip");

    if(!skip)
    {
        //compute the offset
        Vector x(3),o(4);
        dragger.ctrl->getPose(x,o);
        
        if(options.size()>3)
            opcPort.getKinematicOffsets(options.get(3).asString().c_str(),currentKinematicOffset);

        currentKinematicOffset[dragger.arm]=(x-dragger.x0) + currentKinematicOffset[dragger.arm];

        //for homography, the z coordinate does not have offset
        if(modeS2C==S2C_HOMOGRAPHY)
            currentKinematicOffset[dragger.arm][2]=0.0;

        //if no object with specified name is present in the database, then update the default kinematic offsets
        if(options.size()<4 || !opcPort.setKinematicOffset(options.get(3).asString().c_str(),currentKinematicOffset))
        {
            defaultKinematicOffset[dragger.arm]=currentKinematicOffset[dragger.arm];

            //update the offset and save on file
            Bottle bOffset;
            for(int i=0; i<3; i++)
                bOffset.addDouble(defaultKinematicOffset[dragger.arm][i]);

            string arm_name=(dragger.arm==LEFT?"left":"right");
            *bKinOffsets.find(arm_name.c_str()).asList()->find(Vocab::decode(modeS2C).c_str()).asList()=bOffset;

            saveKinematicOffsets();
        }
    }

    arm_mode=ARM_MODE_IDLE;
    Time::delay(0.1);

    dragger.ctrl=NULL;

    //reset the previous control mode
    setTorque(false);

    return true;
}


void MotorThread::getStatus(Bottle &status)
{
    Bottle &gaze=status.addList();
    gaze.addString("gaze");

    if(head_mode!=HEAD_MODE_IDLE)
        gaze.addString("busy");
    else
        gaze.addString("idle");

    Bottle &control_mode=status.addList();
    control_mode.addString("control mode");
    control_mode.addString(status_impedance_on?"impedance":"velocity");

    Bottle &left_arm=status.addList();
    left_arm.addString("left_arm");

    if(action[LEFT]!=NULL)
    {
        bool ongoing;
        action[LEFT]->checkActionsDone(ongoing,false);

        if(ongoing)
            left_arm.addString("busy");
        else
            left_arm.addString("idle");
    }
    else
        left_arm.addString("unavailable");


    Bottle &right_arm=status.addList();
    right_arm.addString("right_arm");

    if(action[RIGHT]!=NULL)
    {
        bool ongoing;
        action[RIGHT]->checkActionsDone(ongoing,false);

        if(ongoing)
            right_arm.addString("busy");
        else
            right_arm.addString("idle");
    }
    else
        right_arm.addString("unavailable");



    Bottle &statusS2C=status.addList();
    statusS2C.addString("[Stereo -> Cartesian] mode: ");

    switch(modeS2C)
    {
        case S2C_HOMOGRAPHY:
        {
            statusS2C.addString("homography");
            break;
        }

        case S2C_DISPARITY:
        {
            statusS2C.addString("disparity");
            break;
        }

        case S2C_NETWORK:
        {
            statusS2C.addString("network");
            break;
        }
    }
}


void MotorThread::update()
{
    if(opcPort.isUpdateNeeded())
    {
        opcPort.getTableHeight(table_height);

        table.resize(4);
        table=0.0;
        table[2]=1.0;
        table[3]=-table_height;
        
        //adjust the table height accordingly to a specified tolerance
        table_height+=table_height_tolerance;
    }
}


void MotorThread::interrupt()
{
    interrupted=true;

    disparityPort.interrupt();

    //if learning is going on
    Bottle bInterrupt("skip");
    suspendLearningModeAction(bInterrupt);
    suspendLearningModeKinOffset(bInterrupt);

    if (ctrl_gaze!=NULL)
    {
        setGazeIdle();
        ctrl_gaze->stopControl();
    }

    if(action[LEFT]!=NULL)
    {
        action[LEFT]->lockActions();
        action[LEFT]->syncCheckInterrupt(true);
        action[LEFT]->stopControl();
    }

    if(action[RIGHT]!=NULL)
    {
        action[RIGHT]->lockActions();
        action[RIGHT]->syncCheckInterrupt(true);
        action[RIGHT]->stopControl();
    }
}


void MotorThread::reinstate()
{
    if(action[LEFT]!=NULL)
    {
        action[LEFT]->unlockActions();
        action[LEFT]->syncCheckReinstate();
    }

    if(action[RIGHT]!=NULL)
    {
        action[RIGHT]->unlockActions();
        action[RIGHT]->syncCheckReinstate();
    }

    disparityPort.resume();

    interrupted=false;
}



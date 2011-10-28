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


#include <yarp/os/Time.h>

#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>

#include <yarp/math/Rand.h>

#include <fstream>
#include <sstream>

#include <iCub/MotorThread.h>

using namespace iCub::perception;



bool MotorThread::checkOptions(Bottle &options, const string &parameter)
{
    bool found=false;
    for(int i=0; i<options.size(); i++)
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


int MotorThread::checkArm(int arm, Vector &xd)
{
    if(arm==ARM_MOST_SUITED && xd.size()>0)
        if(arm==ARM_MOST_SUITED)
            arm=xd[1]<0.0?LEFT:RIGHT;

    if(arm!=ARM_IN_USE && arm!=ARM_MOST_SUITED && action[arm]!=NULL)
        armInUse=arm;

    xd=xd+currentKinematicOffset[armInUse];

    return checkArm(arm);
}

bool MotorThread::isWholeBodyAvailable()
{
    return Network::isConnected("/wholeBodyTorqueObserver/filtered/inertial:o","/wholeBodyTorqueObserver/inertial:i");
}


bool MotorThread::setImpedance(bool turn_on)
{
    bool done=false;

    //if the system is asked to turn on impedance control
    if(turn_on && isWholeBodyAvailable())
    {
        done=true;

        for(int i=0; i<5; i++)
        {
            if(action[LEFT]!=NULL)
                done=done && armCtrlMode[LEFT]->setImpedanceVelocityMode(i);
            if(action[RIGHT]!=NULL)
                done=done && armCtrlMode[RIGHT]->setImpedanceVelocityMode(i);
        }

        done=done && torsoCtrlMode->setVelocityMode(0);
        done=done && torsoCtrlMode->setVelocityMode(2);

        //update the system status
        status_impedance_on=done;
    }

    //if the system is asked to turn off impedance control
    if(!turn_on)
    {
        done=true;

        for(int i=0; i<5; i++)
        {
            if(action[LEFT]!=NULL)
                done=done && armCtrlMode[LEFT]->setVelocityMode(i);
            if(action[RIGHT]!=NULL)
                done=done && armCtrlMode[RIGHT]->setVelocityMode(i);
        }

        for(int i=0; i<3; i++)
            done=done && torsoCtrlMode->setVelocityMode(i);

        status_impedance_on=!done;
    }

    return done;
}



bool MotorThread::setTorque(bool turn_on, int arm)
{
    if(action[arm]==NULL)
        return false;

    bool done=false;

    //if the system is asked to turn on impedance control
    if(turn_on && isWholeBodyAvailable())
    {
        done=true;

        for(int i=0; i<4; i++)
                done=done && armCtrlMode[arm]->setTorqueMode(i);

        done=done && torsoCtrlMode->setTorqueMode(0);
        done=done && torsoCtrlMode->setImpedanceVelocityMode(2);
    }

    //if the system is asked to turn off impedance control
    if(!turn_on)
    {
        done=setImpedance(status_impedance_on);
    }

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

    //set the default current kinematic offsets for the arms
    currentKinematicOffset[LEFT]=defaultKinematicOffset[LEFT];
    currentKinematicOffset[RIGHT]=defaultKinematicOffset[RIGHT];


    // if the tartget's cartesian coordinates was specified, use them.
    if(!found && bTarget!=NULL && bTarget->check("cartesian") && bTarget->find("cartesian").asList()->size()>3)
    {
        Bottle *bCartesian=bTarget->find("cartesian").asList();

        xd.clear();
        for(int i=0; i<xd.size(); i++)
            xd.push_back(bCartesian->get(i+1).asDouble());

        found=true;
    }

    // if an object was specified check for its 3D position associated to the object
    if(!found && bTarget!=NULL &&  bTarget->check("name"))
        if(opcPort.getCartesianPosition(bTarget->find("name").asString().c_str(),xd))
            found=true;

    if(!found &&  bTarget!=NULL &&  bTarget->check("stereo"))
    {
        Bottle *bStereo=bTarget->find("stereo").asList();

        Vector stereo;
        for(int i=0; i<bStereo->size(); i++)
            stereo.push_back(bStereo->get(i).asDouble());

        found=stereoToCartesian(stereo,xd);
    }


    if(found && bTarget!=NULL &&  bTarget->check("name"))
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
        torsoPoses.push_back(v);
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
        gazeCtrl->getHeadPose(eyePos,eyeOrient);
    else
        gazeCtrl->getLeftEyePose(eyePos,eyeOrient);

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

    gazeCtrl->get3DPointOnPlane(eye_in_use,px,table,xd);

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
    head->getEncoders(h.data());

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
    offset[0]=Rand::scalar(-0.1, 0.0);
    offset[1]=Rand::scalar(-0.05,0.05);
    offset[2]=0.0;

    return offset;
}

bool MotorThread::loadKinematicOffsets(string _kinematics_path)
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
    table[3]=-(table_height-table_height_tolerance);


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

    kin_fout << bKinOffsets.toString();

    kin_fout.close();

    return true;
}

bool MotorThread::getGeneralOptions(Bottle &b)
{
    if (Bottle *pB=b.find("home_fixation_point").asList())
    {
        homeFix.resize(pB->size());

        for (int i=0; i<pB->size(); i++)
            homeFix[i]=pB->get(i).asDouble();
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

    if (b.check("external_forces_thresh"))
        extForceThresh[arm]=b.find("external_forces_thresh").asDouble();

    if(b.check("grasp_model_file"))
    {
        string grasp_model_name=b.find("grasp_model_file").asString().c_str();

        string tmpGraspPath=rf.getContextPath().c_str();

        graspPath[arm]=tmpGraspPath+"/"+b.find("grasp_model_file").asString().c_str();
    }

    return true;
}


void MotorThread::close()
{
    //set the system back to velocity mode
    setImpedance(false);

    if(drvHead!=NULL)
        delete drvHead;

    if(drvTorso!=NULL)
        delete drvTorso;

    if(drvArm[LEFT]!=NULL)
        delete drvArm[LEFT];

    if(drvArm[RIGHT]!=NULL)
        delete drvArm[RIGHT];

    if(drvGazeCtrl!=NULL)
    {
        gazeCtrl->restoreContext(initial_gaze_context);
        delete drvGazeCtrl;
    }

    if (action[LEFT]!=NULL)
        delete action[LEFT];

    if (action[RIGHT]!=NULL)
        delete action[RIGHT];

    disparityPort.interrupt();
    disparityPort.close();
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

    actions_path=rf.findPath("actions");

    //fprintf(stdout,"path is %s \n",actions_path.c_str() );
    
    double eyesTrajTime=bMotor.check("eyes_traj_time",Value(1.0)).asDouble();
    double neckTrajTime=bMotor.check("neck_traj_time",Value(2.0)).asDouble();

    double kp=bMotor.check("stereo_kp",Value(0.001)).asDouble();
    double ki=bMotor.check("stereo_ki",Value(0.001)).asDouble();
    double kd=bMotor.check("stereo_kd",Value(0.0)).asDouble();

    stereo_track=bMotor.check("stereo_track",Value("on")).asString()=="on";
    dominant_eye=(bMotor.check("dominant_eye",Value("left")).asString()=="left")?LEFT:RIGHT;

    Bottle *neckPitchRange=bMotor.find("neck_pitch_range").asList();
    Bottle *neckRollRange=bMotor.find("neck_roll_range").asList();

    waving=bMotor.check("waving",Value("on")).asString()=="on";

    //open ports
    disparityPort.open(("/"+name+"/disparity:io").c_str());

    wrenchPort[LEFT].open(("/"+name+"/left/wrench:o").c_str());
    wrenchPort[RIGHT].open(("/"+name+"/right/wrench:o").c_str());

    // open controllers
    Property optHead("(device remote_controlboard)");
    Property optLeftArm("(device remote_controlboard)");
    Property optRightArm("(device remote_controlboard)");
    Property optTorso("(device remote_controlboard)");
    Property optGazeCtrl("(device gazecontrollerclient)");

    optHead.put("remote",("/"+robot+"/head").c_str());
    optHead.put("local",("/"+name+"/head").c_str());

    optLeftArm.put("remote",("/"+robot+"/left_arm").c_str());
    optLeftArm.put("local",("/"+name+"/left_arm").c_str());

    optRightArm.put("remote",("/"+robot+"/right_arm").c_str());
    optRightArm.put("local",("/"+name+"/right_arm").c_str());

    optTorso.put("remote",("/"+robot+"/torso").c_str());
    optTorso.put("local",("/"+name+"/torso").c_str());

    optGazeCtrl.put("remote","/iKinGazeCtrl");
    optGazeCtrl.put("local",("/"+name+"/gaze").c_str());

    drvHead=new PolyDriver;
    drvTorso=new PolyDriver;
    drvGazeCtrl=new PolyDriver;
    if (!drvHead->open(optHead)             ||
        !drvTorso->open(optTorso)           ||
        !drvGazeCtrl->open(optGazeCtrl)       )
    {
        close();
        return false;
    }

    // open views
    drvHead->view(head);
    drvTorso->view(torsoPos);
    drvTorso->view(torsoCtrlMode);
    drvTorso->view(torsoImpedenceCtrl);



    if(partUsed=="both_arms" || partUsed=="left_arm")
    {
        drvArm[LEFT]=new PolyDriver;
        if(!drvArm[LEFT]->open(optLeftArm))
        {   
            close();
            return false;
        }        

        drvArm[LEFT]->view(armCtrlMode[LEFT]);
        drvArm[LEFT]->view(armImpedenceCtrl[LEFT]);
    }

    if(partUsed=="both_arms" || partUsed=="right_arm")
    {
        drvArm[RIGHT]=new PolyDriver;
        if(!drvArm[RIGHT]->open(optRightArm))
        {   
            close();
            return false;
        }        

        drvArm[RIGHT]->view(armCtrlMode[RIGHT]);
        drvArm[RIGHT]->view(armImpedenceCtrl[RIGHT]);
    }

    drvGazeCtrl->view(gazeCtrl);

    Vector vels(3),accs(3);
    vels=5.0; accs=6000.0;
    torsoPos->setRefSpeeds(vels.data());
    torsoPos->setRefAccelerations(accs.data());


    // initialize the gaze controller

    //store the current context
    gazeCtrl->storeContext(&initial_gaze_context);

    gazeCtrl->setTrackingMode(false);
    gazeCtrl->setEyesTrajTime(eyesTrajTime);
    gazeCtrl->setNeckTrajTime(neckTrajTime);


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

    gazeCtrl->setStereoOptions(stereoOpt);

    //bind neck pitch and roll;
    if(neckPitchRange->size()==1)
    {
        double neckPitchBlock=neckPitchRange->get(0).asDouble();
        gazeCtrl->blockNeckPitch(neckPitchBlock);
    }
    else if(neckPitchRange->size()>1)
    {
        double neckPitchMin=neckPitchRange->get(0).asDouble();
        double neckPitchMax=neckPitchRange->get(1).asDouble();
        gazeCtrl->bindNeckPitch(neckPitchMin,neckPitchMax);
    }
    if(neckRollRange->size()==1)
    {
        double neckRollBlock=neckRollRange->get(0).asDouble();
        gazeCtrl->blockNeckRoll(neckRollBlock);
    }
    else if(neckRollRange->size()>1)
    {
        double neckRollMin=neckRollRange->get(0).asDouble();
        double neckRollMax=neckRollRange->get(1).asDouble();
        gazeCtrl->bindNeckRoll(neckRollMin,neckRollMax);
    }

    //store the current context and restore the initial one
    gazeCtrl->storeContext(&default_gaze_context);
    gazeCtrl->restoreContext(initial_gaze_context);
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
        fprintf(stdout,"\n Network:\n");
        net.printStructure();

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
        torsoImpedenceCtrl->setImpedance(i,torso_stiffness[i],torso_damping[i]);


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

    double reachingTimeout=2.0*option.check("default_exec_time",Value("3.0")).asDouble();

    string arm_name[]={"left_arm","right_arm"};

    for(int arm=0; arm<2; arm++)
    {
        if (partUsed=="both_arms" || (partUsed=="left_arm" && arm==LEFT)
                                  || (partUsed=="right_arm" && arm==RIGHT))
        {
            // parsing left_arm config options
            

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

            string grasp_model_name=bArm[arm].find("grasp_model_file").asString().c_str();
            //option_tmp.put("grasp_model_file",rf.findFile(grasp_model_name.c_str()).c_str());

            string tmpGraspPath=rf.getContextPath().c_str();

            option_tmp.put("grasp_model_file",(tmpGraspPath+"/"+bArm[arm].find("grasp_model_file").asString().c_str()).c_str());


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

            fprintf(stdout,"new arm target tol%f\n",tmpTargetTol);

            armInUse=arm;

            for(int i=0; i<bImpedanceArmStiff->size(); i++)
                armImpedenceCtrl[arm]->setImpedance(i,arm_stiffness[i],arm_damping[i]);
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

    grasp_state=GRASP_STATE_IDLE;

    Rand::init();

    head_mode=HEAD_MODE_IDLE;
    arm_mode=ARM_MODE_IDLE;

    return true;
}

void MotorThread::run()
{
    switch(head_mode)
    {
        case(HEAD_MODE_GO_HOME):
        {
            if(!gazeUnderControl)
            {
                gazeCtrl->restoreContext(default_gaze_context);
                gazeUnderControl=true;
            }

            gazeCtrl->lookAtFixationPoint(homeFix);
            break;
        }

        case(HEAD_MODE_TRACK_HAND):
        {
            if(!gazeUnderControl)
            {
                gazeUnderControl=true;
                gazeCtrl->restoreContext(default_gaze_context);
            }


            Vector x,o;
            action[armInUse]->getPose(x,o);
            gazeCtrl->lookAtFixationPoint(x);
            break;
        }

        case(HEAD_MODE_TRACK_TEMP):
        {
            if(!gazeUnderControl)
            {
                gazeCtrl->restoreContext(default_gaze_context);
                gazeUnderControl=true;
            }

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

                    gazeCtrl->lookAtStereoPixels(px[LEFT],px[RIGHT]);
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
                    gazeCtrl->lookAtMonoPixel(dominant_eye,px,0.4);
                }
            }
            break;
        }


        case(HEAD_MODE_TRACK_FIX):
        {
            if(!gazeUnderControl)
            {
                gazeUnderControl=true;
                gazeCtrl->restoreContext(default_gaze_context);
                gazeCtrl->setTrackingMode(true);
            }
            break;
        }

        case(HEAD_MODE_LOOK):
        {
            bool done;
            gazeCtrl->checkMotionDone(&done);
            if(done)
            {
                gazeUnderControl=false;
                gazeCtrl->restoreContext(initial_gaze_context);
                head_mode=HEAD_MODE_IDLE;
            }
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

                for(int i=0; i<tmp_x.size(); i++)
                    tmp_action.addDouble(tmp_x[i]);

                for(int i=0; i<o.size(); i++)
                    tmp_action.addDouble(o[i]);

                //here add timestamp to the files
                tmp_action.addDouble(dragger.t0);//temporary for the data collection

                dragger.t0=Time::now();
            }

            if(norm(force)>dragger.extForceThresh)
                x=x+0.1*(1.0/norm(force))*force;

            //dragger.ctrl->goToPositionSync(x,1.0);

            //dragCtrl->askForPosition(x,xd,od,qd);

            //fprintf(stdout,"desired torso positions = %f\t%f\t%f\n\n",qd[0],qd[1],qd[2]);

            //torsopos->positionmove(0,qd[0]);
            //torsopos->positionmove(1,qd[1]);
            //torsopos->positionmove(2,qd[2]);


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
            break;
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

    if(!checkOptions(options,"no_head"))
    {
        setGazeIdle();
        keepFixation();
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

    action[arm]->pushAction(xd+tmpDisp,tmpOrient);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();

    action[arm]->pushAction(xd,tmpOrient);



    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->checkContact(f); 

    action[arm]->disableContactDetection();
    if(f)
    {
        Vector x,o;
        action[arm]->getPose(x,o);

        if(side)
            action[arm]->pushAction(x+graspAboveRelief,o);
    
        action[arm]->checkActionsDone(f,true);
    }

    setGraspState(side);

    if(!checkOptions(options,"no_head"))
        setGazeIdle();

    return true;
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

    if(!checkOptions(options,"no_head"))
    {
        setGazeIdle();
        look(options);
        keepFixation();
    }

    gazeCtrl->waitMotionDone(0.1,2.0);

    double push_direction=checkOptions(options,"away")?-1.0:1.0;
    Vector tmpDisp=push_direction*reachSideDisp[arm];
    Vector tmpOrient=reachSideOrient[arm];

    action[arm]->pushAction(xd+tmpDisp,tmpOrient);
    bool f;

    action[arm]->checkActionsDone(f,true);

    if(!checkOptions(options,"no_head"))
    {
        setGazeIdle();
        lookAtHand();
        keepFixation();
    }

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();
    
    action[arm]->pushAction(xd-3*push_direction*reachSideDisp[arm],reachSideOrient[arm]);

    action[arm]->checkActionsDone(f,true);

    action[arm]->disableContactDetection();

    if(!checkOptions(options,"no_head"))
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


    if(!checkOptions(options,"no_head"))
    {
        setGazeIdle();
        keepFixation();
        look(options);
        gazeCtrl->setTrackingMode(true);
    }

    arm=checkArm(arm,target);

    Vector x,o;
    action[arm]->getPose(x,o);

    //set the new position
    Vector d=0.6*(target-x);
    xd=x+d;

    //set the new orientation
    Vector x_o(3),y_o(3),z_o(3);
    x_o=(1/norm(d))*d;
    
    z_o=0.0;
    if(arm==LEFT)
        z_o[2]=1.0;
    else
        z_o[2]=-1.0;

    y_o=cross(z_o,x_o);
    y_o=(1/norm(y_o))*y_o;

    z_o=cross(x_o,y_o);
    z_o=(1/norm(z_o))*z_o;

    Matrix R(3,3);
    R.setCol(0,x_o);
    R.setCol(1,y_o);
    R.setCol(2,z_o);

    Vector od=dcm2axis(R);

    action[arm]->pushAction(xd,od,"pointing_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    if(!checkOptions(options,"no_head"))
    {
        setGazeIdle();
        gazeCtrl->setTrackingMode(false);
    }

    return true;
}

bool MotorThread::look(Bottle &options)
{
    Bottle *bTarget=options.find("target").asList();

    Vector xd;
    if(!targetToCartesian(bTarget,xd))
        return false;

    setGazeIdle();
    gazeUnderControl=true;
    gazeCtrl->restoreContext(default_gaze_context);
    gazeCtrl->lookAtFixationPoint(xd);

    return true;
}

bool MotorThread::grasp(Bottle &options)
{
    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    if(grasp_state==GRASP_STATE_ABOVE)
    {
        Vector x,o;
        action[arm]->getPose(x,o);
        action[arm]->pushAction(x,o,"pregrasp_hand");
        action[arm]->pushAction(x,reachAboveOrient[arm]);
    }
        
    action[arm]->pushAction("close_hand");

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

    action[arm]->pushAction("open_hand");

    bool f;
    action[arm]->checkActionsDone(f,true);

    return true;
}



bool MotorThread::goHome(Bottle &options)
{
    bool head_home=checkOptions(options,"head") || checkOptions(options,"gaze");
    bool arms_home=checkOptions(options,"arms") || checkOptions(options,"arm");
    bool hand_home=checkOptions(options,"fingers") || checkOptions(options,"hands") || checkOptions(options,"hand");

    //if none is specified then assume all are going home
    if(!head_home && !arms_home && !hand_home)
        head_home=arms_home=hand_home=true;

    bool left_arm=checkOptions(options,"left") || checkOptions(options,"both");
    bool right_arm=checkOptions(options,"right") || checkOptions(options,"both");

    //if none is specified the assume both arms (or hands) are going home
    if(!left_arm && !right_arm)
        left_arm=right_arm=true;


    bool head_fixing=false;
    gazeCtrl->getTrackingMode(&head_fixing);

    if(!head_fixing && head_mode!=HEAD_MODE_TRACK_TEMP && head_home)
        head_mode=HEAD_MODE_GO_HOME;

    if(arms_home)
    {
        if(left_arm && action[LEFT]!=NULL)
        {
            bool f;
            action[LEFT]->setTrackingMode(true);
            if(hand_home)
                action[LEFT]->pushAction(homePos[LEFT],homeOrient[LEFT],"open_hand");
            else
                action[LEFT]->pushAction(homePos[LEFT],homeOrient[LEFT]);
            action[LEFT]->checkActionsDone(f,true);
        }

        if(right_arm && action[RIGHT]!=NULL)
        {
            bool f;
            action[RIGHT]->setTrackingMode(true);
            if(hand_home)
                action[RIGHT]->pushAction(homePos[RIGHT],homeOrient[RIGHT],"open_hand");
            else
                action[RIGHT]->pushAction(homePos[RIGHT],homeOrient[RIGHT]);
            action[RIGHT]->checkActionsDone(f,true);
        }


        if(right_arm && action[RIGHT]!=NULL)
        {
            if(waving)
                action[RIGHT]->enableArmWaving(homePos[RIGHT]);
            action[RIGHT]->setTrackingMode(false);
        }

        if(left_arm && action[LEFT]!=NULL)
        {
            if(waving)
                action[LEFT]->enableArmWaving(homePos[LEFT]);
            action[LEFT]->setTrackingMode(false);
        }
    }
    else if(hand_home)
    {
        if(left_arm)
        {
            Bottle b("left");
            release(b);
        }

        if(right_arm)
        {
            Bottle b("right");
            release(b);
        }
    }

    if(head_home)
    {
        gazeCtrl->waitMotionDone(0.1,2.0);

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

        if(!checkOptions(options,"gently"))
            deployZone[2]+=table_height-table_height_tolerance+0.10;
        else
            deployZone[2]=table_height;

    }


    if(!checkOptions(options,"no_head"))
    {
        Vector deployFixZone=deployZone;
        deployFixZone[2]=table_height;
        setGazeIdle();
        keepFixation();
        gazeCtrl->lookAtFixationPoint(deployFixZone);
    }

    Vector tmpOrient=(grasp_state==GRASP_STATE_SIDE?reachSideOrient[arm]:reachAboveCata[arm]);

    Vector preDeployZone=deployZone;
    preDeployZone[2]=x[2];

    // prepare hand for deployment
    action[arm]->pushAction(preDeployZone,tmpOrient);
    bool f;
    action[arm]->checkActionsDone(f,true);

    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();


    action[arm]->pushAction(deployZone,tmpOrient);
    action[arm]->checkActionsDone(f,true);

    action[arm]->getPose(x,o);
    action[arm]->disableContactDetection();

    if(!checkOptions(options,"no_head"))
        gazeCtrl->lookAtFixationPoint(x);

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

    ICartesianControl *ctrl;


    action[arm]->getCartesianIF(ctrl);

    Vector currDOF;
    ctrl->getDOF(currDOF);

    fprintf(stdout,"\n\ncurr DOF: %s\n",currDOF.toString().c_str());



    double min,max;
    ctrl->getLimits(2,&min,&max);

    fprintf(stdout,"limits: [%f %f]\n",min,max);


    Vector deployZone=deployPos[arm];
    deployZone=deployZone+randomDeployOffset();

    Vector deployPrepare,deployEnd;
    deployPrepare=deployEnd=deployZone;

    deployPrepare[2]=0.1;
    deployEnd[2]=-0.2;

    bool f=false;
    setGazeIdle();

    keepFixation();
    gazeCtrl->lookAtFixationPoint(deployEnd);

    if(isHolding(options))
        action[arm]->pushAction("open_hand");

    action[arm]->enableTorsoDof();

    action[arm]->pushAction(deployPrepare,reachAboveCata[arm]);
    action[arm]->checkActionsDone(f,true);


    Vector test,o;
    action[arm]->getPose(test,o);
    fprintf(stdout,"\n\nReach Above Cata: %s\n",reachAboveCata[arm].toString().c_str());
    fprintf(stdout,"Actual orient: %s\n\n",o.toString().c_str());


    action[arm]->latchWrenchOffset();
    action[arm]->enableContactDetection();
    action[arm]->pushWaitState(1.0);

    action[arm]->pushAction(deployEnd,reachAboveCata[arm],3.0);
    action[arm]->checkActionsDone(f,true);
    action[arm]->pushWaitState(2.0);
    action[arm]->disableContactDetection();

    bool found;
    action[arm]->checkContact(found);

    if(found)
    {
        Vector x,o;
        action[arm]->getPose(x,o);

        gazeCtrl->lookAtFixationPoint(x);

        table_height=x[2]+table_height_tolerance;
        table[3]=-x[2];

        bKinOffsets.find("table_height")=Value(table_height);

        //save the table height on file
        saveKinematicOffsets();

        //save the table height also in the object database
        opcPort.setTableHeight(table_height);

        fprintf(stdout,"########## Table height found: %f\n",table_height);
    }
    else
        fprintf(stdout,"########## Table height not found.\n");

    action[arm]->pushAction(deployPrepare,reachAboveOrient[arm]);

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
                lookAtHand();

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
        gazeCtrl->waitMotionDone(0.1,1.0);
    }

    setArmInUse(currentArm);

    return calibrated;
}


void MotorThread::exploreTorso(const double &trial_time)
{
    // avoid torso controlDisp
    if(action[LEFT]!=NULL)
        action[LEFT]->setTrackingMode(false);

    if(action[RIGHT]!=NULL)
        action[RIGHT]->setTrackingMode(false);

    double t0=Time::now();

    fprintf(stdout,"torso\n");

    int i=0;
    while(isRunning() && (Time::now()-t0<trial_time))
    {
        torsoPos->positionMove(torsoPoses[i%torsoPoses.size()].data());
        bool done=false;
        while(isRunning() && !done)
        {
            Time::delay(0.1);
            torsoPos->checkMotionDone(&done);
        }
        i++;
    }

    fprintf(stdout,"torso\n");

    //go back to torso home last pose
    torsoPos->positionMove(torsoPoses.back().data());
    bool done=false;
    while(isRunning() && !done)
    {
        Time::delay(0.1);
        torsoPos->checkMotionDone(&done);
    }


    fprintf(stdout,"torso\n");
}


//Action Learning mode
bool MotorThread::startLearningModeAction(Bottle &options)
{
    if(arm_mode!=ARM_MODE_IDLE)
    {
        fprintf(stdout,"Error! The requested arm is busy!\n");
        return false;
    }

    if(!isWholeBodyAvailable()){
        fprintf(stdout,"Error! could not find wholeBodyTorqueObserver!\n");
        return false;
    }

    string action_name=options.find("action_name").asString().c_str();

    if(action_name=="")
    {
        fprintf(stdout,"Error! action name not specified!\n",action_name.c_str());
        return false;
    }

    int arm=ARM_IN_USE;
    if(checkOptions(options,"left") || checkOptions(options,"right"))
        arm=checkOptions(options,"left")?LEFT:RIGHT;

    arm=checkArm(arm);

    string arm_name=(arm==LEFT?"left":"right");

    ifstream action_fin((actions_path+"/"+arm_name+"/"+action_name+".action").c_str());
    if(action_fin.is_open())
    {
        fprintf(stdout,"Error! Action '%s' already learned... stopping\n",action_name.c_str());
        action_fin.close();
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
        ofstream action_fout((actions_path+"/"+arm_name+"/"+dragger.actionName+".action").c_str());

        if(!action_fout.is_open())
        {
            fprintf(stdout,"Error! Unable to open file '%s' for action %s\n",(actions_path+"/"+arm_name+"/"+dragger.actionName+".action").c_str(),dragger.actionName.c_str());
            success=false;
        }
        else
            action_fout << dragger.actions.toString();
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

    ifstream action_fin((actions_path+"/"+arm_name+"/"+action_name+".action").c_str());
    if(!action_fin.is_open())
        return false;

    stringstream strstr;
    strstr << action_fin.rdbuf();

    Bottle actions(strstr.str().c_str());

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
    if(!dragger.using_impedance)
    {
        fprintf(stdout,"!!! Impedance control not available. Using admittance control!\n");
        action[dragger.arm]->latchWrenchOffset();
        action[dragger.arm]->enableTorsoDof();
    }

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

    action[dragger.arm]->disableTorsoDof();

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
        table[3]=-(table_height-table_height_tolerance);
    }
}

void MotorThread::interrupt()
{
    disparityPort.interrupt();

    //if learning is going on
    Bottle bInterrupt("skip");
    suspendLearningModeAction(bInterrupt);
    suspendLearningModeKinOffset(bInterrupt);

    setGazeIdle();
    gazeCtrl->stopControl();

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
}



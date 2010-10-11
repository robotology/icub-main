// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
@ingroup icub_module
\defgroup icub_harmonicPathPlanner harmonicPathPlanner

Provides a harmonic function based collision free gradient

Copyright (C) 2010 RobotCub Consortium
 
Authors: Stephen Hart
 
Date: 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module provides a collision-free harmonic gradient for a map for a robot 
that can follow to a goal
 
\section lib_sec Libraries 
YARP libraries 
OPENCV libraries

\section parameters_sec Parameters
<ul>
<li> --name \e stemName 
<li> --map \e fileName loads the map from a file with the specified name
<li> --nodisplay turns off the display port and corresponding image processing that is need
<li> --trace turns on a red trace of the robot's position on the map
<li> --sor will use SOR relaxation technique for computing Harmonic function.  If not used, uses Gauss-Seidel by default
<li> --threshold \e thresh sets the convergence relaxation threshold
<li> --max \e max_iter sets the max number of iterations to compute if convergence is not met.
<li> --omega \e omega parameter for relaxtion.
<li> --plotScaleFactor \e sf sets the factor to amplify the gradient in the map display (has no effect on actual gradient)
<li> --range \e (x_min y_max) the range of the grid.
<li> --resolution \e (x_res y_res) the resolution of the grid.
<li> --goal \e ((x_goal_1 y_goal_1) ... (x_goal_N y_goal_N)) a list of goal positions
<li> --obstacle \e ((x_obs_1 y_obs_1 radius_1) ... (x_obs_N y_obs_N radius_N)) a list of obstacles and sizes
</ul>

\section portsa_sec Ports Accessed

RPC port
--------

set x_robot y_robot

goal x_goal y_goal

obstacle x_obs y_obs radius

clear

update

save <filename> 

load <filename>


\section portsc_sec Ports Created
- <i> /<stemName>/rpc </i> the RPC port
- <i> /<stemName>/map:o </i> outputs the gradient map

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Linux.

\author Stephen Hart
*/ 
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <stdio.h>
#include <iostream>
#include <string>
#include <yarp/os/Semaphore.h>

#include <highgui.h>
#include <cv.h>

#include "HarmonicFunction.h"

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub;
using namespace std;

class ProcessThread : public RateThread  {
private:

    ResourceFinder &rf;      
    HarmonicFunction *hf;
    string name;
    string mapName;
    double omega;
    double threshold;
    double plotScaleFactor;
    int maxIterations;
    bool useGaussSeidel;
    bool displayMode;
    bool fromMap;
    bool drawTrace;
    bool drawPath;

    BufferedPort< ImageOf<PixelBgr> >  potentialMapPort;
    vector<Vector *> goalList;
    vector<Vector *> obstacleList;
    
    vector<Vector *> positionList;

    yarp::os::Semaphore mutex;

public:
    ProcessThread(ResourceFinder &_rf) : 
        RateThread(100),
        rf(_rf) 
    { 


    }
    
    virtual bool threadInit()
    {

        Vector min(2);
        Vector max(2);
        Vector res(2);
        Vector obs(2);
        Vector goal(2);
        double goalSize;
        double obsSize;

        Value *val;
        Bottle *b;

        min[0] = 0;
        min[1] = 0;

        max[0] = 100;
        max[1] = 100;
        
        res[0] = 2;
        res[1] = 2;

        name=rf.check("name",Value("harmonicPathPlanner")).asString().c_str();

        if(rf.check("map")) {
            fromMap = true;
            mapName=rf.check("map",Value("test.map")).asString().c_str();
        } else {
            fromMap = false;
        }
        omega=rf.check("omega",Value(0.9)).asDouble();
        threshold=rf.check("threshold",Value(0.0001)).asDouble();
        maxIterations=rf.check("max",Value(500)).asDouble();
        plotScaleFactor=rf.check("plotScaleFactor",Value(0.0001)).asDouble();
        if(rf.check("sor")) {
            useGaussSeidel = false;
        } else {
            useGaussSeidel = true;
        }
        if(rf.check("nodisplay")) {
            displayMode = false;
        } else {
            displayMode = true;
        }
        if(rf.check("trace")) {
            drawTrace = true;
        } else {
            drawTrace = false;
        }

        if(rf.check("range",val)) {
            if(val->isList()) {
                b = val->asList();
                if(b->get(0).isList()) {
                    cout << "found range list, size: " << b->size() << endl;
                    if(b->size() != 2) {
                        cout << "please specify range \"((min_x min_y) (max_x max_y))\"" << endl;
                    } else {
                        if(b->get(0).asList()->size() != 2) {
                            cout << "please specify range \"((min_x min_y) (max_x max_y))\"" << endl;
                        } else {
                            min[0] = b->get(0).asList()->get(0).asDouble();
                            min[1] = b->get(0).asList()->get(1).asDouble();                                                        
                        }
                        if(b->get(1).asList()->size() != 2) {
                            cout << "please specify range \"((min_x min_y) (max_x max_y))\"" << endl;
                        } else {
                            max[0] = b->get(1).asList()->get(0).asDouble();
                            max[1] = b->get(1).asList()->get(1).asDouble();                                                        
                        }
                    }
                }
            }
        }

        if(rf.check("resolution",val)) {
            if(val->isList()) {
                b = val->asList();
                if(b->size()!=2) {
                    cout << "please specify resoltion \"(x y))\"" << endl;
                } else {
                    res[0] = b->get(0).asDouble();
                    res[1] = b->get(1).asDouble();
                }
            }
        }

        cout << "Map range ("<<min[0]<<","<<min[1]<<"),("<<max[0]<<","<<max[1]<<"), resolution: ("<<res[0]<<","<<res[1]<<")" << endl;

        // create the Harmonic function
        hf = new HarmonicFunction(min,max,res,omega,maxIterations,threshold,plotScaleFactor,displayMode);
        if(fromMap) {
            hf->loadMap(mapName.c_str());
        }
        cout << hf->getWidth() << "," << hf->getHeight() << endl;        

        // parse the list of goals
        if(rf.check("goal",val)) {
            if(val->isList()) {
                b = val->asList();
                if(b->get(0).isList()) {
                    cout << "found list of goals..." << endl;
                    for(int k=0; k<b->size(); k++) {
                        if(b->get(k).isList()) {
                            goal[0] = b->get(k).asList()->get(0).asDouble();
                            goal[1] = b->get(k).asList()->get(1).asDouble();
                            goalSize = b->get(k).asList()->get(2).asDouble();            
                            cout << "Setting goal to ("<<goal[0]<<","<<goal[1]<<"), size="<<goalSize<<endl; 
                            goalList.push_back(new Vector(3));
                            (*goalList[goalList.size()-1])[0] = goal[0];
                            (*goalList[goalList.size()-1])[1] = goal[1];
                            (*goalList[goalList.size()-1])[2] = goalSize;
                            hf->setGoal(goal);
                        } else {
                            cout << "Something wrong with goal list parsing..." << endl;
                        }
                    }                                                                 
                }  else {
                    cout << "found single goal..." << endl;
                    goal[0] = b->get(0).asDouble();
                    goal[1] = b->get(1).asDouble();
                    goalSize = b->get(2).asDouble();            
                    goalList.push_back(new Vector(3));
                    (*goalList[goalList.size()-1])[0] = goal[0];
                    (*goalList[goalList.size()-1])[1] = goal[1];
                    (*goalList[goalList.size()-1])[2] = goalSize;
                    cout << "Setting goal to ("<<goal[0]<<","<<goal[1]<<"), size="<<goalSize<<endl; 
                    hf->setGoal(goal);                    
                }
            }                            
        } 

        // parse the list of obstacles
        if(rf.check("obstacle",val)) {
            if(val->isList()) {
                b = val->asList();
                if(b->get(0).isList()) {
                    cout << "found list of obstacles..." << endl;
                    for(int k=0; k<b->size(); k++) {
                        if(b->get(k).isList()) {
                            obs[0] = b->get(k).asList()->get(0).asDouble();
                            obs[1] = b->get(k).asList()->get(1).asDouble();
                            obsSize = b->get(k).asList()->get(2).asDouble();            
                            cout << "Setting obstacle ("<<obs[0]<<","<<obs[1]<<"), size="<<obsSize<<endl; 
                            obstacleList.push_back(new Vector(3));
                            (*obstacleList[obstacleList.size()-1])[0] = obs[0];
                            (*obstacleList[obstacleList.size()-1])[1] = obs[1];
                            (*obstacleList[obstacleList.size()-1])[2] = obsSize;
                            hf->setObstacle(obs,obsSize);
                        } else {
                            cout << "Something wrong with obstacle list parsing..." << endl;
                        }
                    }                                                                 
                }  else {
                    cout << "found single obstacle..." << endl;
                    obs[0] = b->get(0).asDouble();
                    obs[1] = b->get(1).asDouble();
                    obsSize = b->get(2).asDouble();            
                    obstacleList.push_back(new Vector(3));
                    (*obstacleList[obstacleList.size()-1])[0] = obs[0];
                    (*obstacleList[obstacleList.size()-1])[1] = obs[1];
                    (*obstacleList[obstacleList.size()-1])[2] = obsSize;
                    cout << "Setting obstacle to ("<<obs[0]<<","<<obs[1]<<"), size="<<obsSize<<endl; 
                    hf->setObstacle(obs,obsSize);                    
                }
            }                            
        } 

        drawPath = false;

        if(displayMode) {
            potentialMapPort.open(("/"+name+"/map:o").c_str());
        }

        if(useGaussSeidel) {
            hf->gaussSeidel();
        } else {
            hf->SOR();
        }

        cout << "finished update..." << endl;
        return true;
    }
    
    void afterStart(bool s)
    {
        if (s) {
            fprintf(stdout,"Process started successfully\n");
            fprintf(stdout,"\n");
            fprintf(stdout,"Using ...\n");
            fprintf(stdout,"name\t\t\t = %s\n",name.c_str());
            if(fromMap) fprintf(stdout,"map\t\t\t = %s\n",mapName.c_str());
            fprintf(stdout,"displayMode\t\t = %d\n",displayMode);
            fprintf(stdout,"omega\t\t\t = %f\n",omega);
            fprintf(stdout,"threshold\t\t = %f\n",threshold);
            fprintf(stdout,"max iterations\t\t = %d\n",maxIterations);
            fprintf(stdout,"plotScaleFactor\t\t = %f\n",plotScaleFactor);
            if(useGaussSeidel) {
                fprintf(stdout,"using Gauss-Seidel Relaxation\n");
            } else {
                fprintf(stdout,"using SOR Relaxation\n");
            }
            fprintf(stdout,"\n");
        }
        else
            fprintf(stdout,"Process did not start\n");
    }
    
    virtual void run()
    {
        //cout << "updating HarmonicPathPlanner module..." << endl;
        Vector robot(2);
        Vector grad(2);

        if(displayMode) {

            mutex.wait();
            // copy input image into output image
            ImageOf<PixelBgr> &potential=potentialMapPort.prepare();
            potential.resize(hf->getWidth(), hf->getHeight());
            potential.zero();

            IplImage* pImg = (IplImage*)potential.getIplImage();
            pImg = cvCloneImage((IplImage*)hf->getPotentialMap());
            potential.wrapIplImage(pImg);
            mutex.post();

            if(drawPath) {

                // get the current robot position
                if(positionList.size()>=1) {
                    robot[0] = (*positionList[positionList.size()-1])[0];
                    robot[1] = (*positionList[positionList.size()-1])[1];

                    // get the gradient
                    grad = hf->computeGradient(robot);
                    hf->computePath(robot, drawTrace);
                    //cout << "Gradient: (" << grad[0] << "," << grad[1] << ")" << endl;
                }

            }
            
            potentialMapPort.write();
        }

    }
    
    virtual void threadRelease()
    {
        if(displayMode) {
            potentialMapPort.close();
        }
        delete hf;
    }

    void interrupt()
    {
        if(displayMode) {
            potentialMapPort.interrupt();
        }
    }

    string getName() { return name; }

    // rpc helper functions
    void saveMap(string s="", int width=0) {
        if(s=="") {
            s = mapName;
        }
        hf->saveMap(s,width);
    }

    void loadMap(string s="") {

        if(s=="") {
            s = mapName;
        }
        hf->loadMap(s);

        // TODO: probably should do some processing to find goals 
        // and obstacles and put them in the local lists

    }
   
    void update() {
        if(useGaussSeidel) {
            hf->gaussSeidel();
        } else {
            hf->SOR();
        }

        if(drawPath) {
            Vector pos(2);
            pos[0] = (*positionList[0])[0];
            pos[1] = (*positionList[0])[1];            
            hf->computeGradient(pos);
        }

    }

    void addGoal(Vector g, double s) {
        goalList.push_back(new Vector(3));
        (*goalList[goalList.size()-1])[0] = g[0];
        (*goalList[goalList.size()-1])[1] = g[1];
        (*goalList[goalList.size()-1])[2] = s;
        cout << "Setting goal to ("<<g[0]<<","<<g[1]<<"), size="<<s<<endl; 
        hf->setGoal(g);
    }

    void addObstacle(Vector obs, double s) {
        obstacleList.push_back(new Vector(3));
        (*obstacleList[obstacleList.size()-1])[0] = obs[0];
        (*obstacleList[obstacleList.size()-1])[1] = obs[1];
        (*obstacleList[obstacleList.size()-1])[2] = s;
        cout << "Setting obstacle to ("<<obs[0]<<","<<obs[1]<<"), size="<<s<<endl; 
        hf->setObstacle(obs,s);                    
    }

    void clearMap() {

        mutex.wait();
        cout << "clearing obstacles..." << endl;
        for(unsigned int i=0; i<obstacleList.size(); i++) {
            delete obstacleList[i];
        }
        obstacleList.clear();
        
        cout << "clearing goals..." << endl;
        for(unsigned int i=0; i<goalList.size(); i++) {
            delete goalList[i];
        }
        goalList.clear();

        cout << "clearing positions..." << endl;
        for(unsigned int i=0; i<positionList.size(); i++) {
            delete positionList[i];
        }
        positionList.clear();
        
        cout << "clearing path..." << endl;
        hf->clearPath();
        mutex.post();

        cout << "re-initializing map..." << endl;
        hf->init();
        update();

    }

    void setPosition(Vector pos) {

        positionList.push_back(new Vector(2));
        (*positionList[positionList.size()-1])[0] = pos[0];
        (*positionList[positionList.size()-1])[1] = pos[1];
        hf->computeGradient(pos);
        drawPath = true;

    }
    
};


class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    Port rpcPort;
    Semaphore mutex;

public:
    ProcessModule() : thr(NULL) { }
    
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        thr=new ProcessThread(rf);
        string rpcPortName;

        if (thr->start()) {
            rpcPortName = "/" + thr->getName() + "/rpc";
            rpcPort.open(rpcPortName.c_str());
            attach(rpcPort);
            return true;
        } else {
            delete thr;    
            return false;
        }     

    }
    
    virtual bool close()
    {
        if (thr)  {
            thr->interrupt();
            thr->stop();
            delete thr;
        }

        rpcPort.interrupt();
        rpcPort.close();
       
        return true;
    }


    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        cout << "Receiving command from rpc port" << endl;
        string mapName;
        int width;
        Vector pos(2);
        double size;
        bool ok = true;

        mutex.wait();
        if (command.size()) {
            switch (command.get(0).asVocab()) {
            case VOCAB4('s','a','v','e'):  {   
                if(command.size() == 1) {
                    thr->saveMap();
                } else if(command.size() == 2) {
                    mapName = command.get(1).asString();
                    thr->saveMap(mapName);
                } else if(command.size() == 3) {
                    mapName = command.get(1).asString();
                    width = command.get(2).asInt();
                    thr->saveMap(mapName,width);
                } else {
                    cout << "Incorrect number of arguments for saving map..." << endl;
                }
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB4('l','o','a','d'): {
                if(command.size() == 1) {
                    thr->loadMap();
                } else if(command.size() == 2) {
                    mapName = command.get(1).asString();
                    thr->loadMap(mapName);
                } else {
                    cout << "Incorrect number of arguments for loading map..." << endl;
                }
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB4('u','p','d','a'): {
                thr->update();
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB4('g','o','a','l'): {
                if(command.size() == 4) {
                    pos[0] = command.get(1).asDouble();                        
                    pos[1] = command.get(2).asDouble();
                    size = command.get(3).asDouble();
                    thr->addGoal(pos,size);
                    thr->update();
                } else if(command.size() == 3) {
                    pos[0] = command.get(1).asDouble();                        
                    pos[1] = command.get(2).asDouble();
                    size = 1;
                    thr->addGoal(pos,size);
                    thr->update();
                } else {
                    cout << "Goal format: (x, y) or (x, y, size)" << endl;
                } 
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB4('o','b','s','t'): {
                if(command.size() == 4) {
                    pos[0] = command.get(1).asDouble();                        
                    pos[1] = command.get(2).asDouble();
                    size = command.get(3).asDouble();
                    thr->addObstacle(pos,size);
                    thr->update();
                } else {
                    cout << "Obstacle format: (x,y,rad)" << endl;
                } 
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB4('c','l','e','a'): {
                thr->clearMap();
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            case VOCAB3('s','e','t'): {
                if(command.size() == 3) {
                    pos[0] = command.get(1).asDouble();                        
                    pos[1] = command.get(2).asDouble();
                    thr->setPosition(pos);
                } else {
                    cout << "Set format: (x,y)" << endl;
                } 
                reply.addVocab(Vocab::encode("ack"));
            }
            break;
            default: 
                ok = false;
                RFModule::respond(command,reply);
                break;
            }

        } else {
            reply.add("command size==0");
            ok = false;
        }
        mutex.post();        

        return ok;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
        return -1;
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("omega","0.9");
    rf.configure("ICUB_ROOT",argc,argv);

    ProcessModule mod;
    
    return mod.runModule(rf);
}






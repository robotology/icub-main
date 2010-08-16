// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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

/**
 * @file saliencyBlobFinderModule.cpp
 * @brief module class implementation of the blob finder module.
 */

#include <iCub/saliencyBlobFinderModule.h>

using namespace std;

const bool NOTIMECONTROL = true;
const int centroidDispacementY = 10;

saliencyBlobFinderModule::saliencyBlobFinderModule() {
    blobFinder=0;
    rateThread=0;

    reinit_flag=false;
    reply=new Bottle();

    contrastLP_flag=false;
    meanColour_flag=false;
    blobCataloged_flag=false;
    foveaBlob_flag=false;
    colorVQ_flag=false;
    maxSaliencyBlob_flag=true;
    blobList_flag=false;
    tagged_flag=false;
    watershed_flag=false;

    timeControl_flag=true;
    filterSpikes_flag=false;
}

void saliencyBlobFinderModule::copyFlags() {
    blobFinder->contrastLP_flag=contrastLP_flag;
    blobFinder->meanColour_flag=meanColour_flag;
    blobFinder->blobCataloged_flag=blobCataloged_flag;
    blobFinder->foveaBlob_flag=foveaBlob_flag;
    blobFinder->colorVQ_flag=colorVQ_flag;
    blobFinder->maxSaliencyBlob_flag=maxSaliencyBlob_flag;
    blobFinder->blobList_flag=blobList_flag;
    blobFinder->tagged_flag=tagged_flag;
    blobFinder->watershed_flag=watershed_flag;
    blobFinder->filterSpikes_flag=filterSpikes_flag;
}


bool saliencyBlobFinderModule::configure(ResourceFinder &rf){
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/blobFinder/icub/left_cam"), 
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the ratethread which will define the period of the processing thread
    */
    rateThread = rf.check("ratethread", Value(33), "processing ratethread (int)").asInt();
    cout << "Module started with the parameter ratethread: " << rateThread << endl;

    /*
    * gets the minBounding area for blob neighbours definition
    */
    minBoundingArea = rf.check("minBoundingArea", Value(225), "minBoundingArea (int)").asInt();

    /*
    * saddlePoint threshold
    */
    minBoundingArea = rf.check("saddleThreshold", Value(10), "saddleThreshold (int)").asInt();

    if (!cmdPort.open(getName())) {
        cout << getName() << ": Unable to open port " << endl;
        return false;
    }

    attach(cmdPort);                  // attach to port

    //initialization of the main thread
    blobFinder=new blobFinderThread(rateThread);

    blobFinder->setName(this->getName().c_str());
    blobFinder->start();
    blobFinder->countSpikes=this->countSpikes;

    copyFlags();
    cout << "waiting for connection of the input port" << endl;
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool saliencyBlobFinderModule::interruptModule() {
    cout << "module interrupted" << endl;
    cmdPort.interrupt();
    blobFinder->interrupt();
    return true;
}


bool saliencyBlobFinderModule::close() {
    cout << "closing command port" << endl;
    cmdPort.close();
    blobFinder->stop();
    return true;
}

void saliencyBlobFinderModule::setOptions(yarp::os::Property opt) {
    //
    ConstString name=opt.find("name").asString();
    if (name != "") {
        cout << "|||  Module named as: " << name.c_str() << endl;
        this->setName(name.c_str());
    }
    int rate=opt.find("rateThread").asInt();
    if (rate != 0) {
        cout << "|||  Module rateThread as: " << rate << endl;
        this->rateThread=rate;
    }
    ConstString value=opt.find("mode").asString();
    if (value != "") {
        cout << "|||  Module operating mode: " << value.c_str() << endl;
        if(value == "MEA") {
            meanColour_flag=true;
            cout << "meancolour image as output selected" << endl;
        }
        if(value == "MAX") {
            maxSaliencyBlob_flag=true;
            cout << "max_saliency image as output selected" << endl;
        }
    }
    value=opt.find("filter").asString();
    if(value != "") {
        cout << "|||  Module filter: " << value.c_str() << endl;
        if (value == "spikes") {
            filterSpikes_flag=true;
            cout << "stimuli filter ON" << endl;
        }
        if (value == "kalman") {
            cout << "kalman filter ON" << endl;
        }
        if (value == "off") {
            filterSpikes_flag=false;
            cout << "all the filters OFF" << endl;
        }
    }
    value=opt.find("timeControl").asString();
    if (value != "") {
        cout << "|||  Module time control flag :" << value.c_str() << endl;
        if (value == "ON") {
            this->timeControl_flag=true;
            cout << "time control ON" << endl;
        }
        
        if (value == "OFF") {
            this->timeControl_flag=false;
            cout << "time control OFF" << endl;
        }
    }
    int numValue=opt.find("xdisp").asInt();
    if (numValue != 0) {
        cout << "|||  Module x disp: " << numValue << endl;
        this->xdisp=numValue;
    }
    numValue=opt.find("ydisp").asInt();
    if (numValue != 0) {
        cout << "|||  Module y disp: " << numValue << endl;
        this->ydisp=numValue;
    }
    numValue=opt.find("countSpikes").asInt();
    if (numValue != 0) {
        cout << "|||  Module countSpikes: " << numValue << endl;
        this->countSpikes=numValue;
    }
    numValue=opt.find("thresholddArea").asInt();
    if (numValue != 0) {
        cout << "|||  Module threshold area: " << numValue << endl;
        this->thresholdArea=numValue;
    }
}

bool saliencyBlobFinderModule::updateModule() {
    return true;
}

void saliencyBlobFinderModule::reinitialise(int weight, int height) {
}

bool saliencyBlobFinderModule::respond(const Bottle &command,Bottle &reply) {
     
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("help");
            reply.addString("\n");
            reply.addString("get fn \t: general get command \n");
            reply.addString("\n");
            reply.addString("set s1 <s> \t: general set command \n");
            reply.addString("\n");
            reply.addString("set mea : plots the meancolour image \n");
            reply.addString("set clp : streams out the contrast LP image");
            reply.addString("set max : streams out the max saliency blob");
            reply.addString("set tag : streams out the image of blobs coloured by the associated tag value");
            reply.addString("\n");
            reply.addString("set kbu : set coefficient k of the bottom-up saliency calculation ");
            reply.addString("set ktd : set coefficient k of the top-down saliency calculation ");
            reply.addString("set Mdb : set maximum dimension allowed for blobs ");
            reply.addString("set mdb : set minimum dimension allowed for blobs");
            reply.addString("set rin : set red intensity value for the target to be sought");
            reply.addString("set gin : set green intensity value for the target to be sought");
            reply.addString("set bin : set blue intensity value for the target to be sought");
            reply.addString("set wax : set minumum bounding area");
            reply.addString("set par : set percentage of the blob dimension considered surrounding area");
            reply.addString("\n");
            reply.addString("set tcon : set the constantTimeGazeControl (ex.: format for iKinGazeCtrl) ");
            reply.addString("set tcen : set the constantTimeCentroidControl (ex.: format for controlGaze2) ");
            reply.addString("\n");
            reply.addString("get kbu : set coefficient k of the bottom-up saliency calculation ");
            reply.addString("get ktd : set coefficient k of the top-down saliency calculation ");
            reply.addString("get Mdb : set maximum dimension allowed for blobs ");
            reply.addString("get mdb : set minimum dimension allowed for blobs");
            reply.addString("get rin : set red intensity value for the target to be sought");
            reply.addString("get gin : set green intensity value for the target to be sought");
            reply.addString("get bin : set blue intensity value for the target to be sought");
            ok = true;
        }
        break;
    case COMMAND_VOCAB_NAME:
        rec = true;
        {
            // check and change filter name to pass on to the next filter
            string fName(command.get(1).asString());
            string subName;
            Bottle subCommand;
            int pos=1;
            //int pos = fName.find_first_of(filter->getFilterName());
            if (pos == 0){
                pos = fName.find_first_of('.');
                if (pos  > -1){ // there is a subfilter name
                    subName = fName.substr(pos + 1, fName.size()-1);
                    subCommand.add(command.get(0));
                    subCommand.add(Value(subName.c_str()));
                }
                for (int i = 2; i < command.size(); i++)
                    subCommand.add(command.get(i));
                //ok = filter->respond(subCommand, reply);
            }
            else{
                cout << "filter name  does not match top filter name" << endl;
                ok = false;
            }
        }
        break;
    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr = command.get(2).asDouble();
            }
                break;
            case COMMAND_VOCAB_MAXSALIENCY:{
                int nb = command.get(2).asInt();
                cout << "most salient blob as output has been selected" << endl;
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->maxSaliencyBlob_flag=true;
                }
                //reply.addString("connection 2");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_CONTRASTLP:{
                cout << "image of the saliency (grayscale)" << endl;
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->contrastLP_flag=true;
                }
                ok =true;
            }
                break;
            case COMMAND_VOCAB_WAT:{
                cout << "image of the watershed" << endl;
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->watershed_flag=true;
                }
                ok =true;
            }
                break;
            case COMMAND_VOCAB_MEANCOLOURS:{
                string s(command.get(2).asString().c_str());
                cout << "image composed by mean colour blobs selected as output" << endl;
                if(0!=blobFinder){
                    //this->blobFinder->resetFlags();
                    this->blobFinder->meanColour_flag=true;
                }
                //reply.addString("connection 1");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TAGGED:{
                //int j = command.get(2).asInt();
                cout << "image of tags(unsigned char) given to blobs" << endl;
                if(0!=blobFinder){
                    blobFinder->resetFlags();
                    blobFinder->tagged_flag=true;
                }
                string s(command.get(3).asString().c_str());
                ok=true;
            }
                break;
            case COMMAND_VOCAB_FOVEA:{
                //int j = command.get(2).asInt();
                cout << "image of the fovea" << endl;
                if(0!=blobFinder){
                    blobFinder->resetFlags();
                    blobFinder->foveaBlob_flag=true;
                }
                string s(command.get(3).asString().c_str());
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TCON:{
                //int j = command.get(2).asInt();
                cout << "constantTimeGazeControl of the output" << endl;
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeGazeControl=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TCEN:{
                //int j = command.get(2).asInt();
                cout << "constantTimeCentroid of the output" << endl;
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeCentroid=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KBU:{
                double w = command.get(2).asDouble();
                cout << "set kbu: " << w << endl;
                if(0!=blobFinder)
                    this->blobFinder->salienceBU=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KTD:{
                double w = command.get(2).asDouble();
                cout << "set ktd: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->salienceTD=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MBA:{
                double w = command.get(2).asDouble();
                cout << "set mBA: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->minBoundingArea=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_PAR:{
                int w = command.get(2).asInt();
                cout << "set PAR: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->salience->pArea=w/100;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_RIN:{
                double w = command.get(2).asDouble();
                cout << "set rin: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->targetRED=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_GIN:{
                double w = command.get(2).asDouble();
                cout << "set gin: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->targetGREEN=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_BIN:{
                double w = command.get(2).asDouble();
                cout << "set bin: " << w << endl;
                if(0!=blobFinder)
                    blobFinder->targetBLUE=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MAXDB:{
                int w = command.get(2).asInt();
                if(0!=blobFinder)
                    blobFinder->maxBLOB=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MINDB:{
                int w = command.get(2).asInt();
                if(0!=blobFinder)
                    blobFinder->minBLOB=w;
                ok=true;
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
     case COMMAND_VOCAB_RUN:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_RGB_PROCESSOR:{
                cout << "RUN RGB" << endl;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_YUV_PROCESSOR:{
                cout << "RUN YUV" << endl;
                ok=true;
            }
                break;
            default:
                cout << "received an unknown request after a _VOCAB_RUN" << endl;
                break;
            }
        }
        break;
        case COMMAND_VOCAB_RSET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_FLT:{
                cout << "reset filter" << endl;
                ok=true;
            }
                break;
            default:
                cout << "received an unknown request after a _VOCAB_RSET" << endl;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr=0.0;
                reply.addDouble(thr);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = 0;
                reply.addInt(nb);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NAME:{
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
             break;                               
            case COMMAND_VOCAB_KBU:{
                if(blobFinder!=0){
                    double nb = blobFinder->salienceBU;
                    reply.addDouble(nb);
                    ok = true;
                }
            }
                break;
            case COMMAND_VOCAB_KTD:{
                if(blobFinder!=0){
                    double nb = blobFinder->salienceTD;
                    reply.addDouble(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_RIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetRED;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_GIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetGREEN;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_BIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetBLUE;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_MAXDB:{
                int nb = blobFinder->maxBLOB;
                reply.addInt(nb);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_MINDB:{
                int nb = blobFinder->minBLOB;
                reply.addInt(nb);
                ok = true;
            }
            break;
            
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 	



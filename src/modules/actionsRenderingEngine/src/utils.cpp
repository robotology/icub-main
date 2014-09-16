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


#include <iCub/utils.h>


#include <iostream>
#include <stdio.h>

//--------------- Object Properties Port ------------------//


bool ObjectPropertiesCollectorPort::getStereoPosition(const string &obj_name, Vector &stereo)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bGet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("name");
    bTempAsk.addString("==");
    bTempAsk.addString(obj_name.c_str());

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bGet.addVocab(Vocab::encode("get"));
    Bottle &bTempGet=bGet.addList().addList();
    bTempGet.addString("id");
    bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    this->write(bGet,bReply);

    if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack"))
        return false;

    if(!bReply.get(1).asList()->check("position_2d_left") && !bReply.get(1).asList()->check("position_2d_right"))
        return false;

    stereo.resize(4);
    stereo=0.0;

    if(bReply.get(1).asList()->check("position_2d_left"))
    {
        Bottle *bStereo=bReply.get(1).asList()->find("position_2d_left").asList();

        stereo[0]=0.5*(bStereo->get(0).asDouble()+bStereo->get(2).asDouble());
        stereo[1]=0.5*(bStereo->get(1).asDouble()+bStereo->get(3).asDouble());
    }

    if(bReply.get(1).asList()->check("position_2d_right"))
    {
        Bottle *bStereo=bReply.get(1).asList()->find("position_2d_right").asList();

        stereo[2]=0.5*(bStereo->get(0).asDouble()+bStereo->get(2).asDouble());
        stereo[3]=0.5*(bStereo->get(1).asDouble()+bStereo->get(3).asDouble());
    }

    return true;
}


bool ObjectPropertiesCollectorPort::getCartesianPosition(const string &obj_name, Vector &x)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bGet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("name");
    bTempAsk.addString("==");
    bTempAsk.addString(obj_name.c_str());

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bGet.addVocab(Vocab::encode("get"));
    Bottle &bTempGet=bGet.addList().addList();
    bTempGet.addString("id");
    bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    this->write(bGet,bReply);

    if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack"))
        return false;


    if(!bReply.get(1).asList()->check("position_3d"))
        return false;

    x.resize(3);

    if(bReply.get(1).asList()->check("position_3d"))
    {
        Bottle *bX=bReply.get(1).asList()->find("position_3d").asList();

        for(int i=0; i<bX->size(); i++)
            x[i]=bX->get(i).asDouble();
    }


    return true;
}



bool ObjectPropertiesCollectorPort::getKinematicOffsets(const string &obj_name, Vector *kinematic_offset)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bGet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("name");
    bTempAsk.addString("==");
    bTempAsk.addString(obj_name.c_str());

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bGet.addVocab(Vocab::encode("get"));
    Bottle &bTempGet=bGet.addList().addList();
    bTempGet.addString("id");
    bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    this->write(bGet,bReply);


    if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack"))
        return false;


    if(bReply.get(1).asList()->check("kinematic_offset_left"))
    {
        kinematic_offset[LEFT].resize(3);
        Bottle *bCartesianOffset=bReply.get(1).asList()->find("kinematic_offset_left").asList();
        for(int i=0; i<bCartesianOffset->size(); i++)
            kinematic_offset[LEFT][i]=bCartesianOffset->get(i).asDouble();
    }

    if(bReply.get(1).asList()->check("kinematic_offset_right"))
    {
        kinematic_offset[RIGHT].resize(3);
        Bottle *bCartesianOffset=bReply.get(1).asList()->find("kinematic_offset_right").asList();
        for(int i=0; i<bCartesianOffset->size(); i++)
            kinematic_offset[RIGHT][i]=bCartesianOffset->get(i).asDouble();
    }

    return true;
}


bool ObjectPropertiesCollectorPort::setKinematicOffset(const string &obj_name, const Vector *kinematic_offset)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bSet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("name");
    bTempAsk.addString("==");
    bTempAsk.addString(obj_name.c_str());

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bSet.addVocab(Vocab::encode("set"));
    Bottle &bTempSet=bSet.addList();

    Bottle &bTempSetId=bTempSet.addList();
    bTempSetId.addString("id");
    bTempSetId.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    //Kinematic offset left
    Bottle &bTempSetKinematicOffsetLeft=bTempSet.addList();
    bTempSetKinematicOffsetLeft.addString("kinematic_offset_left");
    Bottle &bTempSetVectorLeft=bTempSetKinematicOffsetLeft.addList();
    for(size_t i=0; i<kinematic_offset[LEFT].size(); i++)
        bTempSetVectorLeft.addDouble(kinematic_offset[LEFT][i]);

    //Kinematic offset right
    Bottle &bTempSetKinematicOffsetRight=bTempSet.addList();
    bTempSetKinematicOffsetRight.addString("kinematic_offset_right");
    Bottle &bTempSetVectorRight=bTempSetKinematicOffsetRight.addList();
    for(size_t i=0; i<kinematic_offset[RIGHT].size(); i++)
        bTempSetVectorRight.addDouble(kinematic_offset[RIGHT][i]);

    this->write(bSet,bReply);

    return bReply.get(0).asVocab()==Vocab::encode("ack");
}





bool ObjectPropertiesCollectorPort::getTableHeight(double &table_height)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bGet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("entity");
    bTempAsk.addString("==");
    bTempAsk.addString("table");

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bGet.addVocab(Vocab::encode("get"));
    Bottle &bTempGet=bGet.addList().addList();
    bTempGet.addString("id");
    bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    this->write(bGet,bReply);

    if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack"))
        return false;

    if(!bReply.get(1).asList()->check("height"))
        return false;

    table_height=bReply.get(1).asList()->find("height").asDouble();
    return true;
}


bool ObjectPropertiesCollectorPort::setTableHeight(const double table_height)
{
    //if the object property collector port is connected use it to obtain the object 2D position
    if(this->getOutputCount()==0)
        return false;

    //ask for the object's id
    Bottle bAsk,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("entity");
    bTempAsk.addString("==");
    bTempAsk.addString("table");

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false)
        return false;

    //if the table entity has not been created yet
    if(bReply.get(1).asList()->find("id").asList()->size()==0)
    {
        Bottle bAdd;
        bAdd.addVocab(Vocab::encode("add"));
        Bottle &bTempAdd=bAdd.addList();

        Bottle &bEntity=bTempAdd.addList();
        bEntity.addString("entity"); bEntity.addString("table");

        Bottle &bHeight=bTempAdd.addList();
        bHeight.addString("height"); bHeight.addDouble(table_height);

        this->write(bAdd,bReply);
    }
    else
    {
        Bottle bSet;
        bSet.addVocab(Vocab::encode("set"));
        Bottle &bTempSet=bSet.addList();

        Bottle &bTempSetId=bTempSet.addList();
        bTempSetId.addString("id");
        bTempSetId.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

        Bottle &bTableHeight=bTempSet.addList();
        bTableHeight.addString("height");
        bTableHeight.addDouble(table_height);

        this->write(bSet,bReply);
    }

    return bReply.get(0).asVocab()==Vocab::encode("ack");
}


bool ObjectPropertiesCollectorPort::setAction(const string &act_name, const Bottle *trajectory)
{
    if(this->getOutputCount()==0)
        return false;
    // rpc add: should ask to see if the same action already exists?
    Bottle bAdd, bReply;
    bAdd.addVocab(Vocab::encode("add"));
    Bottle &bTempAdd=bAdd.addList();

    Bottle &bEntity=bTempAdd.addList();
    bEntity.addString("entity"); bEntity.addString("action");

    Bottle &bName=bTempAdd.addList();
    bName.addString("name"); bName.addString(act_name.c_str());
    Bottle &bTraj= bTempAdd.addList();
    bTraj.addString("trajectory"); bTraj.addList()=*trajectory;

    this->write(bAdd,bReply);
    return bReply.get(0).asVocab()==Vocab::encode("ack");
};
bool ObjectPropertiesCollectorPort::getAction(const string &act_name, Bottle *trajectory)
{
    if(this->getOutputCount()==0)
        return false;
    //ask for the object's id
    Bottle bAsk,bGet,bReply;
    bAsk.addVocab(Vocab::encode("ask"));
    Bottle &bTempAsk=bAsk.addList().addList();
    bTempAsk.addString("name");
    bTempAsk.addString("==");
    bTempAsk.addString(act_name.c_str());

    this->write(bAsk,bReply);

    if(bReply.size()==0 ||
       bReply.get(0).asVocab()!=Vocab::encode("ack") ||
       bReply.get(1).asList()->check("id")==false ||
       bReply.get(1).asList()->find("id").asList()->size()==0)
        return false;

    bGet.addVocab(Vocab::encode("get"));
    Bottle &bTempGet=bGet.addList().addList();
    bTempGet.addString("id");
    bTempGet.addInt(bReply.get(1).asList()->find("id").asList()->get(0).asInt());

    this->write(bGet,bReply);

    if(bReply.size()==0 || bReply.get(0).asVocab()!=Vocab::encode("ack"))
        return false;

    if(!bReply.get(1).asList()->check("trajectory"))
        return false;
   
    *trajectory =  *(bReply.get(1).asList()->find("trajectory").asList());

    return true;
    
};




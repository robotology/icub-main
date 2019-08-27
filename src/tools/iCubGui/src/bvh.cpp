/*
 * bvh.cpp
 */

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Based on:
 *
 *   Qavimator
 *   Copyright (C) 2006 by Zi Ree   *
 *   Zi Ree @ SecondLife   *
 *   Released under the terms of the GNU GPL v2.0.
 */

#include <iostream>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <qmessagebox.h>

#include "bvh.h"

extern std::string GUI_NAME;

BVH::BVH(ObjectsManager *objManager)
{
    pRoot=NULL;
    mObjectsManager=objManager;

    mAB=new BVHNode**[8];

    for (int p=0; p<8; ++p)
    {
        mAB[p]=new BVHNode*[8];

        for (int l=0; l<8; ++l)
        {
            mAB[p][l]=NULL;
        }
    }
}

bool BVH::Create(yarp::os::ResourceFinder& config)
{
    robot=QString(config.find("robot").asString().c_str());

    bvhChannelName.append("Xposition");
    bvhChannelName.append("Yposition");
    bvhChannelName.append("Zposition");
    bvhChannelName.append("Xrotation");
    bvhChannelName.append("Yrotation");
    bvhChannelName.append("Zrotation");

    // default avatar scale for BVH and AVM files
    dAvatarScale=1.0;

    pRoot=bvhRead(config);
    mObjectsManager->setAddressBook(mAB);

    return pRoot!=NULL;
}

BVH::~ BVH()
{
    portEncBase.interrupt();
    portEncHead.interrupt();
    portEncTorso.interrupt();
    portEncLeftArm.interrupt();
    portEncRightArm.interrupt();
    portEncLeftLeg.interrupt();
    portEncRightLeg.interrupt();

    portEncBase.close();
    portEncHead.close();
    portEncTorso.close();
    portEncLeftArm.close();
    portEncRightArm.close();
    portEncLeftLeg.close();
    portEncRightLeg.close();

    if (pRoot)
    {
        delete pRoot;
        pRoot=NULL;
    }

    Network::fini();

    if (mAB)
    {
        for (int p=0; p<8; ++p)
        {
            if (mAB[p])
            {
                delete [] mAB[p];
                mAB[p]=NULL;
            }
        }

        delete [] mAB;

        mAB=NULL;
    }
}

QString BVH::token()
{
  if (tokenPos>=(int)tokens.size())
  {
    qDebug("BVH::token(): no more tokens at index %d",tokenPos);
    return QString();
  }
  return tokens[tokenPos++];
}

bool BVH::expect_token(const QString& name)
{
  if(name!=token())
  {
    qDebug("BVH::expect_token(): Bad file: %s missing\n", name.toLatin1().data());
    return false;
  }
  return true;
}

BVHNode* BVH::bvhRead(yarp::os::ResourceFinder& config)
{
    QString fileName(config.findPath("geometry").c_str());
    QFile geometryFile(fileName);
    if(!geometryFile.open(QIODevice::ReadOnly))
    {
        QMessageBox::critical(0,QObject::tr("File not found"),QObject::tr("BVH File not found: %1").arg(fileName.toLatin1().data()));
        return NULL;
    }

    inputFile=QString(geometryFile.readAll());
    geometryFile.close();

    tokens=tokenize(inputFile.simplified(),' ');

    tokenPos=0;

    Network::init();

    portEncBase.open((GUI_NAME+"/base:i").c_str());
    portEncTorso.open((GUI_NAME+"/torso:i").c_str());
    portEncHead.open((GUI_NAME+"/head:i").c_str());
    portEncLeftArm.open((GUI_NAME+"/left_arm:i").c_str());
    portEncRightArm.open((GUI_NAME+"/right_arm:i").c_str());
    portEncLeftLeg.open((GUI_NAME+"/left_leg:i").c_str());
    portEncRightLeg.open((GUI_NAME+"/right_leg:i").c_str());

    dEncTorso=dEncHead=dEncLeftArm=dEncRightArm=dEncLeftLeg=dEncRightLeg=0;

    memset(dEncBuffer,0,sizeof(dEncBuffer));
    dEncBuffer[10]=90.0;
    dEncBuffer[26]=90.0;

    nJTorso=3;
    nJHead=6;
    nJLeftArm=16;
    nJRightArm=16;
    nJLeftLeg=6;
    nJRightLeg=6;
    nJBase=6;

    dEncTorso=dEncBuffer;
    dEncHead=dEncTorso+nJTorso;
    dEncLeftArm=dEncHead+nJHead;
    dEncRightArm=dEncLeftArm+nJLeftArm;
    dEncLeftLeg=dEncRightArm+nJRightArm;
    dEncRightLeg=dEncLeftLeg+nJLeftLeg;
    dEncBase=dEncRightLeg+nJRightLeg;

    return bvhReadNode(config);
}

BVHNode* BVH::bvhReadNode(yarp::os::ResourceFinder& config)
{
    QString sType=token();
    if (sType=="}") return NULL;

    // check for node type first

    static const int BVH_ROOT=1,BVH_JOINT=2,BVH_END=3;
    int nType;
    if      (sType=="ROOT")  nType=BVH_ROOT;
    else if (sType=="JOINT") nType=BVH_JOINT;
    else if (sType=="END")   nType=BVH_END;
    else
    {
        qDebug("BVH::bvhReadNode(): Bad animation file: unknown node type: '%s'\n",sType.toLatin1().data());
        return NULL;
    }

    BVHNode *node=NULL;

    QString sName=token();
    partNames << sName;

    expect_token("{");
    iCubMesh *pMesh=0;
    QString tag=token();
    QString ftPortName="";

    int skinPart=0;
    int skinLink=0;

    if (tag=="SKIN")
    {
        QString partName=token();
        if (partName=="torso")
            skinPart=1;
        else if (partName=="head")
            skinPart=2;
        else if (partName=="left_arm")
            skinPart=3;
        else if (partName=="right_arm")
            skinPart=4;
        else if (partName=="left_leg")
            skinPart=5;
        else if (partName=="right_leg")
            skinPart=6;
        skinLink=token().toInt();
        tag=token();
    }

    if (tag=="MESH")
    {
        QString name=token();
        double a=token().toDouble();
        double b=token().toDouble();
        double c=token().toDouble();
        double d=token().toDouble();
        double e=token().toDouble();
        double f=token().toDouble();
        QString aux = QString("covers/%1").arg(name);
        QString file = QString("%1").arg(config.findPath((const char *)(aux.toLatin1().data())).c_str());
        if (file.isEmpty()) file=name;
        printf("\n%s\n\n",file.toLatin1().data());
        pMesh=new iCubMesh(file,a,b,c,d,e,f);
        tag=token();
    }

    if (tag=="FORCE_TORQUE")
    {
        ftPortName=token();
        tag=token();
    }

    switch (nType)
    {
    case BVH_ROOT:
        {
            int id=token().toInt();

            double Px=token().toDouble();
            double Py=token().toDouble();
            double Pz=token().toDouble();

            node=new BVHNodeROOT(sName,id,Px,Py,Pz,pMesh,mObjectsManager);
        }
        break;
    case BVH_JOINT:
        if (tag=="RPY_XYZ")
        {
            double Rz=token().toDouble();
            double Ry=token().toDouble();
            double Rx=token().toDouble();
            double Px=token().toDouble();
            double Py=token().toDouble();
            double Pz=token().toDouble();

            node=new BVHNodeXYZ_RPY(sName,Px,Py,Pz,Rz,Ry,Rx);
        }
        else if (tag=="DH")
        {
            int a=token().toInt();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            double e=token().toDouble();
            if (ftPortName=="")
            {
                node=new BVHNodeDH(sName,a,b,c,d,e,pMesh);
            }
            else
            {
                node=new BVHNodeForceTorque(sName,ftPortName,a,b,c,d,e,pMesh);
            }
        }
        break;
    case BVH_END:
        if (tag=="EYE")
        {
            int n=token().toInt();
            double a=token().toDouble();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            node=new BVHNodeEYE(sName,n,a,b,c,d,pMesh);
        }
        else if (tag=="DH")
        {
            int n=token().toInt();
            double a=token().toDouble();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            node=new BVHNodeEND(sName,n,a,b,c,d,pMesh);
        }
        else if (tag=="LEFTHAND")
        {
            int a=token().toInt();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            double e=token().toDouble();
            node=new BVHNodeLEFTHAND(sName,a,b,c,d,e,pMesh);
        }
        else if (tag=="RIGHTHAND")
        {
            int a=token().toInt();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            double e=token().toDouble();
            node=new BVHNodeRIGHTHAND(sName,a,b,c,d,e,pMesh);
        }
        else if (tag=="INERTIAL")
        {
            double a=token().toDouble();
            double b=token().toDouble();
            double c=token().toDouble();
            double d=token().toDouble();
            node=new BVHNodeINERTIAL(sName,a,b,c,d,robot+"/head/inertials",pMesh);
        }
        break;
    }

    if (skinPart)
    {
        mAB[skinPart][skinLink]=node;
    }

    BVHNode* child=0;

    do
    {
        if((child=bvhReadNode(config)))
        {
            node->addChild(child);
        }
    } while (child);

    return node;
}

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
#include <string.h>

#include <qmessagebox.h>

#include "bvh.h"

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
    portEncHead.interrupt();
    portEncTorso.interrupt();
    portEncLeftArm.interrupt();
    portEncRightArm.interrupt();
    portEncLeftLeg.interrupt();
    portEncRightLeg.interrupt();

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
    qDebug("BVH::expect_token(): Bad file: %s missing\n", name.latin1());
    return false;
  }
  return true;
}

BVHNode* BVH::bvhRead(yarp::os::ResourceFinder& config)
{
    QString fileName(config.findPath("geometry"));
    QFile geometryFile(fileName);
    if(!geometryFile.open(IO_ReadOnly))
    {
        QMessageBox::critical(0,QObject::tr("File not found"),QObject::tr("BVH File not found: %1").arg(fileName.latin1()));
        return NULL;
    }

    inputFile=QString(geometryFile.readAll());    
    geometryFile.close();

    tokens=tokenize(inputFile.simplifyWhiteSpace(),' ');
  
    tokenPos=0;
  
    Network::init();

    portEncTorso.open("/iCubGui/torso:i");
    portEncHead.open("/iCubGui/head:i");
    portEncLeftArm.open("/iCubGui/left_arm:i");
    portEncRightArm.open("/iCubGui/right_arm:i");
    portEncLeftLeg.open("/iCubGui/left_leg:i");
    portEncRightLeg.open("/iCubGui/right_leg:i");

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

    dEncTorso=dEncBuffer;
    dEncHead=dEncTorso+nJTorso;
    dEncLeftArm=dEncHead+nJHead;
    dEncRightArm=dEncLeftArm+nJLeftArm;
    dEncLeftLeg=dEncRightArm+nJRightArm;
    dEncRightLeg=dEncLeftLeg+nJLeftLeg;
    dEncRoot=dEncRightLeg+nJRightLeg;

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
        qDebug("BVH::bvhReadNode(): Bad animation file: unknown node type: '%s'\n",sType.latin1());
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
        if (partName=="head")
            skinPart=1;
        else if (partName=="torso")
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
        QString file(config.findPath(QString("covers/")+name).c_str());
        if (file.isEmpty()) file=name;
        printf("\n%s\n\n",file.latin1());
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
			int a=token().toInt();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
			double e=token().toDouble();
			double f=token().toDouble();
			double g=token().toDouble();
			node=new BVHNodeROOT(sName,a,b,c,d,e,f,g,pMesh,mObjectsManager); 
		}
		break;
    case BVH_JOINT:
        if (tag=="RPY_XYZ")
		{
			double a=token().toDouble();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
			double e=token().toDouble();
			double f=token().toDouble();
            node=new BVHNodeRPY_XYZ(sName,a,b,c,d,e,f);
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
            node=new BVHNodeINERTIAL(sName,a,b,c,d,robot+"/inertial",pMesh);
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

/*
PolyDriver* BVH::OpenDriver(QString part)
{
    Property options;
    if (robot=="/icubSim") 
        options.put("robot","icubSim");
    else
        options.put("robot","icub");
    options.put("device","remote_controlboard");
    options.put("local",(QString("/iCubGui")+part).latin1());
    options.put("remote",(robot+part).latin1());
   
    PolyDriver *pDriver=new PolyDriver(options);
   
    if (!pDriver->isValid()) 
    {
        pDriver->close();
        delete pDriver;
        pDriver=0; 
        //Network::fini();
        //exit(-1);
    }
   
    return pDriver;
}

void BVH::CloseDriver(PolyDriver* &pDriver)
{
    if (pDriver)
    {
        pDriver->close();
        delete pDriver;
        pDriver=0;
    }
}
*/



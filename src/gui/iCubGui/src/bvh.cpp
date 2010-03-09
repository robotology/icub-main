/*
 * bvh.cpp
 */

#include <iostream>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <qmessagebox.h>

#include "bvh.h"

BVH::BVH()
{
    pRoot=NULL;
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

    return pRoot!=NULL;
}

BVH::~ BVH()
{
    // YARP
    #ifdef __YARP
    CloseDriver(pHeadDriver);
    CloseDriver(pTorsoDriver);
    CloseDriver(pLeftArmDriver);
    CloseDriver(pRightArmDriver);
    CloseDriver(pLeftLegDriver);
    CloseDriver(pRightLegDriver);
    #endif
    
    if (pRoot) delete pRoot;

    #ifdef __YARP
    Network::fini();
    #endif
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
  
    // YARP
    #ifdef __YARP
    Network::init();

    yarp::os::Time::delay(1.0);
    pTorsoDriver=OpenDriver("/torso");
    yarp::os::Time::delay(1.0);
    pHeadDriver=OpenDriver("/head");
    yarp::os::Time::delay(1.0);
    pLeftArmDriver=OpenDriver("/left_arm");
    yarp::os::Time::delay(1.0);
    pRightArmDriver=OpenDriver("/right_arm");
    yarp::os::Time::delay(1.0);
    pLeftLegDriver=OpenDriver("/left_leg");
    yarp::os::Time::delay(1.0);
    pRightLegDriver=OpenDriver("/right_leg");
    yarp::os::Time::delay(1.0);
   
    pEncTorso=pEncHead=pEncLeftArm=pEncRightArm=pEncLeftLeg=pEncRightLeg=0;
    dEncTorso=dEncHead=dEncLeftArm=dEncRightArm=dEncLeftLeg=dEncRightLeg=0;

    memset(dEncBuffer,0,sizeof(dEncBuffer));

    if (pTorsoDriver) pTorsoDriver->view(pEncTorso);   
    if (pHeadDriver) pHeadDriver->view(pEncHead);
    if (pLeftArmDriver) pLeftArmDriver->view(pEncLeftArm);
    if (pRightArmDriver) pRightArmDriver->view(pEncRightArm);
    if (pLeftLegDriver) pLeftLegDriver->view(pEncLeftLeg);
    if (pRightLegDriver) pRightLegDriver->view(pEncRightLeg);
   
    if (!pEncTorso || !pEncHead || !pEncLeftArm || !pEncRightArm || !pEncLeftLeg || !pEncRightLeg)
    {
        qDebug("BVH::BVH: error getting IEncoders interfaces");
        dEncBuffer[10]=45.0;
        dEncBuffer[26]=45.0;
    }
    else
    {
        pEncTorso->getAxes(&nJTorso);
        qDebug("BVH::BVH: %d Torso joints found",nJTorso);
        pEncHead->getAxes(&nJHead);
        qDebug("BVH::BVH: %d Head joints found",nJHead);
        pEncLeftArm->getAxes(&nJLeftArm);
        qDebug("BVH::BVH: %d Left Arm joints found",nJLeftArm);
        pEncRightArm->getAxes(&nJRightArm);
        qDebug("BVH::BVH: %d Right Arm joints found",nJRightArm);
        pEncLeftLeg->getAxes(&nJLeftLeg);
        qDebug("BVH::BVH: %d Left Leg joints found",nJLeftLeg);
        pEncRightLeg->getAxes(&nJRightLeg);
        qDebug("BVH::BVH: %d Right Leg joints found",nJRightLeg);
        
        dEncTorso=dEncBuffer;
        dEncHead=dEncTorso+nJTorso;
        dEncLeftArm=dEncHead+nJHead;
        dEncRightArm=dEncLeftArm+nJLeftArm;
        dEncLeftLeg=dEncRightArm+nJRightArm;
        dEncRightLeg=dEncLeftLeg+nJLeftLeg;
        dEncRoot=dEncRightLeg+nJRightLeg;
        dEncLeftArm[1]=dEncRightArm[1]=10.0;
    }
    #endif
    
        dEncTorso=dEncBuffer;
        dEncHead=dEncTorso+3;
        dEncLeftArm=dEncHead+6;
        dEncRightArm=dEncLeftArm+16;
        dEncLeftLeg=dEncRightArm+16;
        dEncRightLeg=dEncLeftLeg+6;
        dEncRoot=dEncRightLeg+6;

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
    int ftSensorId=-1;
    
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
		ftSensorId=token().toInt();
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
			node=new BVHNodeROOT(sName,a,b,c,d,e,f,g,pMesh); 
		}
		break;
    case BVH_JOINT:
        if (tag=="RPY_XYZ")
		{
			int a=token().toInt();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
			double e=token().toDouble();
			double f=token().toDouble();
			double g=token().toDouble();
			double h=token().toDouble();
			double i=token().toDouble();
            node=new BVHNodeRPY_XYZ(sName,a,b,c,d,e,f,g,h,i,pMesh);
		}
        else if (tag=="DH")
		{
	        int a=token().toInt();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
			double e=token().toDouble();
			double f=token().toDouble();
			double g=token().toDouble();
            
            if (ftSensorId==-1)
            {
                node=new BVHNodeDH(sName,a,b,c,d,e,f,g,pMesh); 
		    }
            else
            {
                node=new BVHNodeForceTorque(sName,ftSensorId,a,b,c,d,e,f,g,pMesh);
            }
        }
		break;
    case BVH_END:
        if (tag=="EYE")
		{
			double a=token().toDouble();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
            node=new BVHNodeEYE(sName,a,b,c,d,pMesh);
		}
		else if (tag=="DH")
		{
			double a=token().toDouble();
			double b=token().toDouble();
			double c=token().toDouble();
			double d=token().toDouble();
            node=new BVHNodeEND(sName,a,b,c,d,pMesh);
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


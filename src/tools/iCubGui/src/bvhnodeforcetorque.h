/*
 * bvhnodeforcetorque.h
 */

#ifndef BVHNODEFORCETORQUE_H
#define BVHNODEFORCETORQUE_H

#include "bvhnodedh.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>

class BVHNodeForceTorque : public BVHNodeDH
{
    public:
    
    BVHNodeForceTorque(const QString& name,int id,int enc,double a,double d,double alpha,double theta0,double thetaMin,double thetaMax,iCubMesh* mesh=0)
        : BVHNodeDH(name,enc,a,d,alpha,theta0,thetaMin,thetaMax,mesh)
    {
        const int CAN_DRIVER_BUFFER_SIZE=2047;

        cardId=id;

	    Property prop;
        prop.put("device", "ecan");
        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 0);
        prop.put("CanMyAddress", 0);
        prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
        prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

        pCanBus=NULL;
        pCanBufferFactory=NULL;

        driver.open(prop);
        if (!driver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return;
        }

        driver.view(pCanBus);
    
        if (!pCanBus)
        {
            fprintf(stderr, "Error opening /ecan device not available\n");
            return;
        }

        if (pMesh)
        {
            //delete pMesh;
            //pMesh=0;
        }

        //m_Alpha=0.5;

        driver.view(pCanBufferFactory);
        pCanBus->canSetBaudRate(0); //default 1MB/s

        pCanBus->canIdAdd(cardId | 0x0A); // force
        pCanBus->canIdAdd(cardId | 0x0B); // torque

        canBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    }
        
	virtual ~BVHNodeForceTorque()
    {	
	    if (pCanBufferFactory) 
        {
            pCanBufferFactory->destroyBuffer(canBuffer);
        }

        driver.close();
    }
    
    virtual void drawJoint()
    {
        static const int FORCE=0x0A,TORQUE=0x0B;

        BVHNodeDH::drawJoint();

        if (!pCanBus) return;

        unsigned int canMessages=0;
        bool res=pCanBus->canRead(canBuffer,1000,&canMessages,false);

        int val=0;
        unsigned char *pLsb=(unsigned char *)&val,*pMsb=pLsb+1;

        for (unsigned int i=0; i<canMessages; i++)
        {
            CanMessage &msg=canBuffer[i];

            if ((msg.getId() & 0xFFFFFFF0) == cardId)
            {
                int type=msg.getId() & 0x0F;

                if (type==FORCE)
                {
                    for (int i=0; i<3; ++i)
                    {
                        *pLsb=msg.getData()[i<<1];
                        *pMsb=msg.getData()[1+(i<<1)];

                        dForceTorque[i]=double(val-0x8000);
                    }
                }
                else if (type==TORQUE)
                {
                    for (int i=0; i<3; ++i)
                    {
                        *pLsb=msg.getData()[i<<1];
                        *pMsb=msg.getData()[1+(i<<1)];

                        dForceTorque[i+3]=double(val-0x8000);
                    }
                }
            }
        }

        glColor4f(0.4,0.4,1.0,1.0);
        glPushMatrix();
        glTranslated(0.0,0.0,15.0);
        gluDisk(cyl,0.0,27.5,16,16);
        gluCylinder(cyl,27.5,27.5,18.0,16,16);
        glTranslated(0.0,0.0,18.0);
        gluDisk(cyl,0.0,27.5,16,16);
        glTranslated(0.0,0.0,-9.0);

        glDisable(GL_DEPTH_TEST);

        // Force
        glLineWidth(3.0);

        //dForceTorque[0]=dForceTorque[1]=dForceTorque[2]=300.0;

        // X 
        glPushMatrix();
        glColor4f(1.0,0.0,0.0,1.0);
        glRotated(90.0,0.0,1.0,0.0);
        drawArrow(0.1*dForceTorque[1]);
        glPopMatrix();

        // Y
        glColor4f(0.0,1.0,0.0,1.0);
        glPushMatrix();
        glRotated(-90.0,1.0,0.0,0.0);
        drawArrow(0.1*dForceTorque[2]);
        glPopMatrix();

        // Z
        glColor4f(0.0,0.0,1.0,1.0);
        glPushMatrix();
        drawArrow(0.1*dForceTorque[0]);
        glPopMatrix();

        // Torque

        glLineWidth(2.0);
        glDisable(GL_LINE_SMOOTH);

        // X
        glColor4f(1.0,0.0,0.0,1.0);
        glPushMatrix();
        glRotated(90.0,0.0,1.0,0.0);
        drawArc(0.0005*dForceTorque[4]);
        glPopMatrix();

        // Y
        glColor4f(0.0,1.0,0.0,1.0);
        glPushMatrix();
        glRotated(-90.0,1.0,0.0,0.0);
        glRotated(180.0,0.0,0.0,10.0);
        drawArc(-0.0005*dForceTorque[3]);
        glPopMatrix();

        // Z
        glColor4f(0.0,0.0,1.0,1.0);
        glPushMatrix();
        glRotated(180.0,0.0,0.0,10.0);
        drawArc(0.0005*dForceTorque[5]);
        glPopMatrix();

        glPopMatrix();

        glEnable(GL_LINE_SMOOTH);

        glEnable(GL_DEPTH_TEST);
    }

protected:
    double dForceTorque[6];
    
    PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer canBuffer;

    int cardId;
};

#endif

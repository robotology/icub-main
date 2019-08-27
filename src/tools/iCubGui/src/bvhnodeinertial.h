/*
 * bvhnodeinertial.h
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

#ifndef BVHNODEINERTIAL_H
#define BVHNODEINERTIAL_H

#include "bvhnodeend.h"
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <cstring>

constexpr double DEG2RAD=3.14159265/180.0;
extern std::string GUI_NAME;
class BVHNodeINERTIAL : public BVHNodeEND
{
public:

    BVHNodeINERTIAL(const QString& name,double a,double d,double alpha,double theta0,const QString& portIMUName,iCubMesh* mesh=nullptr)
        : BVHNodeEND(name,-1,a,d,alpha,theta0,mesh)
        {
            Property masConf {{"device",Value("multipleanalogsensorsclient")},
                              {"local", Value( GUI_NAME+"/inertials")},
                              {"remote",Value(portIMUName.toStdString())},
                              {"timeout",Value(0.04)},
                              {"externalConnection",Value(true)}};

            if (!dd_MASClient.open(masConf))
            {
                yError("unable to open the MAS client\n");
            }

            if(!dd_MASClient.view(iAcc) || !dd_MASClient.view(iGyro))
            {
                yError("view of one of the MAS interfaces required failed\n");
            }

            acc.resize(3);
            gyro.resize(3);
            }

    virtual ~BVHNodeINERTIAL()
    {
        dd_MASClient.close();
        qDebug("CLOSING INERTIAL");
    }

    void drawJoint() override
    {
        if (dd_MASClient.isValid())
        {
            bool ok{true};

            double ts;

            ok &= iAcc->getThreeAxisLinearAccelerometerMeasure(0, acc, ts);
            ok &= iGyro->getThreeAxisGyroscopeMeasure(0, gyro, ts);
            if (ok) {

                gyro[0]*=DEG2RAD;
                gyro[1]*=DEG2RAD;
                gyro[2]*=DEG2RAD;

                glTranslated(40.0,0.0,230.0);
                glColor4f(0.4,0.4,1.0,1.0);
                glutSolidCube(22.0);

                // Accelerometer
                glLineWidth(3.0);
                glColor4f(1.0,0.0,0.0,1.0);
                glBegin(GL_LINES);
                glVertex3d(0.0,0.0,0.0);
                glVertex3d(-10.0*acc[0],0.0,0.0);
                glEnd();

                glColor4f(0.0,1.0,0.0,1.0);
                glBegin(GL_LINES);
                glVertex3d(0.0,0.0,0.0);
                glVertex3d(0.0,-10.0*acc[1],0.0);
                glEnd();

                glColor4f(0.0,0.0,1.0,1.0);
                glBegin(GL_LINES);
                glVertex3d(0.0,0.0,0.0);
                glVertex3d(0.0,0.0,-10.0*acc[2]);
                glEnd();

                // Gyro

                glLineWidth(2.0);
                glDisable(GL_LINE_SMOOTH);

                glColor4f(1.0,0.0,0.0,1.0);
                glPushMatrix();
                glRotated(90.0,0.0,1.0,0.0);
                drawArc(gyro[0]);
                glPopMatrix();

                glColor4f(0.0,1.0,0.0,1.0);
                glPushMatrix();
                glRotated(-90.0,1.0,0.0,0.0);
                drawArc(gyro[1]);
                glPopMatrix();

                glColor4f(0.0,0.0,1.0,1.0);
                glPushMatrix();
                glRotated(-90.0,0.0,0.0,1.0);
                drawArc(gyro[2]);
                glPopMatrix();

                glEnable(GL_LINE_SMOOTH);

            }
        }
    }

private:
    yarp::dev::PolyDriver dd_MASClient;
    yarp::dev::IThreeAxisGyroscopes* iGyro{nullptr};
    yarp::dev::IThreeAxisLinearAccelerometers* iAcc{nullptr};
    yarp::sig::Vector gyro;
    yarp::sig::Vector acc;

};

#endif

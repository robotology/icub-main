// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#include "TestMotorsStictionIncremental.h"
#include  <yarp/os/Time.h>
#include  <gsl/gsl_math.h>

iCubTestMotorsStictionIncremental::iCubTestMotorsStictionIncremental(yarp::os::Searchable& configuration) : iCubTest(configuration)
{
    m_part=(iCubPart)0;

    if (configuration.check("robot"))
    {
        m_robot = std::string (configuration.find("robot").asString());
    }
    m_icubDriver.open(m_robot);

    if (configuration.check("device"))
    {
        std::string device(configuration.find("device").asString());
        m_part = iCubPart(device);
    }

    m_NumJoints=m_icubDriver.getNumOfJoints(m_part);
}

iCubTestMotorsStictionIncremental::~iCubTestMotorsStictionIncremental()
{
}

iCubTestReport* iCubTestMotorsStictionIncremental::run()
{
    iCubTestReport* pTestReport=new iCubTestReport(m_Name,m_partCode,m_Description);

    m_bSuccess=true;

    for (int joint=0; joint<1; ++joint)
    {
        double pwm;
        bool complete;
        double pos_start;
        double pos_end;
        //do 50 trials
        for (int trial=0; trial<20; trial++)
        {
            iCubTestMotorsStictionIncrementalReportEntry *pOutput=new iCubTestMotorsStictionIncrementalReportEntry();
            m_icubDriver.startOpenloopCmd(m_part,0,500);
            yarp::os::Time::delay(10);
            m_icubDriver.startOpenloopCmd(m_part,0,0);
            yarp::os::Time::delay(1);

            printf ("*********** trial %d ************\n" ,trial);
            pwm=20;
            complete = false;
            pos_start = 0;
            pos_end = 0;
            char tmpString[200];
            while (!complete)
            {
                m_icubDriver.getEncPos(m_part,0,pos_start);
                m_icubDriver.startOpenloopCmd(m_part,0,pwm);
                yarp::os::Time::delay(1);
                m_icubDriver.getEncPos(m_part,0,pos_end);
                double movement= fabs(pos_start-pos_end);
                if (movement < 1)
                {
                    printf ("pwm %f, movement %f \n", pwm, movement);
                    pwm = pwm + 2.0;
                    complete = false;
                    pos_start = 0;
                    pos_end = 0;
                    //m_icubDriver.startOpenloopCmd(m_part,0,0.0);
                    //yarp::os::Time::delay(1);
                    continue;
                }
                
                yarp::os::Time::delay(15);
                m_icubDriver.getEncPos(m_part,0,pos_end);
                movement= fabs(pos_start-pos_end);

                if ( movement > 90)
                {
                    printf ("pwm %f, movement %f --> OK ***********\n", pwm, movement);

                    sprintf(tmpString,"%f",pwm);
                    pOutput->m_PWM=std::string(tmpString);
                    
                    sprintf(tmpString,"%d",trial);
                    pOutput->m_trial=std::string(tmpString);
                    
                    sprintf(tmpString,"%f",movement);
                    pOutput->m_displacement=std::string(tmpString);
                    
                    sprintf(tmpString,"%f",movement/10.0);
                    pOutput->m_speed=std::string(tmpString);
                    pTestReport->addEntry(pOutput);
                    pwm = 20;
                    complete = true;
                    pos_start = 0;
                    pos_end = 0;
                    m_icubDriver.startOpenloopCmd(m_part,0,0);
                    yarp::os::Time::delay(1);
                }
                else
                {
                    printf ("pwm %f, movement %f \n", pwm, movement);
                    pwm = pwm + 2.0;
                    complete = false;
                    pos_start = 0;
                    pos_end = 0;
                   // m_icubDriver.startOpenloopCmd(m_part,0,0);
                   //yarp::os::Time::delay(1);
                }
            }

        }


    }

    return pTestReport;
}

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Francesco Nori
 * email:  francesco.nori@iit.it
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

#include "graspDetector.h"

graspDetector::graspDetector(int n, fingerDetector **fingDet, Port *statusPort, BufferedPort<Bottle> *analogP, int rate): RateThread(rate)
{
    analogPort = analogP;
    nFingers = n;
    fd = fingDet;
    s = new double[nFingers];
    sp = statusPort;
}

graspDetector::~graspDetector()
{    
    delete[] s;
}

bool graspDetector::threadInit()
{
    return 1;
}

void graspDetector::run()
{
    //get reading from analog sensors
    Bottle *lastBottle;
    while(!(lastBottle=analogPort->read()))
        fprintf(stderr, "Empty read\n"); 
    for(int i=0; i < nFingers; i++)
        fd[i]->copyAnalog(lastBottle);
    Stamp analogEnvelope;
    analogPort->getEnvelope(analogEnvelope);

    Bottle b; 
    for(int i=0; i < nFingers; i++)
        {
            s[i] =  (double) fd[i]->status;
            b.addDouble(s[i]);
        }
    sp->setEnvelope(analogEnvelope);
    sp->write(b);

    /*
    bool print = false;
    for(int i=0; i < nFingers; i++)
        print = print || (s[i]!=0.0);
            
    if (print)
        {
            fprintf(stderr, "Following fingers are grasping: ");
            
            for(int i=0; i < nFingers; i++)    
                if (s[i]!=0.0)
                    fprintf(stderr, "finger%d[%.2f]", i, s[i]);
            
            fprintf(stderr, "\n");
        }
    */
}

void graspDetector::threadRelease()
{

    fprintf(stderr, "Interrupting the input port \n");
    analogPort->interrupt();
    fprintf(stderr, "Closing the input port \n");
    analogPort->close();
    fprintf(stderr, "Grasp detector closed \n"); 
}

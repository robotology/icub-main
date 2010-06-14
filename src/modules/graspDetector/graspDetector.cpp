// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
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

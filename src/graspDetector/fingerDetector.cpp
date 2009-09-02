// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "fingerDetector.h"

fingerDetector::fingerDetector(BufferedPort<Bottle> *p, int rate): RateThread(rate)
{
    analogPort = p;
    status = false;
}

fingerDetector::~fingerDetector()
{    }

bool fingerDetector::threadInit()
{
    return 1;
}

void fingerDetector::stop()
{
    fprintf(stderr, "Interrupting the input port \n");
    analogPort->interrupt();
    fprintf(stderr, "Closing the input port \n");
    analogPort->close();
    fprintf(stderr, "Grasp detector closed \n");
}

void fingerDetector::setIndex(Bottle b)
{
    index.resize(b.size());
    for(int i =0; i<b.size(); i++)
        index(i) = b.get(i).asDouble();
}

void fingerDetector::setModel(Bottle b, double m, double M)
{
    lambda.resize(b.size());
    for(int i =0; i<b.size(); i++)
        lambda(i) = b.get(i).asDouble();
    min = m;
    max = M;
}

void fingerDetector::run()
{
    //fprintf(stderr, "Entering the main thread\n");
    Bottle *lastBottle;
    while(!(lastBottle=analogPort->read()))
        fprintf(stderr, "Empty read\n"); 
    Vector analogs(index.size());
    for(int i = 0; i < index.size(); i++)
        {
            if (lastBottle->get((int) index(i)).isDouble())
                analogs(i)=lastBottle->get((int) index(i)).asDouble();
        }
    //fprintf(stderr, "Reading: %s\n", analogs.toString().c_str());
    
    double res;
    res = dot(lambda,analogs) - 1;
    if (res < 1.1*min || res > 1.1*max)
        //fprintf(stderr, "Model: %s\n min=%f, max=%f and res=%f\n", lambda.toString().c_str(), min, max, res);
        status = true;   //true if grasping (model not respected) 
    else
        status = false;  //false if not grasping (model respected)
}  

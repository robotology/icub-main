// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "fingerDetector.h"

fingerDetector::fingerDetector(int rate): RateThread(rate)
{
    status = false;
}

fingerDetector::~fingerDetector()
{    

}

bool fingerDetector::threadInit()
{
    return 1;
}

void fingerDetector::stop()
{

}

void fingerDetector::setIndex(Bottle b)
{
    index.resize(b.size());
    for(int i =0; i<b.size(); i++)
        index(i) = b.get(i).asDouble();
    fingerAnalog.resize(index.size());
}

void fingerDetector::setModel(Bottle q_0, Bottle q_1, double m, double M, double t, double T)
{
    q0.resize(q_0.size());
    q1.resize(q_1.size());
    for(int i =0; i < q_0.size(); i++)
        q0(i) = q_0.get(i).asDouble();
    for(int i =0; i < q_1.size(); i++)
        q1(i) = q_1.get(i).asDouble();
    min = m;
    max = M;
    minT = t;
    maxT = T;
}

void fingerDetector::copyAnalog(Bottle *b)
{
    for(int i = 0; i < index.size(); i++)
        {
            if (b->get((int) index(i)).isDouble())
                fingerAnalog(i)=b->get((int) index(i)).asDouble();
        }    
}

void fingerDetector::run()
{
    //fprintf(stderr, "Entering the main thread\n");

    //fprintf(stderr, "Reading: %s\n", analogs.toString().c_str());
   
    int n = q1.size(); 
    Vector q(n);
    Vector qStar(n);
    Vector dq(n);
    Vector tStar(1);
    Matrix Q1(n,1);

    for (int i = 0; i < n ; i++)
        {
            Q1(i,0) = q1(i);
            q(i) = fingerAnalog(i);
        }
    tStar = pinv(Q1)*(q-q0);
    qStar = q0 + q1 * tStar(0);
    dq = q - qStar;

    double res = 0;
    double q1N = 0;
    for (int i = 0; i < n ; i++)
        {
            res += sqrt(dq(i)*dq(i));
            q1N += sqrt(q1(i)*q1(i));
        }


    if (tStar(0) > maxT && res < max)
        status = (tStar(0) - maxT) * q1N;
    if (tStar(0) > maxT && res > max)
        status = (tStar(0) - maxT) * q1N + (res - max);

    if (tStar(0) < minT && res < max)
        status = (minT - tStar(0)) * q1N;
    if (tStar(0) < minT && res > max)
        status = (minT - tStar(0)) * q1N + (res - max);

    if ( tStar(0) > minT && tStar(0) < maxT && res < max)
        status = 0.0;
    if ( tStar(0) > minT && tStar(0) < maxT && res > max)
        status = res - max;
}  

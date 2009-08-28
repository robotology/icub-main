// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "graspDetectorConf.h"

graspDetector::graspDetector(PolyDriver *Arm_d, BufferedPort<Bottle> *p, int rate): RateThread(rate)
{
    // open ports
    Arm_dd=Arm_d;

    bool ok;
    ok  = Arm_dd->view(pos);
    ok &= Arm_dd->view(vel);
    ok &= Arm_dd->view(enc);
    ok &= Arm_dd->view(pid);
    ok &= Arm_dd->view(amp);
    ok &= Arm_dd->view(lim);
  
    if (!ok)
        ACE_OS::printf("Problems acquiring arm interfaces\n");
    else
        ACE_OS::printf("Control board arm was accessed succesfully!\n");

    collect = 0;
    analogPort = p;
}

graspDetector::~graspDetector()
{    }

bool graspDetector::threadInit()
{
    return 1;
}

bool graspDetector::startMovement(double p, double v, int j)
{
    joint = j;
    bool ok = pos->setRefSpeed(j, v);
    ok &= pos->positionMove(j, p);
    //if(ok)
    //    fprintf(stderr, "Starting a movement \n");       
    //else
    //    fprintf(stderr, "Unable to move \n");       

    lastBottle= analogPort->read(true);
    if(lastBottle!=NULL)
        nJoints =lastBottle->size();             
    else
        return false;

    //    fprintf(stderr, "Getting something with size %d \n", nJoints);       
    D.resize(nJoints, N_DATA);
    span.resize(nJoints);
    return ok;
}

void graspDetector::startCollect(Bottle analogIndex)
{
    index.resize(analogIndex.size());
    for (int i = 0; i < index.size(); i++)
        index(i) = analogIndex.get(i).asDouble();
    collect = 1;
    collectCounter = 0;    
    D.zero();
    //fprintf(stderr, "Vector of indeces is: %s\n", index.toString().c_str());
}

void graspDetector::stopCollect()
{
    collect = 0;
}

bool graspDetector::endedMovement(Vector &spanned)
{
    if(collect!=0)
        {
            bool done;
            pos->checkMotionDone(joint, &done);
            if (done)
                pos->positionMove(joint, 0);
            return false;
        }   
    else
        {
            spanned.resize(index.size());
            for (int i = 0; i < index.size(); i++)
                {
                    spanned(i) = span((int) index(i));
                    //fprintf(stderr, "Retrieved: s(%d)=%f\n", (int) index(i), span((int) index(i)));
                }
            return true;
        }
}

void graspDetector::stop()
{
    fprintf(stderr, "Closing device driver \n");
    Arm_dd->close();
    fprintf(stderr, "Interrupting the input port \n");
    analogPort->interrupt();
    fprintf(stderr, "Closing the input port \n");
    analogPort->close();
    fprintf(stderr, "Grasp detector closed \n");
}

/*
 * Builds a description of the data contained in
 * the data matrix D.
 */

void graspDetector::buildPattern()
{
    
    //Mean value is t
    //for (int j = 0; j < nJoints; j++)
    //    D(j,0) = 0;
    //ACE_OS::printf("Matrix D is:\n%s\n", D.getRow(1).toString().c_str());
    //Matrix V(nJoints,nJoints);
    //Vector S(nJoints);
    //Matrix U(N_DATA, N_DATA);
    //SVD(D.transposed(), U, S, V);
    //ACE_OS::printf("Vector S is: %s\n", S.toString().c_str());
    //ACE_OS::printf("Confidence on spanned space is: %f\n", S(0)/S(1));
    //ACE_OS::printf("Spanned space is: %s\n", V.getCol(0).toString().c_str());
    //span = V.getCol(0);
    
    int i,j;
    Matrix  Q(index.size(), N_DATA);
    Vector b(N_DATA);
    for (i = 0; i < index.size(); i++)
        for (j = 0; j < N_DATA; j++)
            Q(i,j) = D(index(i),j);

    for (j = 0; j < N_DATA; j++)
        b(j) = 1;

    lambda = pinv(Q.transposed())*b;
    //ACE_OS::printf("Space is: %s\n", lambda.toString().c_str());
    Vector res;
    res = Q.transposed() * lambda - b;
    //ACE_OS::printf("Residuals are: %s\n", res.toString().c_str());
    gsl_vector_minmax((const gsl_vector *) res.getGslVector(), &minError, &maxError);

    //ACE_OS::printf("Error statistics are: max=%f, min=%f\n", maxError, minError);

    span.zero();
    for (i = 0; i < index.size(); i++)
        span(index(i)) = lambda(i);

    collect = 0;
}

bool graspDetector::getPattern(Vector &l, double &m, double &M)
{
    m = minError;
    M = maxError;
    l = lambda;
}
    

void graspDetector::run()
{
    //ACE_OS::printf("Entering the main thread\n");
    if (collect!=0)
        {
            while(!(lastBottle=analogPort->read()))
                ACE_OS::printf("Empty read\n");
            if(lastBottle!=NULL)
                {    
                    //ACE_OS::printf("Getting something\n");
                    Vector q(lastBottle->size());
                    for(int i = 0; i < nJoints; i++)
                        {
                            if (lastBottle->get(i).isDouble())
                                D(i,collectCounter)=lastBottle->get(i).asDouble();
                            //ACE_OS::printf("Getting a value %d %d  %s\n", collectCounter, i, D.getCol(collectCounter).toString().c_str());
                        }
                    collectCounter++;
                    if (collectCounter >= N_DATA)
                        {
                            buildPattern();
                        }
                }
        }
}  

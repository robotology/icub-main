// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "graspDetectorConf.h"

graspDetectorConf::graspDetectorConf(BufferedPort<Bottle> *p, int rate): RateThread(rate)
{
    collect = 0;
    analogPort = p;
}

graspDetectorConf::~graspDetectorConf()
{    }

bool graspDetectorConf::threadInit()
{
    return 1;
}

bool graspDetectorConf::startCollect(Bottle analogIndex)
{
    lastBottle= analogPort->read(true);
    if(lastBottle!=NULL)
        nJoints =lastBottle->size();             
    else
        return false;

    //    fprintf(stderr, "Getting something with size %d \n", nJoints);       
    D.resize(nJoints, N_DATA);

    index.resize(analogIndex.size());
    for (int i = 0; i < index.size(); i++)
        index(i) = analogIndex.get(i).asDouble();
    collect = 1;
    collectCounter = 0;    
    D.zero();
    return true;
    //fprintf(stderr, "Vector of indeces is: %s\n", index.toString().c_str());
}

void graspDetectorConf::stopCollect()
{
    collect = 0;
}

bool graspDetectorConf::endedMovement()
{
    if(collect!=0)
        return false;
    else
        return true;
}

void graspDetectorConf::stop()
{
    fprintf(stderr, "Interrupting the input port \n");
    analogPort->interrupt();
    fprintf(stderr, "Closing the input port \n");
    analogPort->close();
    fprintf(stderr, "Grasp detector closed \n");
}

void graspDetectorConf::buildPattern()
{    
    int i,j;
    int n=index.size();

    Vector tmp;
    Matrix  Q(2, N_DATA);
    Vector b(N_DATA);

    q0.resize(n);
    q1.resize(n);

    Vector lambda;

    q0(0) = 0;
    q1(0) = 1;
    for (i = 1; i < n ; i++)
        {
            for (j = 0; j < N_DATA; j++)
                {
                    Q(0,j) = D((int) index(0),j);
                    Q(1,j) = D((int) index(i),j);
                    b(j) = 1;
                }

            Matrix QTpinv = pinv(Q.transposed(), 1e-5);

            lambda = QTpinv*b;
            //tmp = Q.transposed()*lambda - b;
            //for (j = 0; j < N_DATA; j++)
            //    fprintf(stderr, "pinv(QT): %s\n", QTpinv.getCol(j).toString().c_str());       
            //fprintf(stderr, "b: %s\n", b.toString().c_str());       
            ///fprintf(stderr, "lambda: %s\n", lambda.toString().c_str());       
            //fprintf(stderr, "tmp: %s\n", tmp.toString().c_str());       
            double k_i1 = lambda(0);
            double k_ii = lambda(1);
            q0(i) =     1/k_ii;
            q1(i) = -k_i1/k_ii;
        }
    //fprintf(stderr, "q0: %s\n", q0.toString().c_str());
    //fprintf(stderr, "q1: %s\n\n", q1.toString().c_str());       


    Vector tStar, q, dq, res;
    tStar.resize(N_DATA);
    res.resize(N_DATA);
    q.resize(n);
    dq.resize(n);
    Matrix Q1(n,1);
    Matrix QStar(n,N_DATA);
    for (i = 0; i < n ; i++)
        Q1(i,0) = q1(i);

    //fprintf(stderr, "Q1: %s\n", Q1.toString().c_str());        
    for (j = 0; j < N_DATA; j++)
        {
            for (i = 0; i < n ; i++)
                q(i) = D((int) index(i),j);
            tmp = pinv(Q1)*(q-q0);
            tStar(j) = tmp(0); 
            
            for (i = 0; i < n ; i++)
                {
                    QStar(i,j) = q0(i) + q1(i) * tStar(j);
                    dq(i) = QStar(i,j) - q(i);
                }
            //fprintf(stderr, "dq: %s\n", dq.toString().c_str());        

            res(j)=0;
            for (i = 0; i < n ; i++)
                res(j) += sqrt(dq(i)*dq(i));
        }
    
    //fprintf(stderr, "tStar: %s\n", tStar.toString().c_str());        
    //fprintf(stderr, "res: %s\n", res.toString().c_str());        
    gsl_vector_minmax((const gsl_vector *) res.getGslVector(), &minError, &maxError);    
    gsl_vector_minmax((const gsl_vector *) tStar.getGslVector(), &minT, &maxT);    
    collect = 0;
}

void graspDetectorConf::getPattern(Vector &q_0, Vector &q_1, double &m, double &M, double &t, double &T)
{
    t = minT;
    T = maxT;
    m = minError;
    M = maxError;
    q_0 = q0;
    q_1 = q1;
}
    

void graspDetectorConf::run()
{
    //ACE_OS::printf("Entering the main thread\n");
    if (collect!=0)
        {
            while(!(lastBottle=analogPort->read()))
                printf("Empty read\n");
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

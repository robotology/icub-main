#include "rfwr.h"

#include "RFNet.h"
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

using namespace yarp;
using namespace yarp::sig;

Vector & copy(Vector &l, YVector &r)
{
    for(int i=0;i<l.length();i++)
        l[i]=r[i];

    return l;
}

Matrix &copy(Matrix &l, YMatrix &r)
{
    double *tmpL=l.data();
    double *tmpR=*r.data();

    for(int kk=0;kk<l.rows()*l.cols();kk++)
       tmpL[kk]=tmpR[kk];
    return l;
}

RFWR::RFWR()
{
    rfnet=new RFNet();
}

RFWR::~RFWR()
{
    delete rfnet;
}

void RFWR::initialize(int n_in,int n_out, 
	     bool diag_only, 
	     bool meta, 
	     double meta_rate,	
	     double penalty,	
	     double init_alpha,
         const sig::Vector &norm,
         const sig::Vector &norm_out)
{
    rfnet->Init(n_in, n_out, diag_only, meta, meta_rate, penalty, init_alpha,
        YVector(norm.length(), norm.data()),
        YVector(norm_out.length(), norm_out.data()));
}

double RFWR::train(const sig::Vector &x, const sig::Vector &y)
{
    return rfnet->Train(YVector(x.length(), x.data()), 
                        YVector(y.length(), y.data()));
}

double RFWR::train(const sig::Vector &x, const sig::Vector &y, sig::Vector &yp)
{
    YVector tmp(yp.length());

    double ret=rfnet->Train(YVector(x.length(), x.data()), 
                        YVector(y.length(), y.data()),
                        tmp);

    copy(yp,tmp);

    return ret;
}

double RFWR::simulate(const sig::Vector &x, sig::Vector &yp, double cutoff)
{
    YVector tmp(yp.length());

    double ret=rfnet->Simulate(YVector(x.length(), x.data()), cutoff,
                        tmp);

    copy(yp,tmp);
    return ret;
}

void RFWR::jacobian(const sig::Vector &x, sig::Matrix &J, double cutoff)
{
    YMatrix tmp(J.rows(), J.cols());

    rfnet->Jacobian(YVector(x.length(),x.data()),
            cutoff,
            tmp);
    
    copy(J,tmp);
} 

void RFWR::save(const char *file)
{
    rfnet->SaveNet(file);
}

int RFWR::load(const char *file)
{
    return rfnet->LoadNet(file);
}

int RFWR::getInSize()
{
    return rfnet->getInSize();
}

int RFWR::getOutSize()
{
    return rfnet->getOutSize();
}


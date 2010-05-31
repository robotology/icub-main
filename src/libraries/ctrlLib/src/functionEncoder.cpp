
#include <yarp/math/Math.h>
#include <iCub/ctrl/functionEncoder.h>

#define W_SZ    57

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;

namespace ctrl
{
    double waveLUP[W_SZ][2]={ {   0.0,                   0.0 },
                              { 0.125,    0.0345833411415645 },
                              {  0.25,     0.107309738113335 },
                              { 0.375,     0.202014885729746 }, 
                              {   0.5,     0.328773960240219 }, 
                              { 0.625,     0.467801596736295 },
                              {  0.75,     0.618433317340402 },
                              { 0.875,     0.799799587363102 },
                              {   1.0,      1.00839956631986 },
                              { 1.125,      1.11213550554368 },
                              {  1.25,      1.10075377628576 },
                              { 1.375,      1.03456741914056 },
                              {   1.5,     0.876691003658645 },
                              { 1.625,     0.708936729237485 },
                              {  1.75,     0.535149009932799 },
                              { 1.875,     0.281586312944196 },
                              {   2.0,   -0.0358782406601501 },
                              { 2.125,    -0.233040642106083 },
                              {  2.25,    -0.302412553008806 },
                              { 2.375,    -0.321628766530846 },
                              {   2.5,    -0.241966742214465 },
                              { 2.625,    -0.190865137334676 },
                              {  2.75,    -0.175103704342541 },
                              { 2.875,   -0.0890678401406110 },
                              {   3.0,    0.0407115008395991 },
                              { 3.125,     0.108682072349430 },
                              {  3.25,     0.118185639560005 },
                              { 3.375,     0.101870197888310 },
                              {   3.5,    0.0341494900045930 },
                              { 3.625,   0.00431657421191742 },
                              {  3.75,    0.0162890023018952 },
                              { 3.875,   0.00436750867994683 },
                              {   4.0,   -0.0120853649259263 },
                              { 4.125,   -0.0194095628614931 },
                              {  4.25,   -0.0234198437934784 },
                              { 4.375,   -0.0169172631594144 },
                              {   4.5,   0.00224836903406590 },
                              { 4.625,   0.00948830118465152 },
                              {  4.75,   0.00525142022644811 },
                              { 4.875,   0.00340625270534754 },
                              {   5.0,  -0.00116770339087780 },
                              { 5.125,  -0.00296115923392665 },
                              {  5.25, -0.000413390931405029 },
                              { 5.375,  9.35269316433299e-05 },
                              {   5.5,  0.000103919276941685 },
                              { 5.625,  0.000321935964326630 },
                              {  5.75, -1.90454590045895e-05 },
                              { 5.875, -9.18215519817765e-05 },
                              {   6.0,  2.02418174948211e-05 },
                              { 6.125,  1.04451668231589e-05 },
                              {  6.25, -3.36622541321180e-06 },
                              { 6.375,                   0.0 },
                              {   6.5,                   0.0 },
                              { 6.625,                   0.0 },
                              {  6.75,                   0.0 },
                              { 6.875,                   0.0 },
                              {   7.0,                   0.0 } };

    /************************************************************************/
    double integrand(double x, void *params)
    {
        waveletEncoder *pWavEnc=(waveletEncoder*)params;

        Vector *Val=pWavEnc->pVal;
        unsigned int n=pWavEnc->iCoeff;
        double R=pWavEnc->Resolution;

        return (pWavEnc->interpFunction(*Val,x)-(*Val)[0]) * (R*pWavEnc->interpWavelet(R*x-n));
    }
}


/************************************************************************/
waveletEncoder::waveletEncoder()
{
    w=gsl_integration_workspace_alloc(1000);
    
    F.function=&integrand;
    F.params=(void*)this;
}


/************************************************************************/
double waveletEncoder::interpWavelet(const double x)
{
    if (x<=waveLUP[0][0] || x>=waveLUP[W_SZ-1][0])
        return 0.0;
    else
    {
        unsigned int i=(unsigned int)(x*(double)(W_SZ-1)/waveLUP[W_SZ-1][0]);
        double x0=waveLUP[i][0];
        double y0=waveLUP[i][1];
        double x1=waveLUP[i+1][0];
        double y1=waveLUP[i+1][1];

        return ((x-x0)/(x1-x0))*(y1-y0)+y0;
    }
}


/************************************************************************/
double waveletEncoder::interpFunction(const Vector &Val, const double x)
{
    // x shall be in [0,1]
    double L=Val.size()-1;

    if (x<=0.0)
        return Val[0];
    else if (x>=1.0)
        return Val[(size_t)L];
    else
    {
        size_t i=(size_t)(x*L);
        double x0=((double)i)/L;
        double y0=Val[i];

        if (++i>L)
            i=(size_t)L;

        double x1=((double)i)/L;
        double y1=Val[i];

        return ((x-x0)/(x1-x0))*(y1-y0)+y0;
    }
}


/************************************************************************/
Vector waveletEncoder::encode(Vector &Val, double R)
{
    // apply saturation
    if (R<0.0)
        R=0.0;
    else if (R>W_SZ-1)
        R=W_SZ-1;

    pVal=&Val;
    Resolution=R;

    unsigned int N=(unsigned int)R+1;
    Vector Coeffs(N+1);
    double error;

    Coeffs[0]=Val[0];

    for (iCoeff=0; iCoeff<N; iCoeff++)
    {
        double tau1=(waveLUP[0][0]+iCoeff)/R;
        double tau2=(waveLUP[W_SZ-1][0]+iCoeff)/R;

        gsl_integration_qag(&F,tau1,tau2,1e-3,1e-2,1000,GSL_INTEG_GAUSS61,w,
                            &Coeffs[iCoeff+1],&error);
    }

    return Coeffs;
}


/************************************************************************/
double waveletEncoder::decode(const Vector &Coeffs, double R, const double x)
{
    // apply saturation
    if (R<0.0)
        R=0.0;
    else if (R>W_SZ-1)
        R=W_SZ-1;

    unsigned int N=(unsigned int)R+1;
    double decodedVal=Coeffs[0];

    // compute the linear combination of wavelets
    for (unsigned int n=0; n<N; n++)
        decodedVal+=Coeffs[n+1]*interpWavelet(R*x-n);

    return decodedVal;
}


/************************************************************************/
waveletEncoder::~waveletEncoder()
{
    gsl_integration_workspace_free(w);
}




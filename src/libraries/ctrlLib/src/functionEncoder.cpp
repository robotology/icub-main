/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <algorithm>

#include <gsl/gsl_integration.h>

#include <yarp/math/Math.h>
#include <iCub/ctrl/functionEncoder.h>

#define CAST_GSLFUNC(x)     (static_cast<gsl_function*>(x))
#define CAST_GSLWS(x)       (static_cast<gsl_integration_workspace*>(x))
#define WAVELET_LUP_SIZE    57

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

namespace iCub
{

namespace ctrl
{
    double waveLUP[WAVELET_LUP_SIZE][2]={ { 0.000,                   0.0 },
                                          { 0.125,    0.0345833411415645 },
                                          { 0.250,     0.107309738113335 },
                                          { 0.375,     0.202014885729746 }, 
                                          { 0.500,     0.328773960240219 }, 
                                          { 0.625,     0.467801596736295 },
                                          { 0.750,     0.618433317340402 },
                                          { 0.875,     0.799799587363102 },
                                          { 1.000,      1.00839956631986 },
                                          { 1.125,      1.11213550554368 },
                                          { 1.250,      1.10075377628576 },
                                          { 1.375,      1.03456741914056 },
                                          { 1.500,     0.876691003658645 },
                                          { 1.625,     0.708936729237485 },
                                          { 1.750,     0.535149009932799 },
                                          { 1.875,     0.281586312944196 },
                                          { 2.000,   -0.0358782406601501 },
                                          { 2.125,    -0.233040642106083 },
                                          { 2.250,    -0.302412553008806 },
                                          { 2.375,    -0.321628766530846 },
                                          { 2.500,    -0.241966742214465 },
                                          { 2.625,    -0.190865137334676 },
                                          { 2.750,    -0.175103704342541 },
                                          { 2.875,   -0.0890678401406110 },
                                          { 3.000,    0.0407115008395991 },
                                          { 3.125,     0.108682072349430 },
                                          { 3.250,     0.118185639560005 },
                                          { 3.375,     0.101870197888310 },
                                          { 3.500,    0.0341494900045930 },
                                          { 3.625,   0.00431657421191742 },
                                          { 3.750,    0.0162890023018952 },
                                          { 3.875,   0.00436750867994683 },
                                          { 4.000,   -0.0120853649259263 },
                                          { 4.125,   -0.0194095628614931 },
                                          { 4.250,   -0.0234198437934784 },
                                          { 4.375,   -0.0169172631594144 },
                                          { 4.500,   0.00224836903406590 },
                                          { 4.625,   0.00948830118465152 },
                                          { 4.750,   0.00525142022644811 },
                                          { 4.875,   0.00340625270534754 },
                                          { 5.000,  -0.00116770339087780 },
                                          { 5.125,  -0.00296115923392665 },
                                          { 5.250, -0.000413390931405029 },
                                          { 5.375,  9.35269316433299e-05 },
                                          { 5.500,  0.000103919276941685 },
                                          { 5.625,  0.000321935964326630 },
                                          { 5.750, -1.90454590045895e-05 },
                                          { 5.875, -9.18215519817765e-05 },
                                          { 6.000,  2.02418174948211e-05 },
                                          { 6.125,  1.04451668231589e-05 },
                                          { 6.250, -3.36622541321180e-06 },
                                          { 6.375,                   0.0 },
                                          { 6.500,                   0.0 },
                                          { 6.625,                   0.0 },
                                          { 6.750,                   0.0 },
                                          { 6.875,                   0.0 },
                                          { 7.000,                   0.0 } };

    /************************************************************************/
    double waveletIntegrand(double x, void *params)
    {
        WaveletEncoder *pWavEnc=(WaveletEncoder*)params;
        const Vector &values=*pWavEnc->pVal;
        unsigned int n=pWavEnc->iCoeff;
        double R=pWavEnc->resolution;
        return (pWavEnc->interpFunction(values,x)-values[0]) * (R*pWavEnc->interpWavelet(R*x-n));
    }
}

}


/************************************************************************/
WaveletEncoder::WaveletEncoder()
{
    resolution=(WAVELET_LUP_SIZE-1)/2.0;
    w=gsl_integration_workspace_alloc(1000);
    F=new gsl_function;

    CAST_GSLFUNC(F)->function=&waveletIntegrand;
    CAST_GSLFUNC(F)->params=(void*)this;
}


/************************************************************************/
double WaveletEncoder::interpWavelet(const double x)
{
    if ((x<=waveLUP[0][0]) || (x>=waveLUP[WAVELET_LUP_SIZE-1][0]))
        return 0.0;
    else
    {
        unsigned int i=(unsigned int)(x*(double)(WAVELET_LUP_SIZE-1)/waveLUP[WAVELET_LUP_SIZE-1][0]);
        double x0=waveLUP[i][0];
        double y0=waveLUP[i][1];
        double x1=waveLUP[i+1][0];
        double y1=waveLUP[i+1][1];

        return ((x-x0)/(x1-x0))*(y1-y0)+y0;
    }
}


/************************************************************************/
double WaveletEncoder::interpFunction(const Vector &values, const double x)
{
    // x shall be in [0,1]
    double L=(double)values.size()-1.0;

    if (x<=0.0)
        return values[0];
    else if (x>=1.0)
        return values[(size_t)L];
    else
    {
        size_t i=(size_t)(x*L);
        double x0=((double)i)/L;
        double y0=values[i];

        if (++i>L)
            i=(size_t)L;

        double x1=((double)i)/L;
        double y1=values[i];

        return ((x-x0)/(x1-x0))*(y1-y0)+y0;
    }
}


/************************************************************************/
bool WaveletEncoder::setEncoderOptions(const Property &options)
{
    if (options.check("resolution"))
    {
        double R=options.find("resolution").asDouble();

        // apply saturation
        resolution=std::max(0.0,std::min(R,double(WAVELET_LUP_SIZE-1)));
        return true;
    }
    else
        return false;
}


/************************************************************************/
Property WaveletEncoder::getEncoderOptions()
{
    Property options;
    options.put("resolution",resolution);
    return options;
}


/************************************************************************/
Code WaveletEncoder::encode(const Vector &values)
{
    Code code;
    pVal=&values;
    unsigned int N=(unsigned int)resolution+1;    
    code.coefficients.resize(N+1);
    double error;

    code.coefficients[0]=values[0];
    for (iCoeff=0; iCoeff<N; iCoeff++)
    {
        double tau1=(waveLUP[0][0]+iCoeff)/resolution;
        double tau2=(waveLUP[WAVELET_LUP_SIZE-1][0]+iCoeff)/resolution;

        gsl_integration_qag(CAST_GSLFUNC(F),tau1,tau2,1e-3,1e-2,1000,GSL_INTEG_GAUSS61,CAST_GSLWS(w),
                            &code.coefficients[iCoeff+1],&error);
    }

    return code;
}


/************************************************************************/
double WaveletEncoder::decode(const Code &code, const double x)
{
    unsigned int N=(unsigned int)resolution+1;
    double decodedVal=code.coefficients[0];

    // compute the linear combination of wavelets
    for (unsigned int n=0; n<N; n++)
        decodedVal+=code.coefficients[n+1]*interpWavelet(resolution*x-n);

    return decodedVal;
}


/************************************************************************/
WaveletEncoder::~WaveletEncoder()
{
    gsl_integration_workspace_free(CAST_GSLWS(w));
    delete CAST_GSLFUNC(F);
}




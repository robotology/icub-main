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

/**
 * \defgroup functionEncoder Functions Encoding
 *  
 * @ingroup ctrlLib
 *
 * Classes for encoding functions.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __FUNCTIONENCODER_H__
#define __FUNCTIONENCODER_H__

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup functionEncoder
*  
* Code. 
*/
struct Code
{
    /**
    * The vector of coefficients encoding the function.
    */
    yarp::sig::Vector coefficients;
};


/**
* \ingroup functionEncoder
*
* Abstract class to deal with function encoding.
*/
class FunctionEncoder
{
public:
    /**
    * Configure the encoder. 
    * @param options encoder's options in form of a Property object.
    * @return true/false on success/fail. 
    */
    virtual bool setEncoderOptions(const yarp::os::Property &options) = 0;

    /**
    * Retrieve the encoder's configuration options.
    * @return a Property object containing the encoder's options. 
    */
    virtual yarp::os::Property getEncoderOptions() = 0;

    /**
    * Encode the function.
    * @param values is the vector containing the samples of function
    *            \b f to be encoded. The \e x coordinates of the
    *            points are intended to be normalized in [0,1], so
    *            that f(0)=values[0] and
    *            f(1)=values[values.length()-1].
    * @return the code encoding the function \b f.
    */
    virtual Code encode(const yarp::sig::Vector &values) = 0;

    /**
    * Compute the approximated value of function in \b x, given the 
    * code. 
    * @param code contains the function representation in the 
    *                 vector space.
    * @param x is the point at which the result is computed. It 
    *          shall be in [0,1].
    * @return the decoded function value. 
    */
    virtual double decode(const Code &code, const double x) = 0;

    /**
    * Destructor. 
    */
    virtual ~FunctionEncoder() { }
};


/**
* \ingroup functionEncoder
*
* Encode any given function as a set of wavelet coefficients. 
* The father wavelet used here is the \b db4.
*/
class WaveletEncoder : public FunctionEncoder
{
protected:
    void *w;
    void *F;
    const yarp::sig::Vector *pVal;

    unsigned int iCoeff;
    double resolution;

    double interpWavelet(const double x);
    double interpFunction(const yarp::sig::Vector &values, const double x);

    friend double waveletIntegrand(double, void*);

public:
    /**
    * Constructor. 
    */
    WaveletEncoder();

    /**
    * Configure the encoder. 
    * @param options lets user specify the resolution R to which the
    *                encoding is computed. Accepted options are of
    *                the form ("resolution" <double>).
    * @return true/false on success/fail. 
    *  
    * @note It holds that N=floor(R)+1, where N is the number of 
    *       coefficients of the vector space. Recap that floor(.) is
    *       the round function towards minus infinity.
    */
    virtual bool setEncoderOptions(const yarp::os::Property &options);

    /**
    * Retrieve the encoder's configuration options.
    * @return a Property object containing the encoder options. 
    */
    virtual yarp::os::Property getEncoderOptions();

    /**
    * Encode the function.
    * @param values is the vector containing the samples of function
    *            \b f to be encoded. The \e x coordinates of the
    *            points are intended to be normalized in [0,1], so
    *            that f(0)=values[0] and
    *            f(1)=values[values.length()-1].
    * @return the code containing the computed 1+N coefficients,
    *         with the first one being f(0) and the following N are
    *         the actual wavelet expansion coefficients.
    */
    virtual Code encode(const yarp::sig::Vector &values);

    /**
    * Compute the approximated value of function in \b x, given the 
    * input set of wavelet coefficients. 
    * @param code contains wavelet coefficients vector along with 
    *               the first initial value f(0).
    * @param x is the point at which the result is computed. It 
    *          shall be in [0,1].
    * @return the decoded function value. 
    *  
    * @note It shall hold that coefficients.length()>=floor(R)+2.
    */
    virtual double decode(const Code &code, const double x);

    /**
    * Destructor. 
    */
    virtual ~WaveletEncoder();
};

}

}

#endif



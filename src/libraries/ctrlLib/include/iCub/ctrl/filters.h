/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

/**
 * \defgroup Filters Filters 
 *  
 * @ingroup ctrlLib
 *
 * Classes for filtering.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <deque>

#include <yarp/sig/Vector.h>
#include <iCub/ctrl/math.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup IFilter
*
* Interface for the filters implemented in iCub::crtl.
*/
class IFilter
{
public:
    /**
    * Destructor
    */
    virtual ~IFilter() { }

    /**
    * Internal state reset.
    * @param y0 new internal state.
    */
    virtual void init(const yarp::sig::Vector& y0) = 0;

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input.
    * @return the corresponding output.
    */
    virtual const yarp::sig::Vector& filt(const yarp::sig::Vector& u) = 0;

    /**
    * Return current filter output.
    * @return the filter output.
    */
    virtual const yarp::sig::Vector& output() const = 0;
};


/**
* \ingroup Filters
*
* IIR and FIR.
*/
class Filter : public IFilter
{
protected:
   yarp::sig::Vector b;
   yarp::sig::Vector a;
   yarp::sig::Vector y;

   std::deque<yarp::sig::Vector> uold;
   std::deque<yarp::sig::Vector> yold;
   size_t n;
   size_t m;

public:
   /**
   * Creates a filter with specified numerator and denominator 
   * coefficients. 
   * @param num vector of numerator elements given as increasing 
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing 
   *            power of z^-1. 
   * @param y0 initial output.
   * @note den[0] shall not be 0. 
   */ 
   Filter(const yarp::sig::Vector &num, const yarp::sig::Vector &den,
          const yarp::sig::Vector &y0=yarp::sig::Vector(1,0.0));

   /**
   * Internal state reset. 
   * @param y0 new internal state.
   */ 
   virtual void init(const yarp::sig::Vector &y0);

   /**
   * Internal state reset for filter with zero gain.
   * @param y0 new internal state.
   * @param u0 expected next input.
   * @note The gain of a digital filter is the sum of the coefficients of its 
   *       numerator divided by the sum of the coefficients of its denumerator.
   */ 
   virtual void init(const yarp::sig::Vector &y0, const yarp::sig::Vector &u0);

   /**
   * Returns the current filter coefficients.
   * @param num vector of numerator elements returned as increasing
   *            power of z^-1.
   * @param den vector of denominator elements returned as 
   *            increasing power of z^-1.
   */ 
   void getCoeffs(yarp::sig::Vector &num, yarp::sig::Vector &den);

   /**
   * Sets new filter coefficients.
   * @param num vector of numerator elements given as increasing 
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing 
   *            power of z^-1. 
   * @note den[0] shall not be 0. 
   * @note the internal state is reinitialized to the current 
   *       output.
   */ 
   void setCoeffs(const yarp::sig::Vector &num, const yarp::sig::Vector &den);

   /**
   * Modifies the values of existing filter coefficients without 
   * varying their lengths. 
   * @param num vector of numerator elements given as increasing 
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing 
   *            power of z^-1.
   * @return true/false on success/fail. 
   * @note den[0] shall not be 0. 
   * @note the adjustment is carried out iff num.size() and 
   *       den.size() match the existing numerator and denominator
   *       lengths.
   */ 
   bool adjustCoeffs(const yarp::sig::Vector &num, const yarp::sig::Vector &den);

   /**
   * Returns the current filter states.
   * @param u the current input states. 
   * @param y the current output states. 
   */ 
   void getStates(std::deque<yarp::sig::Vector> &u, std::deque<yarp::sig::Vector> &y);

   /**
   * Performs filtering on the actual input.
   * @param u reference to the actual input. 
   * @return the corresponding output. 
   */ 
   virtual const yarp::sig::Vector& filt(const yarp::sig::Vector &u);

   /**
   * Return current filter output.
   * @return the filter output. 
   */ 
   virtual const yarp::sig::Vector& output() const { return y; }
};


/**
* \ingroup Filters
*
* Rate Limiter.
*/
class RateLimiter : public IFilter
{
protected:
    yarp::sig::Vector uD;
    yarp::sig::Vector uLim;
    yarp::sig::Vector rateUpperLim;
    yarp::sig::Vector rateLowerLim;

    size_t n;

public:
    /**
    * Creates a Rate Limiter which keeps the rate of the input 
    * within assigned thresholds. 
    * @param rL Rate lower limit.
    * @param rU Rate upper limit.
    */
    RateLimiter(const yarp::sig::Vector &rL, const yarp::sig::Vector &rU);

    /**
    * Init internal state.
    * @param u0 new internal state.
    */
    virtual void init(const yarp::sig::Vector &u0);

    /**
    * Returns the current Rate limits.
    * @param rL Rate lower limit.
    * @param rU Rate upper limit.
    */
    void getLimits(yarp::sig::Vector &rL, yarp::sig::Vector &rU);

    /**
    * Sets new Rate limits
    * @param rL Rate lower limit.
    * @param rU Rate upper limit. 
    * @note coherence between new limits length and the state 
    *       length is not veriified.
    */
    void setLimits(const yarp::sig::Vector &rL, const yarp::sig::Vector &rU);

    /**
    * Limits the input rate. 
    * @param u is the current input.
    * @return the output within the thresholds.
    */
    virtual const yarp::sig::Vector& filt(const yarp::sig::Vector &u);

    /**
    * Return current filter output.
    * @return the filter output. 
    */ 
    virtual const yarp::sig::Vector& output() const { return uLim; }
};


/**
* \ingroup Filters
*
* First order low pass filter implementing the transfer function
* H(s) = \frac{1}{1+\tau s}
*
*/
class FirstOrderLowPassFilter : public IFilter
{
protected:
    Filter *filter;         // low pass filter
    double fc;              // cut frequency
    double Ts;              // sample time
    yarp::sig::Vector y;    // filter current output

    void computeCoeff();

public:
    /**
    * Creates a filter with specified parameters
    * @param cutFrequency cut frequency (Hz).
    * @param sampleTime sample time (s).
    * @param y0 initial output.
    */ 
    FirstOrderLowPassFilter(const double cutFrequency, const double sampleTime,
                            const yarp::sig::Vector &y0=yarp::sig::Vector(1,0.0));

    /**
    * Destructor. 
    */
    virtual ~FirstOrderLowPassFilter();

    /**
    * Internal state reset. 
    * @param y0 new internal state.
    */ 
    virtual void init(const yarp::sig::Vector &y0);

    /**
    * Change the cut frequency of the filter. 
    * @param cutFrequency the new cut frequency (Hz). 
    */
    bool setCutFrequency(const double cutFrequency);

    /**
    * Change the sample time of the filter. 
    * @param sampleTime the new sample time (s). 
    */
    bool setSampleTime(const double sampleTime);

    /**
    * Retrieve the cut frequency of the filter. 
    * @return the cut frequency (Hz). 
    */
    double getCutFrequency() { return fc; }

    /**
    * Retrieve the sample time of the filter. 
    * @return the sample time (s). 
    */
    double getSampleTime() { return Ts; }

    /**
    * Performs filtering on the actual input.
    * @param u reference to the actual input. 
    * @return the corresponding output. 
    */ 
    virtual const yarp::sig::Vector& filt(const yarp::sig::Vector &u);

    /**
    * Return current filter output.
    * @return the filter output. 
    */ 
    virtual const yarp::sig::Vector& output() const { return y; }
};


/**
* \ingroup Filters
*
* Median Filter
*/
class MedianFilter : public IFilter
{
protected:
   std::deque<std::deque<double> > uold;
   yarp::sig::Vector y;
   size_t n;
   size_t m;

   double median(std::deque<double>& v);

public:
   /**
   * Creates a median filter of the specified order.
   * @param n the filter order.
   * @param y0 initial output.
   */ 
   MedianFilter(const size_t n, const yarp::sig::Vector &y0=yarp::sig::Vector(1,0.0));

   /**
   * Internal state reset. 
   * @param y0 new internal state.
   */ 
   virtual void init(const yarp::sig::Vector &y0);

   /**
   * Sets new filter order.
   * @param n new filter order. 
   * @note the internal memory is reset. 
   */ 
   void setOrder(const size_t n);

   /**
   * Returns the current filter order.
   */ 
   size_t getOrder() const { return n; }

   /**
   * Performs filtering on the actual input.
   * @param u reference to the actual input. 
   * @return the corresponding output. 
   */ 
   virtual const yarp::sig::Vector& filt(const yarp::sig::Vector &u);

   /**
   * Return current filter output.
   * @return the filter output. 
   */ 
   virtual const yarp::sig::Vector& output() const { return y; }
};

}

}

#endif




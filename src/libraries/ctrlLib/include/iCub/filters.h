/**
 * \defgroup Filters Filters 
 *  
 * @ingroup ctrlLib
 *
 * Classes for filtering 
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <yarp/sig/Vector.h>
#include <iCub/ctrlMath.h>
#include <deque>


namespace ctrl
{

/**
* \ingroup Filters
*
* IIR and FIR.
*/
class Filter
{
protected:
   yarp::sig::Vector b;
   yarp::sig::Vector a;
   yarp::sig::Vector y;

   std::deque<yarp::sig::Vector> uold;
   std::deque<yarp::sig::Vector> yold;
   int n;
   int m;

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
          const yarp::sig::Vector &y0);

   /**
   * Internal state reset. 
   * @param y0 new internal state.
   */ 
   void init(const yarp::sig::Vector &y0);

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
   * @note den[0] shall not be 0. 
   * @note the adjustment is carried out iff num.size() and 
   *       den.size() match the existing numerator and denominator
   *       lenghts.
   */ 
   void adjustCoeffs(const yarp::sig::Vector &num, const yarp::sig::Vector &den);

   /**
   * Performs filtering on the actual input.
   * @param u reference to the actual input. 
   * @return the corresponding output. 
   */ 
   yarp::sig::Vector filt(const yarp::sig::Vector &u);

   /**
   * Return current filter output.
   * @return the filter output. 
   */ 
   yarp::sig::Vector output();
};


/**
* \ingroup Filters
*
* Rate Limiter.
*/
class RateLimiter
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
    void init(const yarp::sig::Vector &u0);

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
    yarp::sig::Vector filt(const yarp::sig::Vector &u);
};

}

#endif




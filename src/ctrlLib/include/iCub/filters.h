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
    class Filter;
    class RateLimiter;
}


/**
* \ingroup Filters
*
* IIR and FIR.
*/
class ctrl::Filter
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
    * \note den[0] shall not be 0. 
    */ 
   Filter(const yarp::sig::Vector &num, const yarp::sig::Vector &den,
          const yarp::sig::Vector &y0);

   /**
   * Internal state reset. 
   * @param y0 new internal state.
   */ 
   void init(const yarp::sig::Vector &y0);

   /**
   * Performs filtering on the actual input.
   * @param u reference to the actual input. 
   * @return the corresponding output. 
   */ 
   yarp::sig::Vector filt(const yarp::sig::Vector &u);
};


/**
* \ingroup Filters
*
* Rate Limiter.
*/
class ctrl::RateLimiter
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
     * 
     * @param _rL Rate lower limit 
     * @param _rU Rate upper limit 
     */
    RateLimiter(const yarp::sig::Vector &_rL, const yarp::sig::Vector &_rU);

    /**
    * Init internal state 
    * @param u0 new internal state.
    */
    void init(const yarp::sig::Vector &u0);

    /**
     * Limits the input rate. 
     * 
     * @param z Current input.
     * 
     * @return Output within the thresholds.
     */
    yarp::sig::Vector filt(const yarp::sig::Vector &u);
};


#endif




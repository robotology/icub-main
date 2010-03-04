/**
 * \defgroup adaptWinPolyEstimator adaptWinPolyEstimator 
 *  
 * @ingroup ctrlLib
 *
 * Implementation of polynomial fitting through a least-squares 
 * based algorithm on a adpative window (<a 
 * href="http://ieeexplore.ieee.org/iel5/87/19047/00880606.pdf">PDF</a>) 
 *
 * \author Ugo Pattacini
 *
 */ 


#ifndef __ADAPTWINPOLYESTIMATOR_H__
#define __ADAPTWINPOLYESTIMATOR_H__

#include <yarp/sig/Vector.h>
#include <iCub/ctrlMath.h>
#include <deque>


namespace ctrl
{

typedef struct
{
    double time;
    yarp::sig::Vector data;
} AWPolyElement;


typedef std::deque<AWPolyElement> AWPolyList;

/**
* \ingroup adaptWinPolyEstimator
*
* Adaptive window polynomial fitting. 
* Abstract class. 
*/
class AWPolyEstimator
{
protected:
    AWPolyList elemList;
    unsigned int order;
    unsigned int N;
    double D;

    yarp::sig::Vector t;
    yarp::sig::Vector x;
    yarp::sig::Vector coeff;
    yarp::sig::Vector winLen;

    bool firstRun;

    /**
    * Finds the regressor which best fits in least square sense the 
    * last n data sample couples, or all couples if n==0. 
    * @param x 
    * @param y 
    * @param n 
    * @return the regressor's coefficients.
    */ 
    virtual yarp::sig::Vector fit(const yarp::sig::Vector &x,
                                  const yarp::sig::Vector &y, const unsigned int n=0);

    /** 
    * Evaluates regressor at certain point. 
    * @param x 
    * @return regressor evaluated in x.
    */ 
    virtual double eval(double x);

    /** 
    * Returns esteeme. 
    * @note needs to be defined. 
    * @return esteeme.
    */ 
    virtual double getEsteeme() = 0;

public:
    /**
    * Creates a polynomial estimator object of order _order on an 
    * adaptive window of a maximum length _N an threshold _D.
    * @param _order is the order of polynomial fitting.
    * @param _N is the maximum windows length.
    * @param _D is the threshold.
    */ 
    AWPolyEstimator(unsigned int _order, unsigned int _N, const double _D);

    /**
    * Returns a reference to internal elements list.
    * @return reference to internal elements list.
    */
    AWPolyList &getList() { return elemList; }

    /**
    * Feeds data into the algorithm.
    * @param el is the new data of type AWPolyElement.
    */
    void feedData(const AWPolyElement &el);

    /**
    * Returns the current windows lengths.
    * @return the current windows lengths. 
    */
    yarp::sig::Vector getWinLen() { return winLen; }

    /**
    * Executes the algorithm upon the elements list, with the max 
    * deviation threshold given by D. 
    * @return the current estimation. 
    */
    yarp::sig::Vector estimate();

    /**
    * Executes the algorithm upon the elements list, with the max 
    * deviation threshold given by D. 
    * @param el is the new data of type AWPolyElement. 
    * @return the current estimation. 
    */
    yarp::sig::Vector estimate(const AWPolyElement &el);

    /**
    * Reinitializes the internal state. 
    * @note Windows lenghts are brought to the maximum value N and 
    *       output remains zero as long as fed data size reaches N.
    */
    void reset();

    /**
     * Destructor.
     */
    virtual ~AWPolyEstimator() { }
};


/**
* \ingroup adaptWinPolyEstimator
*
* Adaptive window linear fitting.
*/
class AWLinEstimator : public AWPolyEstimator
{
protected:
    /** 
     * redefines method to improve computation just for first-order 
     * estimator. 
     */
    virtual yarp::sig::Vector fit(const yarp::sig::Vector &x,
                                  const yarp::sig::Vector &y, const unsigned int n=0);

    virtual double getEsteeme() { return coeff[1]; }

public:
    AWLinEstimator(unsigned int _N, const double _D) : AWPolyEstimator(1,_N,_D) { }
};


/**
* \ingroup adaptWinPolyEstimator
*
* Adaptive window quadratic fitting.
*/
class AWQuadEstimator : public AWPolyEstimator
{
protected:
    virtual double getEsteeme() { return 2.0*coeff[2]; }

public:
    AWQuadEstimator(unsigned int _N, const double _D) : AWPolyEstimator(2,_N,_D) { }
};

}

#endif



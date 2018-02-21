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
 * \defgroup adaptWinPolyEstimator Adaptive Window Polynomial Fitting Estimator
 *  
 * @ingroup ctrlLib
 *
 * Implementation of polynomial fitting through a least-squares 
 * based algorithm on a adpative window (<a 
 * href="https://doi.org/10.1109/87.880606">PDF</a>) 
 *
 * \author Ugo Pattacini
 *
 */ 


#ifndef __ADAPTWINPOLYESTIMATOR_H__
#define __ADAPTWINPOLYESTIMATOR_H__

#include <deque>

#include <yarp/sig/Vector.h>
#include <iCub/ctrl/math.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup adaptWinPolyEstimator
*
* Basic element for adaptive polynomial fitting.
*/
class AWPolyElement
{
public:
    yarp::sig::Vector data;
    double time;

    /**
    * Default constructor.
    */ 
    AWPolyElement() {}

    /**
    * Create an element for adaptive polynomial fitting.
    * @param d is the element data vector.
    * @param t is time instant of sampled data.
    */ 
    AWPolyElement(const yarp::sig::Vector &d, const double t): data(d), time(t) { }
};


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
    yarp::sig::Vector mse;

    bool firstRun;

    /**
    * Find the regressor which best fits in least square sense the 
    * last n data sample couples, or all couples if n==0. 
    * @param x vector containing the input data.
    * @param y vector containing the output data.
    * @param n last n data sample couples to fit.
    * @return the regressor's coefficients.
    */ 
    virtual yarp::sig::Vector fit(const yarp::sig::Vector &x,
                                  const yarp::sig::Vector &y, const unsigned int n=0);

    /** 
    * Evaluate regressor at certain point. 
    * @param x the point.
    * @return regressor evaluated in x.
    */ 
    virtual double eval(double x);

    /** 
    * Return the current estimation. 
    * @note needs to be defined. 
    * @return esteeme.
    */ 
    virtual double getEsteeme() = 0;

public:
    /**
    * Create a polynomial estimator object of order _order on an 
    * adaptive window of a maximum length _N an threshold _D.
    * @param _order is the order of polynomial fitting.
    * @param _N is the maximum windows length.
    * @param _D is the threshold.
    */ 
    AWPolyEstimator(unsigned int _order, unsigned int _N, const double _D);

    /**
    * Return a reference to internal elements list.
    * @return reference to internal elements list.
    */
    AWPolyList &getList() { return elemList; }

    /**
    * Feed data into the algorithm.
    * @param el is the new data of type AWPolyElement.
    */
    void feedData(const AWPolyElement &el);

    /**
    * Return the current windows lengths.
    * @return the current windows lengths. 
    */
    yarp::sig::Vector getWinLen() { return winLen; }

    /**
    * Return the mean squared error (MSE) computed over the current 
    * windows lengths between the predictions and the real data.
    * @return the MSE. 
    */
    yarp::sig::Vector getMSE() { return mse; }

    /**
    * Execute the algorithm upon the elements list, with the max 
    * deviation threshold given by D. 
    * @return the current estimation. 
    */
    yarp::sig::Vector estimate();

    /**
    * Execute the algorithm upon the elements list, with the max 
    * deviation threshold given by D. 
    * @param el is the new data of type AWPolyElement. 
    * @return the current estimation. 
    */
    yarp::sig::Vector estimate(const AWPolyElement &el);

    /**
    * Reinitialize the internal state. 
    * @note Windows lengths are brought to the maximum value N and 
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
* Adaptive window linear fitting to estimate the first
* derivative.
*/
class AWLinEstimator : public AWPolyEstimator
{
protected:
    /** 
     * Redefine method to improve computation just for first-order 
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
* Adaptive window quadratic fitting to estimate the second
* derivative.
*/
class AWQuadEstimator : public AWPolyEstimator
{
protected:
    virtual double getEsteeme() { return 2.0*coeff[2]; }

public:
    AWQuadEstimator(unsigned int _N, const double _D) : AWPolyEstimator(2,_N,_D) { }
};

}

}

#endif



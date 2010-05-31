/**
 * \defgroup Kalman Kalman 
 *  
 * @ingroup ctrlLib
 *
 * Classes for Kalman estimation
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/ctrlMath.h>


namespace ctrl
{

/**
* \ingroup Kalman
*
* Classic Kalman estimator.
*/
class Kalman
{
protected:
    yarp::sig::Matrix A, At;
    yarp::sig::Matrix H, Ht;
    yarp::sig::Matrix Q;
    yarp::sig::Matrix R;
    yarp::sig::Matrix I;
    
    yarp::sig::Vector x;
    yarp::sig::Matrix P;
    yarp::sig::Matrix K;

    size_t n;
    size_t m;

public:
    /**
     * Init a Kalman state estimator
     * 
     * @param _A State transition matrix
     * @param _H Measurement matrix
     * @param _Q Process noise covariance
     * @param _R Measurement noise covariance 
     */
    Kalman(const yarp::sig::Matrix &_A, const yarp::sig::Matrix &_H,
           const yarp::sig::Matrix &_Q, const yarp::sig::Matrix &_R);

    /**
     * Set initial state and error covariance
     * 
     * @param _x0 Initial condition for estimated state 
     * @param _z0 Initial input measurement 
     * @param _P0 Initial condition for estimated error covariance 
     */
    void init(const yarp::sig::Vector &_z0, const yarp::sig::Vector &_x0,
              const yarp::sig::Matrix &_P0);

    /**
     * Returns the estimated state vector given the current measurement
     * 
     * @param z Current measurement
     * 
     * @return Estimated state vector
     */
    yarp::sig::Vector filt(const yarp::sig::Vector &z);

    /**
     * Returns the estimated state
     * 
     * @return Estimated state
     */
    yarp::sig::Vector get_x() { return x; }

    /**
     * Returns the estimated error covariance
     * 
     * @return Estimated error covariance
     */
    yarp::sig::Matrix get_P() { return P; }

    /**
     * Returns the Kalman gain matrix
     * 
     * @return Kalman gain matrix
     */
    yarp::sig::Matrix get_K() { return K; }
};

}

#endif




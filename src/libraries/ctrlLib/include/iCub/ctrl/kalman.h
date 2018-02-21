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
 * \defgroup Kalman Kalman Estimation
 *  
 * @ingroup ctrlLib
 *
 * Classes for Kalman estimation.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __KALMAN_H__
#define __KALMAN_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>


namespace iCub
{

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
    yarp::sig::Matrix B;
    yarp::sig::Matrix Q;
    yarp::sig::Matrix R;
    yarp::sig::Matrix I;
    
    yarp::sig::Vector x;
    yarp::sig::Matrix P;
    yarp::sig::Matrix K;
    yarp::sig::Matrix S;
    double validationGate;

    size_t n;
    size_t m;

    void initialize();

    // Default constructor: not implemented.
    Kalman();

public:
    /**
     * Init a Kalman state estimator.
     * 
     * @param _A State transition matrix.
     * @param _H Measurement matrix.
     * @param _Q Process noise covariance.
     * @param _R Measurement noise covariance.
     */
    Kalman(const yarp::sig::Matrix &_A, const yarp::sig::Matrix &_H,
           const yarp::sig::Matrix &_Q, const yarp::sig::Matrix &_R);

    /**
     * Init a Kalman state estimator.
     * 
     * @param _A State transition matrix.
     * @param _B Input matrix. 
     * @param _H Measurement matrix.
     * @param _Q Process noise covariance.
     * @param _R Measurement noise covariance.
     */
    Kalman(const yarp::sig::Matrix &_A, const yarp::sig::Matrix &_B,
           const yarp::sig::Matrix &_H, const yarp::sig::Matrix &_Q,
           const yarp::sig::Matrix &_R);

    /**
     * Set initial state and error covariance.
     * 
     * @param _x0 Initial condition for estimated state. 
     * @param _P0 Initial condition for estimated error covariance.
     * @return true/false on success/failure. 
     */
    bool init(const yarp::sig::Vector &_x0, const yarp::sig::Matrix &_P0);

    /**
     * Predicts the next state vector given the current input. 
     * 
     * @param u Current input. 
     * 
     * @return Estimated state vector.
     */
    const yarp::sig::Vector& predict(const yarp::sig::Vector &u);

    /**
     * Predicts the next state vector. 
     * 
     * @return Estimated state vector.
     */
    const yarp::sig::Vector& predict();

    /**
     * Corrects the current estimation of the state vector given the
     * current measurement. 
     * 
     * @param z Current measurement. 
     * 
     * @return Estimated state vector.
     */
    const yarp::sig::Vector& correct(const yarp::sig::Vector &z);

    /**
     * Returns the estimated state vector given the current 
     * input and the current measurement by performing a prediction 
     * and then correcting the result. 
     * 
     * @param u Current input. 
     * @param z Current measurement. 
     * 
     * @return Estimated state vector.
     */
    const yarp::sig::Vector& filt(const yarp::sig::Vector &u, const yarp::sig::Vector &z);

    /**
     * Returns the estimated state vector given the current 
     * measurement by performing a prediction and then correcting 
     * the result. 
     * 
     * @param z Current measurement.
     * 
     * @return Estimated state vector.
     */
    const yarp::sig::Vector& filt(const yarp::sig::Vector &z);

    /**
     * Returns the estimated state.
     * 
     * @return Estimated state.
     */
    const yarp::sig::Vector& get_x() const { return x; }

    /**
     * Returns the estimated output.
     * 
     * @return Estimated output.
     */
    yarp::sig::Vector get_y() const;

    /**
     * Returns the estimated state covariance.
     * 
     * @return Estimated state covariance.
     */
    const yarp::sig::Matrix& get_P() const { return P; }

    /**
     * Returns the estimated measurement covariance.
     * 
     * @return Estimated measurement covariance.
     */
    const yarp::sig::Matrix& get_S() const { return S; }

    /**
     * Returns the validation gate.
     * @note The validation gate is meaningful only after 
     *       correction.
     * @see correct
     * @return validation gate.
     */
    double get_ValidationGate() const { return validationGate; }

    /**
     * Returns the Kalman gain matrix.
     * 
     * @return Kalman gain matrix.
     */
    const yarp::sig::Matrix& get_K() const { return K; }

    /**
     * Returns the state transition matrix.
     * 
     * @return State transition matrix.
     */
    const yarp::sig::Matrix& get_A() const { return A; }

    /**
     * Returns the input matrix.
     * 
     * @return Input matrix.
     */
    const yarp::sig::Matrix& get_B() const { return B; }

    /**
     * Returns the measurement matrix.
     * 
     * @return Measurement matrix.
     */
    const yarp::sig::Matrix& get_H() const { return H; }

    /**
     * Returns the process noise covariance matrix.
     * 
     * @return Process noise covariance matrix.
     */
    const yarp::sig::Matrix& get_Q() const { return Q; }

    /**
     * Returns the measurement noise covariance matrix.
     * 
     * @return Measurement noise covariance matrix.
     */
    const yarp::sig::Matrix& get_R() const { return R; }

    /**
     * Returns the state transition matrix. 
     *  
     * @param _A State transition matrix. 
     * @return true/false on success/failure.
     */
    bool set_A(const yarp::sig::Matrix &_A);

    /**
     * Returns the input matrix. 
     *  
     * @param _B Input matrix. 
     * @return true/false on success/failure.
     */
    bool set_B(const yarp::sig::Matrix &_B);

    /**
     * Returns the measurement transition matrix. 
     *  
     * @param _H Measurement matrix. 
     * @return true/false on success/failure.
     */
    bool set_H(const yarp::sig::Matrix &_H);

    /**
     * Returns the process noise covariance matrix. 
     *  
     * @param _Q Process noise covariance matrix. 
     * @return true/false on success/failure.
     */
    bool set_Q(const yarp::sig::Matrix &_Q);

    /**
     * Returns the measurement noise covariance matrix. 
     *  
     * @param _R Mearurement noise covariance matrix. 
     * @return true/false on success/failure.
     */
    bool set_R(const yarp::sig::Matrix &_R);
};

}

}

#endif




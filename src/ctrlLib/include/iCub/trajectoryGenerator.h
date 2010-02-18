/**
 * \defgroup trajectoryGenerator trajectoryGenerator
 *  
 * @ingroup ctrlLib
 *
 * Classes for Trajectory Generation
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __TRAJECTORYGENERATOR_H__
#define __TRAJECTORYGENERATOR_H__

#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <deque>

#include <iCub/pids.h>

#define MINJERK_STATE_RUNNING       0
#define MINJERK_STATE_REACHED       1


namespace ctrl
{

/**
* \ingroup trajectoryGenerator
*
* Generates minimum jerk trajectory
*/
class minJerkTrajGen
{
private:
    // Default constructor: not implemented.
    minJerkTrajGen();

protected:
    yarp::os::Semaphore *mutex;
    unsigned int dim;

    std::deque<ctrl::Integrator*> Int;
    yarp::sig::Vector x;
    yarp::sig::Vector v;
    yarp::sig::Vector a;

    yarp::sig::Matrix A;
    yarp::sig::Vector b;

    double TOld;
    double Ts;

    int state;

public:
    /**
    * Constructor. 
    * @param _Ts is the block sample time in seconds. 
    * @param x0 is the initial position. 
    */
    minJerkTrajGen(const double _Ts, const yarp::sig::Vector &x0);

    /**
    * Computes the trajectory. 
    * @param T the current execution time.  
    * @param xd the desired position to reach. 
    * @param fb the current position. 
    * @param tol the tolerance for the in-target checking. 
    */
    virtual void compute(const double T, const yarp::sig::Vector &xd, const yarp::sig::Vector &fb,
                         const double tol);

    /**
    * Returns the current reference position.
    * @return the current reference position.
    */
    virtual yarp::sig::Vector get_x();

    /**
    * Returns the current reference velocity.
    * @return the current reference velocity.
    */
    virtual yarp::sig::Vector get_v();

    /**
    * Returns the current reference acceleration.
    * @return the current reference acceleration.
    */
    virtual yarp::sig::Vector get_a();

    /**
    * Returns the internal state.
    * @return the internal state.
    */
    virtual int get_state() { return state; }

    /**
    * Destructor. 
    */
    ~minJerkTrajGen();
};

}

#endif




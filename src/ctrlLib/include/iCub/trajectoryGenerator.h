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

#define MINJERK_STATE_STARTING      0
#define MINJERK_STATE_FEEDBACK      1
#define MINJERK_STATE_REACHED       2

#define MINJERK_OPT_DISABLED        -1


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

    yarp::sig::Vector x;
    yarp::sig::Vector v;
    yarp::sig::Vector a;

    std::deque<yarp::sig::Vector> coeff;
    yarp::sig::Vector vtau;
    yarp::sig::Vector vData;
    yarp::sig::Vector aData;
    yarp::sig::Vector xdOld;
    double TOld;
    double Tmin;
    double fT;
    double t0;
    double t;
    double Ts;

    int state;

    virtual void calcCoeff(const double T, const yarp::sig::Vector &xd, const yarp::sig::Vector &fb);

public:
    /**
    * Constructor. 
    * @param _Ts is the block sample time in seconds. 
    * @param x0 is the initial position. 
    * @param _Tmin is the minimum time interval for reading the 
    *              feedback (if _Tmin<0, the minimum time interval
    *              is set equal to 10*_Ts).
    */
    minJerkTrajGen(const double _Ts, const yarp::sig::Vector &x0, const double _Tmin=MINJERK_OPT_DISABLED);

    /**
    * Computes the trajectory. 
    * @param T the current execution time.  
    * @param xd the desired position to reach. 
    * @param fb the current position. 
    * @param tol the tolerance for in-target checking 
    * @param dt the delta time expired from the previous call and 
    *           externally provided (if dt<0, internal sample time
    *           is used).
    */
    virtual void compute(const double T, const yarp::sig::Vector &xd, const yarp::sig::Vector &fb,
                         const double tol, const double dt=MINJERK_OPT_DISABLED);

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
    * Returns the current state.
    * @return the current state.
    */
    virtual int get_state() { return state; }

    /**
    * Destructor. 
    */
    ~minJerkTrajGen();
};

}

#endif




/**
 * \defgroup PIDs PIDs 
 *  
 * @ingroup ctrlLib
 *
 * Classes for PIDs
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __PIDS_H__
#define __PIDS_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <deque>

#include <iCub/ctrlMath.h>
#include <iCub/filters.h>


namespace ctrl
{

/**
* \ingroup PIDs
*
* A class for defining a saturated integrator based on Tustin 
* formula: 1/s => Ts/2*(z+1)/(z-1)
*/
class Integrator
{
private:
    // Default constructor: not implemented.
    Integrator();

protected:
    unsigned int dim;
    yarp::sig::Vector y;
    yarp::sig::Vector x_old;
    yarp::sig::Matrix lim;
    double Ts;
    bool   applySat;

    void   _allocate(const Integrator &I);
    yarp::sig::Vector saturate(const yarp::sig::Vector &v);

public:
    /**
    * Constructor. 
    * @param _Ts is the integrator sample time.
    * @param y0 is the initial value of the output vector.
    * @param _lim is a Nx2 matrix describing for each row i the 
    *             lower (1st column) and the upper limit (2nd
    *             column) of the ith component:
    *             _lim(i,1)<=output[i]<=_lim(i,2)
    * @note the saturation here is on by default 
    */
    Integrator(const double _Ts, const yarp::sig::Vector &y0, const yarp::sig::Matrix &_lim);

    /**
    * Constructor. 
    * @param _Ts is the integrator sample time.
    * @param y0 is the initial value of the output vector. 
    * @note the saturation here is off by default. 
    */
    Integrator(const double _Ts, const yarp::sig::Vector &y0);

    /**
    * Creates a new Integrator from an already existing object.
    * @param I is the Integrator to be copied.
    */
    Integrator(const Integrator &I) { _allocate(I); }

    /**
    * Copies a Integrator object into the current one.
    * @param I is a reference to an object of type Integrator.
    * @return a reference to the current object.
    */
    Integrator &operator=(const Integrator &I) { _allocate(I); return *this; }

    /**
    * Executes one-step integration of input vector. 
    * To be called each Ts seconds.
    * @param x is the input vector to be integrated.
    * @return the current output vector.
    */
    yarp::sig::Vector integrate(const yarp::sig::Vector &x);

    /**
    * Sets the saturation status.
    * @param _applySat if true then the saturation is applied 
    *                  (initialized as true).
    */
    void setSaturation(bool _applySat);

    /**
    * Returns the current saturation status. 
    * @return current saturation status.
    */
    bool getSaturation() { return applySat; }

    /**
    * Sets the output vector constraints matrix. 
    * @param _lim is the constraints matrix.
    */
    void setLim(const yarp::sig::Matrix &_lim);

    /**
    * Returns the constraints matrix. 
    * @return the constraints matrix.
    */
    yarp::sig::Matrix getLim() { return lim; }

    /**
    * Resets the internal state and sets the output vector to the 
    * given value. 
    * @param y0 is new value of output vector.
    */
    void reset(const yarp::sig::Vector &y0);

    /**
    * Returns the current output vector. 
    * @return the current output vector.
    */
    yarp::sig::Vector get() { return y; }
};


/**
* \ingroup PIDs
*
* General structure of parallel (non-interactive) PID. 
*  
* u = sat(P + I + D)
*  
* Components expressed in Laplace transform: 
*  
* - ep=Wp*ref-fb 
* - ei=Wi*ref-fb 
* - ed=Wd*ref-fb  
* - P=Kp*ep
* - I=(Ki*ei+(sat(u)-u)/Tt)/s
* - D=Kd*ed*s/(1+s*Td/N) [Td=Kd/Kp]
*/
class parallelPID
{
private:
    // Default constructor: not implemented.
    parallelPID();

protected:
    yarp::sig::Vector Kp;
    yarp::sig::Vector Ki;
    yarp::sig::Vector Kd;

    yarp::sig::Vector Wp;
    yarp::sig::Vector Wi;
    yarp::sig::Vector Wd;

    yarp::sig::Vector N;
    yarp::sig::Vector Tt;
    yarp::sig::Matrix satLim;

    yarp::sig::Vector P;
    yarp::sig::Vector I;
    yarp::sig::Vector D;
    yarp::sig::Vector u;
    yarp::sig::Vector uSat;

    unsigned int dim;
    double Ts;

    Integrator          *Int;
    std::deque<Filter*>  Der;

public:
    /**
    * Constructor. 
    * @param _Ts is the block sample time in seconds. 
    * @param _Kp are the proportional gains. 
    * @param _Ki are the integral gains. 
    * @param _Kd are the derivative gains. 
    * @param _Wp are the setpoint weigths for proportional part.
    * @param _Wi are the setpoint weigths for integral part.
    * @param _Wd are the setpoint weigths for derivative part.
    * @param _N  are derivative low-pass filter bandwidth (3 to 20, 
    *            typ. 10).
    * @param _Tt are anti-windup reset time (0.1 to 1 the value of
    *            Ti=Kp/Ki).
    * @param _satLim is the saturation thresholds matrix 
    *                (min_i=satLim(i,0), max_i=satLim(i,1)).
    */
    parallelPID(const double _Ts,
                const yarp::sig::Vector &_Kp, const yarp::sig::Vector &_Ki, const yarp::sig::Vector &_Kd,
                const yarp::sig::Vector &_Wp, const yarp::sig::Vector &_Wi, const yarp::sig::Vector &_Wd,
                const yarp::sig::Vector &_N,  const yarp::sig::Vector &_Tt, const yarp::sig::Matrix &_satLim);

    /**
    * Computes the PID output.
    * @param ref the actual reference to track. 
    * @param fb the actual plant feedback. 
    * @return the actual PID output.  
    */
    virtual yarp::sig::Vector compute(const yarp::sig::Vector &ref, const yarp::sig::Vector &fb);

    /**
    * Resets the internal state of integral and derivative part.
    */
    virtual void reset();

    /**
    * Destructor. 
    */
    ~parallelPID();
};


/**
* \ingroup PIDs
*
* General structure of series (interactive) PID. 
*  
* u = sat((P + I) * (1 + D) * (ref-fb))
*  
* Components expressed in Laplace transform: 
*  
* - P=Kp
* - I=Kp/(Ti*s)
* - D=Kd*s/(1+s*Td/N) [Td=Kd/Kp]
*/
class seriesPID
{
private:
    // Default constructor: not implemented.
    seriesPID();

protected:
    yarp::sig::Vector Kp;
    yarp::sig::Vector Ti;
    yarp::sig::Vector Kd;

    yarp::sig::Vector N;
    yarp::sig::Matrix satLim;    

    yarp::sig::Vector e;
    yarp::sig::Vector P;
    yarp::sig::Vector I;
    yarp::sig::Vector D;
    yarp::sig::Vector u;
    yarp::sig::Vector uSat;

    unsigned int dim;
    double Ts;

    std::deque<Filter*> Int;
    std::deque<Filter*> Der;

public:
    /**
    * Constructor. 
    * @param _Ts is the block sample time in seconds. 
    * @param _Kp are the proportional gains. 
    * @param _Ti are the integral time constants (so that integral 
    *            part cannot be switched off).
    * @param _Kd are the derivative gains. 
    * @param _N  are derivative low-pass filter bandwidth (3 to 20, 
    *            typ. 10).
    * @param _satLim is the saturation thresholds matrix 
    *                (min_i=satLim(i,0), max_i=satLim(i,1)).
    */
    seriesPID(const double _Ts,
              const yarp::sig::Vector &_Kp, const yarp::sig::Vector &_Ti, const yarp::sig::Vector &_Kd,
              const yarp::sig::Vector &_N,  const yarp::sig::Matrix &_satLim);

    /**
    * Computes the PID output.
    * @param ref the actual reference to track. 
    * @param fb the actual plant feedback. 
    * @return the actual PID output.  
    */
    virtual yarp::sig::Vector compute(const yarp::sig::Vector &ref, const yarp::sig::Vector &fb);

    /**
    * Resets the internal state of integral and derivative part.
    */
    virtual void reset();

    /**
    * Destructor. 
    */
    ~seriesPID();
};

}

#endif




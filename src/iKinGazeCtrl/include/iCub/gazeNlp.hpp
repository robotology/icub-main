
#ifndef __GAZENLP_H__
#define __GAZENLP_H__

#include <yarp/sig/Vector.h>

#include <iCub/iKinInv.h>
#include <iCub/iKinIpOpt.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;
using namespace iKin;


// Describe the kinematic of the straight line
// coming out from the point located between eyes.
class iCubHeadCenter : public iCubEye
{
protected:
    virtual void allocate(const string &_type);

public:
    iCubHeadCenter()                           { allocate("right"); }
    iCubHeadCenter(const iCubHeadCenter &head) { clone(head);       }
};


// Describe the nonlinear problem of aligning two vectors
// in counterphase for controlling neck movements.
class HeadCenter_NLP : public iKin_NLP
{
private:
    HeadCenter_NLP(const HeadCenter_NLP&);
    HeadCenter_NLP &operator=(const HeadCenter_NLP&);

protected:
    iKinChain dummyChain;
    Vector    dummyVector;

    Matrix Hxd;
    Matrix GeoJacobP;
    Matrix AnaJacobZ;

    double mod;
    double cosAng;

    virtual void computeQuantities(const Ipopt::Number *x);

public:
    HeadCenter_NLP(iKinChain &c, const Vector &_q0, Vector &_xd,
                   iKinLinIneqConstr &_LIC, bool *_exhalt=NULL) :
                   iKin_NLP(c,IKINCTRL_POSE_XYZ,_q0,_xd,
                            0.0,dummyChain,dummyVector,dummyVector,
                            0.0,dummyVector,dummyVector,
                            _LIC,_exhalt) { }

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);
};


// Compute fixation point position and Jacobian wrt eyes (tilt,pan,vergence)
// Return true if division by zero is detected
bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR, Vector &fp, Matrix &J);


// Describe the nonlinear problem of eyes handling.
// DOFs=tilt+pan+vergence.
class Eyes_NLP : public iKin_NLP
{
private:
    Eyes_NLP(const Eyes_NLP&);
    Eyes_NLP &operator=(const Eyes_NLP&);

protected:
    iKinChain dummyChain;
    Vector    dummyVector;

    iKinChain &eyeL;
    iKinChain &eyeR;

    Vector qL;
    Vector qR;
    Vector fp;

    Matrix J;

    double subsStartVerg0;
    bool   divByZero;

    void         reinforceBounds();
    virtual void computeQuantities(const Ipopt::Number *x);

public:
    Eyes_NLP(iKinChain &_eyeL, iKinChain &_eyeR, const Vector &_q0, Vector &_xd,
             iKinLinIneqConstr &_LIC, bool *_exhalt=NULL);

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Ipopt::Number* lambda);
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Index m, Ipopt::Number* g);
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                            Ipopt::Number* values);
    virtual void finalize_solution(Ipopt::SolverReturn status,
                                   Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                   Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);
};


// Solve through IPOPT the nonlinear problem 
class GazeIpOptMin : public iKinIpOptMin
{
private:
    GazeIpOptMin();
    GazeIpOptMin(const GazeIpOptMin&);
    GazeIpOptMin &operator=(const GazeIpOptMin&);

protected:
    iKinChain &chain2;
    bool neckType;

public:
    GazeIpOptMin(const string &type, iKinChain &c1, iKinChain &c2,
                 const double tol, const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0);
    void set_ctrlPose(unsigned int _ctrlPose) { }
    void setHessianOpt(const bool useHessian) { }   // Hessian not implemented
    virtual Vector solve(const Vector &q0, Vector &xd,
                         Ipopt::ApplicationReturnStatus *exit_code=NULL, bool *exhalt=NULL,
                         iKinIterateCallback *iterate=NULL);
};


#endif



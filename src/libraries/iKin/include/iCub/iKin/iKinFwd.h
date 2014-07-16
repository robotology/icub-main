/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \defgroup iKin iKin
 *  
 * @ingroup icub_libraries 
 *  
 * Classes for forward-inverse kinematics of serial-links chains
 * of revolute joints and iCub limbs 
 *  
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \section dep_sec Dependencies 
 * - ctrlLib 
 * - IPOPT: see the <a
 *   href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>.
 *  
 * \note <b>If you're going to use iKin for your work, please
 *       quote it within any resulting publication</b>: <i>
 *       Pattacini U., Modular Cartesian Controllers for
 *       Humanoid Robots: Design and Implementation on the iCub,
 *       Ph.D. Dissertation, RBCS, Istituto Italiano di
 *       Tecnologia, 2011</i>.
 *  
 * \author Ugo Pattacini 
 *  
 * \defgroup iKinFwd iKinFwd 
 *  
 * @ingroup iKin  
 *
 * Classes for forward kinematics of serial-links chains 
 * and iCub limbs
 *
 * Date: first release 16/06/2008
 *
 * \author Ugo Pattacini 
 *  
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */ 

#ifndef __IKINFWD_H__
#define __IKINFWD_H__

#include <string>
#include <deque>

#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>


namespace iCub
{

namespace iKin
{

void notImplemented(const unsigned int verbose);

/**
* \ingroup iKinFwd
*
* A Base class for defining a Link with standard 
* Denavit-Hartenberg convention. 
*  
* \note This class implements revolute joints only, as they are 
*       the unique type of joints used in humanoid robotics.
*/
class iKinLink
{
protected:
    double       A;
    double       D;
    double       Alpha;
    double       Offset;
    double       c_alpha;
    double       s_alpha;
    double       Min;
    double       Max;
    double       Ang;
    bool         blocked;
    bool         cumulative;
    bool         constrained;
    unsigned int verbose;

    yarp::sig::Matrix H;
    yarp::sig::Matrix cumH;
    yarp::sig::Matrix DnH;

    const yarp::sig::Matrix zeros1x1;
    const yarp::sig::Vector zeros1;

    friend class iKinChain;

    // Default constructor: not implemented.
    iKinLink();

    virtual void clone(const iKinLink &l);    
    bool         isCumulative()     { return cumulative;          }
    void         block()            { blocked=true;               }
    void         block(double _Ang) { setAng(_Ang); blocked=true; }
    void         release()          { blocked=false;              }
    void         rmCumH()           { cumulative=false;           }
    void         addCumH(const yarp::sig::Matrix &_cumH);

public:
    /**
    * Constructor. 
    * @param _A is the Link length.
    * @param _D is the Link offset. 
    * @param _Alpha is the Link twist. 
    * @param _Offset is the joint angle offset in [-pi,pi]
    * @param _Min is the joint angle lower bound in [-pi,pi] (-pi by
    *             default).
    * @param _Max is the joint angle higher bound in [-pi,pi] (pi by
    *             default).
    */
    iKinLink(double _A, double _D, double _Alpha, double _Offset,
             double _Min=-M_PI, double _Max=M_PI);

    /**
    * Creates a new Link from an already existing Link object.
    * @param l is the Link to be copied.
    */
    iKinLink(const iKinLink &l);

    /**
    * Copies a Link object into the current one.
    * @param l is a reference to an object of type iKinLink.
    * @return a reference to the current object.
    */
    iKinLink &operator=(const iKinLink &l);

    /**
    * Sets the constraint status.
    * @param _constrained if true the joint angle cannot be set out 
    *                     of its limits (initialized as true).
    */
    void setConstraint(bool _constrained) { constrained=_constrained; }

    /**
    * Returns the constraint status.
    * @return current constraint status.
    */
    bool getConstraint() const { return constrained; }

    /**
    * Sets Link verbosity level.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Link verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() const { return verbose; }

    /**
    * Returns the Link blocking status.
    * @return true if link is blocked.
    */
    bool isBlocked() const { return blocked; }

    /**
    * Returns the Link length A.
    * @return Link length A.
    */
    double getA() const { return A; }

    /**
    * Sets the Link length A. 
    * @param new Link length _A. 
    */
    void setA(const double _A) { A=_A; }

    /**
    * Returns the Link offset D.
    * @return Link offset D.
    */
    double getD() const { return D; }

    /**
    * Sets the Link offset D. 
    * @param new Link offset _D. 
    */
    void setD(const double _D);

    /**
    * Returns the Link twist Alpha.
    * @return Link twist Alpha.
    */
    double getAlpha() const { return Alpha; }

    /**
    * Sets the Link twist Alpha. 
    * @param new Link twist _Alpha. 
    */
    void setAlpha(const double _Alpha);

    /**
    * Returns the joint angle offset.
    * @return joint angle offset.
    */
    double getOffset() const { return Offset; }

    /**
    * Sets the joint angle offset. 
    * @param new joint angle offset _Offset. 
    */
    void setOffset(const double _Offset) { Offset=_Offset; }

    /**
    * Returns the joint angle lower bound.
    * @return joint angle lower bound.
    */
    double getMin() const { return Min; }

    /**
    * Sets the joint angle lower bound.
    * @param _Min is the joint angle lower bound in [-pi,pi].
    */
    void setMin(const double _Min);

    /**
    * Returns the joint angle upper bound.
    * @return joint angle upper bound.
    */
    double getMax() const { return Max; }

    /**
    * Sets the joint angle higher bound.
    * @param _Max is the joint angle higher bound in [-pi,pi].
    */
    void setMax(const double _Max);

    /**
    * Returns the current joint angle value.
    * @return current joint angle value.
    */
    double getAng() const { return Ang; }

    /**
    * Sets the joint angle value. 
    * @param _Ang new value for joint angle.
    * @return actual joint angle value (constraints are evaluated).
    */
    double setAng(double _Ang);

    /**
    * Computes the homogeneous transformation matrix H of the Link.
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain, i.e. H=Hn
    *                   and not H=Hn*Hn-1 (false by default)
    * @return a reference to H.
    */
    yarp::sig::Matrix getH(bool c_override=false);

    /**
    * Same as getH() with specification of new joint angle position. 
    * @param _Ang is the new joint angle position. 
    * @param c_override.
    * @return a reference to H. 
    * @see getH 
    */
    yarp::sig::Matrix getH(double _Ang, bool c_override=false);

    /**
    * Computes the derivative of order n of the homogeneous 
    * transformation matrix H with respect to the joint angle. 
    * @param n is the order of the derivative (1 by default)
    * @param c_override if true avoid accumulating the computation 
    *                   of previous links in the chain (false by
    *                   default).
    * @return a reference to derivative of order n.
    */
    yarp::sig::Matrix getDnH(unsigned int n=1, bool c_override=false);

    /**
    * Default destructor. 
    */
    virtual ~iKinLink() { }

    // void methods for iDyn
    // set
    virtual double setDAng(const double)                                          { notImplemented(verbose); return 0.0;   }
    virtual double setD2Ang(const double)                                         { notImplemented(verbose); return 0.0;   }
    virtual void   setPosVelAcc(const double, const double, const double)         { notImplemented(verbose);               }
    virtual bool   setDynamicParameters(const double, const yarp::sig::Matrix&,
                                        const yarp::sig::Matrix&, const double,
                                        const double, const double, const double) { notImplemented(verbose); return false; }
    virtual bool   setDynamicParameters(const double, const yarp::sig::Matrix&,
                                        const yarp::sig::Matrix&)                 { notImplemented(verbose); return false; }
    virtual bool   setStaticParameters(const double, const yarp::sig::Matrix&)    { notImplemented(verbose); return false; }
    virtual bool   setInertia(const yarp::sig::Matrix &)                          { notImplemented(verbose); return false; }
    virtual void   setMass(const double)                                          { notImplemented(verbose);               }
    virtual bool   setCOM(const yarp::sig::Matrix&)                               { notImplemented(verbose); return false; }
    virtual bool   setCOM(const yarp::sig::Vector&)                               { notImplemented(verbose); return false; }
    virtual bool   setForce(const yarp::sig::Vector&, const yarp::sig::Vector&)   { notImplemented(verbose); return false; }
    virtual bool   setMoment(const yarp::sig::Vector&)                            { notImplemented(verbose); return false; }
    virtual void   setTorque(const double)                                        { notImplemented(verbose);               }

    // get
    virtual const  yarp::sig::Matrix &getInertia() const { notImplemented(verbose); return zeros1x1; }
    virtual double                    getMass()    const { notImplemented(verbose); return 0.0;      }
    virtual double                    getIm()      const { notImplemented(verbose); return 0.0;      }
    virtual double                    getKr()      const { notImplemented(verbose); return 0.0;      }
    virtual double                    getFs()      const { notImplemented(verbose); return 0.0;      }
    virtual double                    getFv()      const { notImplemented(verbose); return 0.0;      }
    virtual const  yarp::sig::Matrix &getCOM()     const { notImplemented(verbose); return zeros1x1; }
    virtual double                    getDAng()    const { notImplemented(verbose); return 0.0;      }
    virtual double                    getD2Ang()   const { notImplemented(verbose); return 0.0;      }
    virtual const  yarp::sig::Matrix &getR()             { notImplemented(verbose); return zeros1x1; }
    virtual const  yarp::sig::Matrix &getRC()            { notImplemented(verbose); return zeros1x1; }
    virtual const  yarp::sig::Vector &getr()             { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getrC()            { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getW()       const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getdW()      const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getdWM()     const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getLinAcc()  const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getLinAccC() const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getLinVel()  const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getLinVelC() const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getForce()   const { notImplemented(verbose); return zeros1;   }
    virtual const  yarp::sig::Vector &getMoment()  const { notImplemented(verbose); return zeros1;   }
    virtual double                    getTorque()  const { notImplemented(verbose); return 0.0;      }
};


/**
* \ingroup iKinFwd
*
* A Base class for defining a Serial Link Chain. 
*/
class iKinChain
{
protected:
    unsigned int      N;
    unsigned int      DOF;
    unsigned int      verbose;
    yarp::sig::Matrix H0;
    yarp::sig::Matrix HN;
    yarp::sig::Vector curr_q;

    std::deque<iKinLink*> allList;
    std::deque<iKinLink*> quickList;

    std::deque<unsigned int> hash;
    std::deque<unsigned int> hash_dof;

    yarp::sig::Matrix hess_J;
    yarp::sig::Matrix hess_Jlnk;

    virtual void clone(const iKinChain &c);
    virtual void build();
    virtual void dispose();

    yarp::sig::Vector RotAng(const yarp::sig::Matrix &R);
    yarp::sig::Vector dRotAng(const yarp::sig::Matrix &R, const yarp::sig::Matrix &dR);
    yarp::sig::Vector d2RotAng(const yarp::sig::Matrix &R, const yarp::sig::Matrix &dRi,
                               const yarp::sig::Matrix &dRj, const yarp::sig::Matrix &d2R);

public:
    /**
    * Default constructor. 
    */
    iKinChain();

    /**
    * Creates a new Chain from an already existing Chain object.
    * @param c is the Chain to be copied.
    */
    iKinChain(const iKinChain &c);

    /**
    * Copies a Chain object into the current one.
    * @param c is a reference to an object of type iKinChain.
    * @return a reference to the current object.
    */
    iKinChain &operator=(const iKinChain &c);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added.
    * @return a reference to the current object.
    */
    iKinChain &operator<<(iKinLink &l);

    /**
    * Removes a Link from the bottom of the Chain.
    * @return a reference to the current object.
    */
    iKinChain &operator--(int);

    /**
    * Returns a reference to the ith Link of the Chain.
    * @param i is the Link number within the Chain.
    * @return a reference to the ith Link object.
    */
    iKinLink &operator[](const unsigned int i) { return *allList[i]; }

    /**
    * Returns a reference to the ith Link of the Chain considering 
    * only those Links related to DOF.
    * @param i is the Link number within the Chain (in DOF order)
    * @return a reference to the ith Link object.
    */
    iKinLink &operator()(const unsigned int i) { return *quickList[hash_dof[i]]; }

    /**
    * Adds a Link at the position ith within the Chain. 
    * @param i is the ith position where the Link is to be added.
    * @param l is the Link to be added.
    * @return true if successful (e.g. param i is in range).
    */
    bool addLink(const unsigned int i, iKinLink &l);

    /**
    * Removes the ith Link from the Chain. 
    * @param i is the ith position from which the Link is to be 
    *          removed.
    * @return true if successful (e.g. param i is in range).
    */
    bool rmLink(const unsigned int i);

    /**
    * Adds a Link at the bottom of the Chain.
    * @param l is the Link to be added. 
    * @see operator<<
    */
    void pushLink(iKinLink &l);

    /**
    * Removes all Links.
    */
    void clear();

    /**
    * Removes a Link from the bottom of the Chain. 
    * @see operator--()
    */
    void popLink();

    /**
    * Blocks the ith Link at the a certain value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link number. 
    * @param Ang is the value of joint angle to which the Link is 
    *            blocked.
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i, double Ang);

    /**
    * Blocks the ith Link at the current value of its joint angle.
    * Chain DOF reduced by one. 
    * @param i is the Link number. 
    * @return true if successful (e.g. param i is in range).
    */
    bool blockLink(const unsigned int i) { return blockLink(i,getAng(i)); }

    /**
    * Changes the value of the ith blocked Link. Avoid the overhead 
    * required for DOFs handling. 
    * @param i is the Link number. 
    * @param Ang is the new value of joint angle to which the Link 
    *            is blocked.
    * @return true if successful (e.g. param i is in range and the 
    *         ith Link was already blocked).
    */
    bool setBlockingValue(const unsigned int i, double Ang);

    /**
    * Releases the ith Link. 
    * Chain DOF augmented by one.
    * @param i is the Link number. 
    * @return true if successful (e.g. param i is in range).
    */
    bool releaseLink(const unsigned int i);

    /**
    * Queries whether the ith Link is blocked.
    * @param i is the Link number. 
    * @return true if blocked && (param i is in range).
    */
    bool isLinkBlocked(const unsigned int i);

    /**
    * Sets the constraint status of all chain links.
    * @param _constrained is the new constraint status. 
    */
    void setAllConstraints(bool _constrained);

    /**
    * Sets the constraint status of ith link.
    * @param _constrained is the new constraint status. 
    */
    void setConstraint(unsigned int i, bool _constrained) { allList[i]->setConstraint(_constrained); }

    /**
    * Returns the constraint status of ith link. 
    * @return current constraint status.
    */
    bool getConstraint(unsigned int i) { return allList[i]->getConstraint(); }

    /**
    * Sets the verbosity level of all Links belonging to the Chain.
    * @param _verbose is the verbosity level. 
    */
    void setAllLinkVerbosity(unsigned int _verbose);

    /**
    * Sets the verbosity level of the Chain.
    * @param _verbose is a integer number which progressively 
    *                 enables different levels of warning messages.
    *                 The larger this value the more detailed is the
    *                 output.
    */
    void setVerbosity(unsigned int _verbose) { verbose=_verbose; }

    /**
    * Returns the current Chain verbosity level.
    * @return Link verbosity level.
    */
    unsigned int getVerbosity() const { return verbose; }

    /**
    * Returns the number of Links belonging to the Chain.
    * @return number of Links.
    */
    unsigned int getN() const { return N; }

    /**
    * Returns the current number of Chain's DOF. 
    * DOF=N-M, where N=# of Links, M=# of blocked Links. 
    * @return number of DOF.
    */
    unsigned int getDOF() const { return DOF; }

    /**
    * Returns H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @return H0
    */
    yarp::sig::Matrix getH0() const { return H0; }

    /**
    * Sets H0, the rigid roto-translation matrix from the root 
    * reference frame to the 0th frame. 
    * @param H0 
    * @return true if succeed, false otherwise. 
    */
    bool setH0(const yarp::sig::Matrix &_H0);

    /**
    * Returns HN, the rigid roto-translation matrix from the Nth
    * frame to the end-effector. 
    * @return HN
    */
    yarp::sig::Matrix getHN() const { return HN; }

    /**
    * Sets HN, the rigid roto-translation matrix from the Nth frame 
    * to the end-effector. 
    * @param HN 
    * @return true if succeed, false otherwise. 
    */
    bool setHN(const yarp::sig::Matrix &_HN);

    /**
    * Sets the free joint angles to values of q[i].
    * @param q is a vector containing values for DOF.
    * @return the actual DOF values (angles constraints are 
    *         evaluated).
    */
    yarp::sig::Vector setAng(const yarp::sig::Vector &q);

    /**
    * Returns the current free joint angles values.
    * @return the actual DOF values.
    */
    yarp::sig::Vector getAng();

    /**
    * Sets the ith joint angle. 
    * @param i is the Link number. 
    * @param Ang the new angle's value. 
    * @return current ith joint angle (angle constraint is 
    *         evaluated).
    */
    double setAng(const unsigned int i, double _Ang);

    /**
    * Returns the current angle of ith joint. 
    * @param i is the Link number.  
    * @return current ith joint angle.
    */
    double getAng(const unsigned int i);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the ith frame in Denavit-Hartenberg 
    * notation. The second parameter if true enables the spannig 
    * over the full set of links, i.e. the blocked links as well. 
    * @param i is the Link number. 
    * @param allLink if true enables the spanning over the full set 
    *                of links (false by default).
    * @return Hi 
    */
    yarp::sig::Matrix getH(const unsigned int i, const bool allLink=false);

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the end-effector frame in 
    * Denavit-Hartenberg notation (HN is taken into account). 
    * @return H(N-1)*HN
    */
    yarp::sig::Matrix getH();

    /**
    * Returns the rigid roto-translation matrix from the root 
    * reference frame to the end-effector frame in 
    * Denavit-Hartenberg notation (HN is taken into account). 
    * @param q is the vector of new DOF values.  
    * @return H(N-1)*HN
    */
    yarp::sig::Matrix getH(const yarp::sig::Vector &q);

    /**
    * Returns the coordinates of ith Link. Two notations are
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param i is the Link number. 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the ith Link Pose.
    */
    yarp::sig::Vector Pose(const unsigned int i, const bool axisRep=true);

    /**
    * Returns the 3D position coordinates of ith Link. 
    * @param i is the Link number. 
    * @return the ith Link Position.
    */
    yarp::sig::Vector Position(const unsigned int i);

    /**
    * Returns the coordinates of end-effector. Two notations are 
    * provided: the first with Euler Angles (XYZ form=>6x1 output 
    * vector) and second with axis/angle representation 
    * (default=>7x1 output vector). 
    * @param axisRep if true returns the axis/angle notation. 
    * @return the end-effector pose.
    */
    yarp::sig::Vector EndEffPose(const bool axisRep=true);

    /**
    * Returns the coordinates of end-effector computed in q. Two
    * notations are provided: the first with Euler Angles (XYZ 
    * form=>6x1 output vector) and second with axis/angle 
    * representation (default=>7x1 output vector). 
    * @param q is the vector of new DOF values. 
    * @param axisRep if true returns the axis/angle notation.  
    * @return the end-effector pose.
    */
    yarp::sig::Vector EndEffPose(const yarp::sig::Vector &q, const bool axisRep=true);

    /**
    * Returns the 3D coordinates of end-effector position.
    * @return the end-effector position.
    */
    yarp::sig::Vector EndEffPosition();

    /**
    * Returns the 3D coordinates of end-effector position computed in q.
    * @param q is the vector of new DOF values. 
    * @return the end-effector position.
    */
    yarp::sig::Vector EndEffPosition(const yarp::sig::Vector &q);

    /**
    * Returns the analitical Jacobian of the ith link.
    * @param i is the Link number. 
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(const unsigned int i, unsigned int col);

    /**
    * Returns the analitical Jacobian of the end-effector.
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(unsigned int col=3);

    /**
    * Returns the analitical Jacobian of the end-effector computed 
    * in q. 
    * @param q is the vector of new DOF values. 
    * @param col selects the part of the derived homogeneous matrix 
    *            to be put in the upper side of the Jacobian
    *            matrix: 0 => x, 1 => y, 2 => z, 3 => p (default)
    * @return the analitical Jacobian.
    */
    yarp::sig::Matrix AnaJacobian(const yarp::sig::Vector &q, unsigned int col=3);

    /**
    * Returns the geometric Jacobian of the ith link. 
    * @param i is the Link number.
    * @return the 6x(i-1) geometric Jacobian matrix.
    * @note All the links are considered.
    */
    yarp::sig::Matrix GeoJacobian(const unsigned int i);

    /**
    * Returns the geometric Jacobian of the end-effector.
    * @return the 6xDOF geometric Jacobian matrix.
    * @note The blocked links are not considered.
    */
    yarp::sig::Matrix GeoJacobian();

    /**
    * Returns the geometric Jacobian of the end-effector computed in
    * q. 
    * @param q is the vector of new DOF values. 
    * @return the geometric Jacobian.
    * @note The blocked links are not considered.
    */
    yarp::sig::Matrix GeoJacobian(const yarp::sig::Vector &q);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the DOF couple. 
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector Hessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Prepares computation for a successive call to 
    * fastHessian_ij(). 
    * @see fastHessian_ij
    */
    void prepareForHessian();

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the DOF couple. 
    * <i>Fast Version</i>: to be used in conjunction with 
    * prepareForHessian(). 
    * @param i is the index of the first DOF. 
    * @param j is the index of the second DOF.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    * @note It is advisable to use this version when successive 
    * computations with different indexes values are needed. 
    * @see prepareForHessian
    */
    yarp::sig::Vector fastHessian_ij(const unsigned int i, const unsigned int j);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the couple of 
    * links. 
    * @param lnk is the Link number up to which consider the 
    *            computation.
    * @param i is the index of the first link. 
    * @param j is the index of the second link.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    */
    yarp::sig::Vector Hessian_ij(const unsigned int lnk, const unsigned int i,
                                 const unsigned int j);

    /**
    * Prepares computation for a successive call to 
    * fastHessian_ij() (link version). 
    * @param lnk is the Link number up to which consider the 
    *            computation.
    * @see fastHessian_ij 
    */
    void prepareForHessian(const unsigned int lnk);

    /**
    * Returns the 6x1 vector \f$ 
    * \partial{^2}F\left(q\right)/\partial q_i \partial q_j, \f$
    * where \f$ F\left(q\right) \f$ is the forward kinematic 
    * function and \f$ \left(q_i,q_j\right) \f$ is the couple of 
    * links. 
    * <i>Fast Version</i>: to be used in conjunction with 
    * prepareForHessian(lnk). 
    * @param lnk is the Link number up to which consider the 
    *            computation. 
    * @param i is the index of the first link. 
    * @param j is the index of the second link.
    * @return the 6x1 vector \f$ 
    *         \partial{^2}F\left(q\right)/\partial q_i \partial q_j.
    *                 \f$
    * @note It is advisable to use this version when successive 
    * computations with different indexes values are needed. 
    * @see prepareForHessian
    */
    yarp::sig::Vector fastHessian_ij(const unsigned int lnk, const unsigned int i,
                                     const unsigned int j);

    /**
    * Compute the time derivative of the geometric Jacobian.
    * @param dq the joint velocities.
    * @return the 6xDOF matrix \f$ 
    *         \partial{^2}F\left(q\right)/\partial t \partial q.
    *                 \f$
    */
    yarp::sig::Matrix DJacobian(const yarp::sig::Vector &dq);

    /**
    * Compute the time derivative of the geometric Jacobian
    * (link version).
    * @param lnk is the Link number up to which consider the 
    *            computation. 
    * @param dq the (lnk-1)x1 joint velocity vector.
    * @return the 6x(lnk-1) matrix \f$ 
    *         \partial{^2}F\left(q\right)/\partial t \partial q.
    *                 \f$
    */
    yarp::sig::Matrix DJacobian(const unsigned int lnk, const yarp::sig::Vector &dq);

    /**
    * Destructor. 
    */
    virtual ~iKinChain();
};


/**
* \ingroup iKinFwd
*
* A class for defining generic Limb.
*/
class iKinLimb : public iKinChain
{
protected:
    std::deque<iKinLink*> linkList;
    std::string           type;
    bool                  configured;

    virtual void getMatrixFromProperties(yarp::os::Property &options,
                                         const std::string &tag, yarp::sig::Matrix &H);
    virtual void setMatrixToProperties(yarp::os::Property &options,
                                       const std::string &tag, yarp::sig::Matrix &H);
    virtual void allocate(const std::string &_type);
    virtual void clone(const iKinLimb &limb);
    virtual void dispose();

    // make the following methods protected in order to prevent user from changing
    // too easily the internal structure of the chain;
    // to get access anyway to these hidden methods, user can rely on asChain()
    iKinChain &operator=(const iKinChain &c)              { return iKinChain::operator=(c);  }
    iKinChain &operator<<(iKinLink &l)                    { return iKinChain::operator<<(l); }
    iKinChain &operator--(int)                            { return iKinChain::operator--(0); }
    iKinLink  &operator[](const unsigned int i)           { return iKinChain::operator[](i); }
    iKinLink  &operator()(const unsigned int i)           { return iKinChain::operator()(i); }
    bool       addLink(const unsigned int i, iKinLink &l) { return iKinChain::addLink(i,l);  }
    bool       rmLink(const unsigned int i)               { return iKinChain::rmLink(i);     }
    void       pushLink(iKinLink &l)                      { iKinChain::pushLink(l);          }
    void       clear()                                    { iKinChain::clear();              }
    void       popLink()                                  { iKinChain::popLink();            }
    void       pushLink(iKinLink *pl);

public:
    /**
    * Default constructor. 
    */
    iKinLimb();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" limb
    */
    iKinLimb(const std::string &_type);

    /**
    * Creates a new Limb from an already existing Limb object.
    * @param limb is the Limb to be copied.
    */
    iKinLimb(const iKinLimb &limb);

    /**
    * Creates a new Limb from a list of properties wherein links 
    * parameters are specified. 
    * @param options is the list of links properties. 
    * @see fromLinksProperties
    */
    iKinLimb(const yarp::os::Property &options);

    /**
    * Initializes the Limb from a list of properties wherein links 
    * parameters are specified. 
    * @param options is the list of links properties. 
    *  
    * @note Available options are: 
    *  
    * \b type <string>: specifies the limb handedness [left/right] 
    *    (default=right).
    *  
    * \b H0 <list of 4x4 doubles (per rows)>: specifies the rigid 
    *    roto-translation matrix from the root reference frame to
    *    the 0th frame (default=eye(4,4)).
    *  
    * \b HN <list of 4x4 doubles (per rows)>: specifies the rigid 
    *    roto-translation matrix from the Nth frame to the
    *    end-effector (default=eye(4,4)).
    *  
    * \b numLinks <int>: specifies the expected number of links.
    *  
    * \b A <double>: specifies the link length [m] (default=0.0).
    *  
    * \b D <double>: specifies the link offset [m] (default=0.0).
    *  
    * \b alpha <double>: specifies the link twist [deg] 
    *    (default=0.0).
    *  
    * \b offset <double>: specifies the joint angle offset [deg] 
    *    (default=0.0).
    *  
    * \b min <double>: specifies the joint angle lower bound [deg]
    *    (default=0.0).
    *  
    * \b max <double>: specifies the joint angle upper bound [deg] 
    *    (default=0.0).
    *  
    * \b blocked <double>: blocks the link at the specified value 
    *    [deg] (default=released).
    *  
    * @note The list should look like as the following: 
    *  
    * @code 
    * type right 
    * H0 (1.0 2.0 3.0 ...) 
    * HN (1.0 2.0 3.0 ...) 
    * numLinks 4 
    * link_0 (option1 value1) (option2 value2) ... 
    * link_1 (option1 value1) ... 
    * ... 
    * @endcode 
    *  
    * @note Please note how the link options (e.g. A, D, alpha ...)
    *       are to be given using the group identifiers link_#.
    *  
    * @return true iff correctly configured. 
    */
    bool fromLinksProperties(const yarp::os::Property &options);

    /**
    * Provides the links attributes listed in a property object.
    * @param options is the list of links properties. 
    * @return true iff successful.  
    */
    bool toLinksProperties(yarp::os::Property &options);

    /**
    * Checks if the limb has been properly configured.
    * @return true iff correctly configured.
    */
    bool isValid() const { return configured; }

    /**
    * Copies a Limb object into the current one.
    * @param limb is a reference to an object of type iKinLimb.
    * @return a reference to the current object.
    */
    iKinLimb &operator=(const iKinLimb &limb);

    /**
    * Returns a pointer to the Limb seen as Chain object. 
    * Useful to to operate on the Links of Limb.
    * @return a pointer to a Chain object with the same Links of 
    *         Limb.
    */
    iKinChain *asChain() { return static_cast<iKinChain*>(this); }

    /**
    * Returns the Limb type as a string. 
    * @return the type as a string {"left", "right"}.
    */
    std::string getType() const { return type; }

    /**
    * Alignes the Limb joints bounds with current values set aboard 
    * the robot. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Limb limits.
    * @return true/false on success/failure. 
    *  
    * @note This method is empty in iKinLimb because it's 
    * limb-specific: see the implementations for iCubLimbs. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*>&) { notImplemented(verbose); return true; }

    /**
    * Destructor. 
    */
    virtual ~iKinLimb();
};


/**
* \ingroup iKinFwd
*
* A class for defining the iCub Arm.
*/
class iCubArm : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubArm();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" arm. Further available options are
    *              "[left|right]_v[1|1.7|2]".
    */
    iCubArm(const std::string &_type);

    /**
    * Creates a new Arm from an already existing Arm object.
    * @param arm is the Arm to be copied.
    */
    iCubArm(const iCubArm &arm);

    /**
    * Alignes the Arm joints bounds with current values set aboard 
    * the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Torso and the Arm limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};


/**
* \ingroup iKinFwd
*
* A class for defining the iCub Finger. \n 
* The outcomes of forward kinematics are relative to the 
* reference frame attached to the corresponding iCubArm 
* end-effector. 
*/
class iCubFinger : public iKinLimb
{
protected:
    std::string hand;
    std::string finger;
    std::string version;

    virtual void allocate(const std::string &_type);
    virtual void clone(const iKinLimb &limb);

public:
    /**
    * Default constructor. 
    */
    iCubFinger();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between the finger's 
    *              type which has to be passed in the form
    *              <i><hand>_<finger>_<version></i>, where
    *              <i>hand</i> accounts for "left"|"right",
    *              <i>finger</i> for
    *              "thumb"|"index"|"middle"|"ring"|"little", and
    *              <i>version</i> for "na"|"a"|"b".
    *  
    * @note "na" (i.e. not-applicable) is the default hardware 
    *       version for index, middle, ring and little finger,
    *       whereas "b" is the default for the thumb; "a" and "b"
    *       refer to different mechanical configurations, please see
    *       the manual.
    */
    iCubFinger(const std::string &_type);

    /**
    * Creates a new Finger from an already existing object.
    * @param sensor is the object to be copied.
    */
    iCubFinger(const iCubFinger &finger);

    /**
    * Alignes the finger joints bounds with current values set 
    * aboard the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Arm limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);

    /**
    * Retrieves the vector of actual finger's joint values (to be 
    * used in conjuction with the iKinLimb methods) from the vector 
    * of robot encoders. 
    * @param robotEncoders the vector of robot encoders from which 
    *                  to extract the joint values of the finger. It
    *                  can be composed of 16 or 9 elements,
    *                  depending if it comprises the arm encoders as
    *                  well or just the hand encoders.
    * @param chainJoints the vector containing the joints values to 
    *                    be used with the iKinLimb methods.
    * @return true/false on success/failure. 
    *  
    * @note This method accounts also for the underactuated joints, 
    *       meaning that the actual number of DOFs of the index, for
    *       example, is 4 and not 3: one for the abduction movement,
    *       one for the proximal movement and 2 coupled movements
    *       for the distal joint (equally partitioned).
    */
    virtual bool getChainJoints(const yarp::sig::Vector &robotEncoders,
                                yarp::sig::Vector &chainJoints);
};


/**
* \ingroup iKinFwd
*
* A class for defining the iCub Leg.
*/
class iCubLeg : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubLeg();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" leg. Further available options are
    *              "[left|right]_v[1|2.5]".
    */
    iCubLeg(const std::string &_type);

    /**
    * Creates a new Leg from an already existing Leg object.
    * @param leg is the Leg to be copied.
    */
    iCubLeg(const iCubLeg &leg);

    /**
    * Alignes the Leg joints bounds with current values set aboard 
    * the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Leg limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};


/**
* \ingroup iKinFwd
*
* A class for defining the iCub Eye.
*/
class iCubEye : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEye();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye. Further available options are
    *              "[left|right]_v[1|2]".
    */
    iCubEye(const std::string &_type);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEye(const iCubEye &eye);

    /**
    * Alignes the Eye joints bounds with current values set aboard 
    * the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Torso and the Head limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};


/**
* \ingroup iKinFwd
*
* A class for defining the iCub Eye with the root reference 
* frame attached to the neck. 
*/
class iCubEyeNeckRef : public iCubEye
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubEyeNeckRef();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye. Further available options are
    *              "left_v2" and "right_v2".
    */
    iCubEyeNeckRef(const std::string &_type);

    /**
    * Creates a new Eye from an already existing Eye object.
    * @param eye is the Eye to be copied.
    */
    iCubEyeNeckRef(const iCubEyeNeckRef &eye);
};


/**
* \ingroup iKinFwd
*
* A class for describing the kinematic of the straight line
* coming out from the point located between the eyes. 
*/
class iCubHeadCenter : public iCubEye
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubHeadCenter();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "left" and 
    *              "right" eye. Further available options are
    *              "[left|right]_v[1|2]".
    */
    iCubHeadCenter(const std::string &_type);

    /**
    * Creates a new iCubHeadCenter from an already existing 
    * iCubHeadCenter object. 
    * @param head is the iCubHeadCenter object to be copied.
    */
    iCubHeadCenter(const iCubHeadCenter &head);
};


/**
* \ingroup iKinFwd
*
* A class for defining the Inertia Sensor Kinematics of the
* iCub. 
*/
class iCubInertialSensor : public iKinLimb
{
protected:
    virtual void allocate(const std::string &_type);

public:
    /**
    * Default constructor. 
    */
    iCubInertialSensor();

    /**
    * Constructor. 
    * @param _type is a string to discriminate between "v1" and "v2" 
    *              hardware versions.
    */
    iCubInertialSensor(const std::string &_type);

    /**
    * Creates a new Inertial Sensor from an already existing object.
    * @param sensor is the object to be copied.
    */
    iCubInertialSensor(const iCubInertialSensor &sensor);

    /**
    * Alignes the Inertial Sensor joints bounds with current values 
    * set aboard the iCub. 
    * @param lim is the ordered list of control interfaces that 
    *            allows to access the Torso and the Head limits.
    * @return true/false on success/failure. 
    */
    virtual bool alignJointsBounds(const std::deque<yarp::dev::IControlLimits*> &lim);
};

}

}

#endif




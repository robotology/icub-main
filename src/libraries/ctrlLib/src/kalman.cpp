
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/kalman.h>

using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/**********************************************************************/
Kalman::Kalman(const Matrix &_A, const Matrix &_H, const Matrix &_Q,
               const Matrix &_R) : A(_A), H(_H), Q(_Q), R(_R)
{
    n=A.rows();
    m=H.rows();

    At=A.transposed();
    Ht=H.transposed();
    I=eye(n,n);

    x.resize(n,0.0);
    P.resize(n,n); P.zero();
    K.resize(n,m); K.zero();
}


/**********************************************************************/
void Kalman::init(const Vector &_z0, const Vector &_x0, const Matrix &_P0)
{ 
	x=_x0;
    P=_P0;
}


/**********************************************************************/
Vector Kalman::filt(const Vector &z)
{
    // prediction
    x=A*x;
    P=A*P*At+Q;

    // Kalman gain
    K=P*Ht*pinv(H*P*Ht+R);

    // correction
    x=x+K*(z-H*x);
    P=(I-K*H)*P;

    return x;
}




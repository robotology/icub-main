
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/adaptWinPolyEstimator.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/***************************************************************************/
AWPolyEstimator::AWPolyEstimator(unsigned int _order, unsigned int _N, const double _D) : 
                                 order(_order), N(_N), D(_D)
{
    order=order<1 ? 1 : order;
    coeff.resize(order+1);
    N=N<=order ? N+1 : N;
    t.resize(N);
    x.resize(N);

    firstRun=true;
}


/***************************************************************************/
double AWPolyEstimator::eval(double x)
{
    double y=coeff[0];

    for (unsigned int i=1; i<=order; i++)
    {
        y+=coeff[i]*x;
        x*=x;
    }

    return y;
}


/***************************************************************************/
Vector AWPolyEstimator::fit(const Vector &x, const Vector &y, const unsigned int n)
{
    size_t nx=x.length();
    size_t ny=y.length();
    size_t i2=nx>ny ? ny : nx;

    unsigned int i1=0;
    unsigned int M =i2;

    if (n)
    {
        i1=i2-n;
        M=n;
    }

    Matrix R(M,order+1);
    Vector _y(M);

    for (unsigned int i=i1; i<i2; i++)
    {
        double _x=x[i];

        R(i-i1,0)=1.0;

        for (unsigned int j=1; j<=order; j++)
        {
            R(i-i1,j)=_x;
            _x*=_x;
        }

        _y[i-i1]=y[i];
    }

    if (R.rows()>=R.cols())
        return pinv(R)*_y;
    else
        return pinv(R.transposed()).transposed()*_y;
}


/***************************************************************************/
Vector AWPolyEstimator::estimate()
{
    if (!elemList.size())
        return Vector(0);   // generate error

    size_t dim=elemList[0].data.length();
    Vector esteem(dim);

    if (firstRun)
    {    
        winLen.resize(dim);
        winLen=N;

        firstRun=false;
    }    

    size_t L=elemList.size();
    int delta=L-N;

    if (delta<0)
        return esteem=0.0;

    // retrieve the time vector
    // starting from t=0 (numeric stability reason)
    for (unsigned int j=0; j<N; j++)
        t[j]=elemList[delta+j].time-elemList[delta].time;

    // cycle upon all elements
    for (unsigned int i=0; i<dim; i++)
    {
        // retrieve the data vector
        for (unsigned int j=0; j<N; j++)
            x[j]=elemList[delta+j].data[i];

        // change the window length of two units, back and forth
        unsigned int n1=(unsigned int)(winLen[i]>order+1 ? winLen[i]-1 : order+1);
        unsigned int n2=(unsigned int)(winLen[i]<N ? winLen[i]+1 : N);

        // cycle upon all possibile window's length
        for (unsigned int n=n1; n<=n2; n++)
        {
            // find the regressor's coefficients
            coeff=fit(t,x,n);
            bool _stop=false;

            // test the regressor upon all the elements
            // belonging to the actual window
            for (unsigned int k=N-n; k<N; k++)
                if (fabs(x[k]-eval(t[k]))>D)
                {
                    // exit if the max deviation is not verified
                    _stop=true;
                    break;
                }

            // set the new window's length in case of
            // crossing of max deviation threshold
            if (_stop)
            {
                winLen[i]=n;
                break;
            }
        }

        esteem[i]=getEsteeme();
    }

    if (delta>10)
        elemList.pop_front();

    return esteem;
}


/***************************************************************************/
AWPolyEstimator::~AWPolyEstimator()
{
    elemList.clear();
}


/***************************************************************************/
Vector AWLinEstimator::fit(const Vector &x, const Vector &y, const unsigned int n)
{
    size_t nx=x.length();
    size_t ny=y.length();
    size_t i2=nx>ny ? ny : nx;

    unsigned int i1=0;
    unsigned int M =i2;

    if (n)
    {
        i1=i2-n;
        M=n;
    }

    double sum_xi  =0.0;
    double sum_xixi=0.0;
    double sum_yi  =0.0;
    double sum_xiyi=0.0;

    for (unsigned int i=i1; i<i2; i++)
    {
        sum_xi  +=x[i];
        sum_xixi+=x[i]*x[i];
        sum_yi  +=y[i];
        sum_xiyi+=x[i]*y[i];
    }

    double den=(M*sum_xixi-sum_xi*sum_xi);

    Vector c(2);

    // the bias
    c[0]=(sum_yi*sum_xixi-sum_xi*sum_xiyi) / den;

    // the linear coefficient
    c[1]=(M*sum_xiyi-sum_xi*sum_yi) / den;

    return c;
}




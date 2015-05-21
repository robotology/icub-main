#include "iCub/skinDynLib/utils.h"
using namespace std;

yarp::sig::Vector iCub::skinDynLib::toVector(yarp::sig::Matrix m)
{
    yarp::sig::Vector res(m.rows()*m.cols(),0.0);
    
    for (size_t r = 0; r < m.rows(); r++)
    {
        res.setSubvector(r*m.cols(),m.getRow(r));
    }

    return res;
}

void iCub::skinDynLib::closePort(yarp::os::Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

yarp::sig::Matrix iCub::skinDynLib::matrixFromBottle(const yarp::os::Bottle b, int in, const int r, const int c)
{
    yarp::sig::Matrix m(r,c);
    m.zero();
    
    for (size_t i = 0; i<r; i++)
    {
        for (size_t j = 0; j<c; j++)
        {
            m(i,j) =  b.get(in).asDouble();
            in++;
        }
    }
    
    return m;
}

yarp::sig::Vector iCub::skinDynLib::vectorFromBottle(const yarp::os::Bottle b, int in, const int size)
{
    yarp::sig::Vector v(size,0.0);

    for (size_t i = 0; i < size; i++)
    {
        v[i] = b.get(in).asDouble();
        in++;
    }
    return v;
}

void iCub::skinDynLib::vectorIntoBottle(const yarp::sig::Vector v, yarp::os::Bottle &b)
{
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

void iCub::skinDynLib::matrixIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b)
{
    yarp::sig::Vector v = toVector(m);
    
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

void iCub::skinDynLib::matrixOfIntIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b)
{
    yarp::sig::Vector v = toVector(m);
    
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addInt(int(v[i]));
    }
}

string iCub::skinDynLib::int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

unsigned int iCub::skinDynLib::factorial(unsigned int n) 
{
    if (n == 0)
       return 1;
    return n * factorial(n - 1);
}

// empty line to make gcc happy

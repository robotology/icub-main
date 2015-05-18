#include "iCub/skinDynLib/utils.h"

yarp::sig::Vector iCub::skinDynLib::toVector(yarp::sig::Matrix m)
{
    Vector res(m.rows()*m.cols(),0.0);
    
    for (size_t r = 0; r < m.rows(); r++)
    {
        res.setSubvector(r*m.cols(),m.getRow(r));
    }

    return res;
}

void iCub::skinDynLib::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

yarp::sig::Matrix iCub::skinDynLib::matrixFromBottle(const Bottle b, int in, const int r, const int c)
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

yarp::sig::Vector iCub::skinDynLib::vectorFromBottle(const Bottle b, int in, const int size)
{
    yarp::sig::Vector v(size,0.0);

    for (size_t i = 0; i < size; i++)
    {
        v[i] = b.get(in).asDouble();
        in++;
    }
    return v;
}

void iCub::skinDynLib::vectorIntoBottle(const yarp::sig::Vector v, Bottle &b)
{
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

void iCub::skinDynLib::matrixIntoBottle(const yarp::sig::Matrix m, Bottle &b)
{
    Vector v = toVector(m);
    
    for (unsigned int i = 0; i < v.size(); i++)
    {
        b.addDouble(v[i]);
    }
}

void iCub::skinDynLib::matrixOfIntIntoBottle(const yarp::sig::Matrix m, Bottle &b)
{
    Vector v = toVector(m);
    
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

/****************************************************************/
/* INCOMING EVENT WRAPPER
*****************************************************************/
    IncomingEvent::IncomingEvent()
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;
    }

    IncomingEvent::IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s)
    {
        Pos = p;
        Vel = v;
        Src = s;
        Radius = r;
    }

    IncomingEvent::IncomingEvent(const IncomingEvent &e)
    {
        *this = e;
    }

    IncomingEvent::IncomingEvent(const Bottle &b)
    {
        fromBottle(b);
    }

    IncomingEvent & IncomingEvent::operator=(const IncomingEvent &e)
    {
        Pos    = e.Pos;
        Vel    = e.Vel;
        Src    = e.Src;
        Radius = e.Radius;
        return *this;
    }

    Bottle IncomingEvent::toBottle()
    {
        Bottle b;
        b.clear();

        b.addDouble(Pos[0]);
        b.addDouble(Pos[1]);
        b.addDouble(Pos[2]);
        b.addDouble(Vel[0]);
        b.addDouble(Vel[1]);
        b.addDouble(Vel[2]);
        b.addDouble(Radius);
        b.addString(Src);

        return b;
    }

    bool IncomingEvent::fromBottle(const Bottle &b)
    {
        Pos.resize(3,0.0);
        Vel.resize(3,0.0);
        Src="";
        Radius=-1.0;

        Pos[0] = b.get(0).asDouble();
        Pos[1] = b.get(1).asDouble();
        Pos[2] = b.get(2).asDouble();

        Vel[0] = b.get(3).asDouble();
        Vel[1] = b.get(4).asDouble();
        Vel[2] = b.get(5).asDouble();

        Radius = b.get(6).asDouble();
        Src    = b.get(7).asString();

        return true;
    }

    void IncomingEvent::print()
    {
        yDebug("\tPos: %s\t Vel: %s\t Radius %g\t Src %s\n",Pos.toString().c_str(),Vel.toString().c_str(),Radius,Src.c_str());
    }

    string IncomingEvent::toString() const
    {
        stringstream res;
        res << "Pos: "<< Pos.toString(3,3) << "\t Vel: "<< Vel.toString(3,3)
            << "\t Radius: "<< Radius << "\t Src: "<< Src;
        return res.str();
    }

/****************************************************************/
/* INCOMING EVENT 4 TAXEL WRAPPER
*****************************************************************/
    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE() : IncomingEvent()
    {
        NRM = 0;
        TTC = 0;
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const Vector &p, const Vector &v,
                                                   const double r, const string &s) :
                                                   IncomingEvent(p,v,r,s)
    {
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e)
    {
        *this = e;
        computeNRMTTC();
    }

    IncomingEvent4TaxelPWE::IncomingEvent4TaxelPWE(const IncomingEvent &e)
    {
        *this = e;   
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent4TaxelPWE &e)
    {
        IncomingEvent::operator=(e);
        TTC    = e.TTC;
        NRM    = e.NRM;
        return *this;
    }

    IncomingEvent4TaxelPWE & IncomingEvent4TaxelPWE::operator=(const IncomingEvent &e)
    {
        IncomingEvent::operator=(e);
        computeNRMTTC();
        return *this;
    }

    void IncomingEvent4TaxelPWE::computeNRMTTC()
    {
        int sgn = Pos[2]>=0?1:-1;
        NRM = sgn * norm(Pos);

        // if (norm(Vel) < 0.38 && norm(Vel) > 0.34)
        // {
        //     TTC = 10000.0;
        // }
        // else
        // 
        if (dot(Pos,Vel)==0) TTC = 0;
        else                 TTC = -norm(Pos)*norm(Pos)/dot(Pos,Vel);
    }

    std::vector<double> IncomingEvent4TaxelPWE::getNRMTTC()
    {
        std::vector<double> x;
        x.push_back(NRM);
        x.push_back(TTC);

        return x;
    }

    void IncomingEvent4TaxelPWE::print()
    {
        yDebug("\tNRM: %g\t TTC: %g \t %s", NRM, TTC, IncomingEvent::toString().c_str());
    }

    string IncomingEvent4TaxelPWE::toString() const
    {
        stringstream res;
        res << "NRM: "<< NRM << "\t TTC: " << TTC << "\t "<< IncomingEvent::toString();
        return res.str();
    }

// empty line to make gcc happy

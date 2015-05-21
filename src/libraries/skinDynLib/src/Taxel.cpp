#include "iCub/skinDynLib/Taxel.h"

/****************************************************************/
/* TAXEL WRAPPER
*****************************************************************/
    Taxel::Taxel()
    {
        init();
    }

    Taxel::Taxel(const Vector &_position, const Vector &_normal)
    {
        init();
        Position = _position;
        Normal   = _normal;
        setFoR();
    }

    Taxel::Taxel(const Vector &_position, const Vector &_normal, const int &_id)
    {
        init();
        ID       = _id;
        Position = _position;
        Normal   = _normal;
        setFoR();
    }

    Taxel::Taxel(const Taxel &_t)
    {
        *this = _t;
    }

    Taxel & Taxel::operator=(const Taxel &_t)
    {
        if (this == &_t)
        {
            return *this;
        }

        ID          = _t.ID;
        Position    = _t.Position;
        WRFPosition = _t.WRFPosition;
        Normal      = _t.Normal;
        px          = _t.px;
        FoR         = _t.FoR;
        return *this;
    }

    void Taxel::init()
    {
        ID = 0;
        Position.resize(3,0.0);
        WRFPosition.resize(3,0.0);
        Normal.resize(3,0.0);
        px.resize(2,0.0);
        FoR = eye(4);
    }

    void Taxel::setFoR()
    {
        if (Normal == zeros(3))
        {
            FoR=eye(4);
            return;
        }
        
        // Set the proper orientation for the touching end-effector
        Vector x(3,0.0), z(3,0.0), y(3,0.0);

        z = Normal;
        if (z[0] == 0.0)
        {
            z[0] = 0.00000001;    // Avoid the division by 0
        }
        y[0] = -z[2]/z[0]; y[2] = 1;
        x = -1*(cross(z,y));
        
        // Let's make them unitary vectors:
        x = x / norm(x);
        y = y / norm(y);
        z = z / norm(z);

        FoR=eye(4);
        FoR.setSubcol(x,0,0);
        FoR.setSubcol(y,0,1);
        FoR.setSubcol(z,0,2);
        FoR.setSubcol(Position,0,3);
    }

    void Taxel::print(int verbosity)
    {
        if (verbosity)
        {
            yDebug("ID %i \tPosition %s \tNormal %s \tWRFPosition %s \tpx %s", ID,
                    Position.toString(3,3).c_str(), Normal.toString(3,3).c_str(),
                    WRFPosition.toString(3,3).c_str(),px.toString(3,3).c_str());
            yDebug("Frame of Reference \n%s",FoR.toString(3,3).c_str());
        }
        else 
            yDebug("ID %i \tPosition %s \tNormal %s\n", ID,
                    Position.toString(3,3).c_str(), Normal.toString(3,3).c_str());
    }

    std::string Taxel::toString(int verbosity)
    {
        std::stringstream res;
        res << "ID: " << ID << "\tPosition: "<< Position.toString(3,3) <<
            "\tNormal: "<< Normal.toString(3,3);

        if (verbosity)
        {
            res << "\tWRFPosition: " << WRFPosition.toString(3,3) <<
                   "\tPx: " << px.toString(3,3) <<
                   "Frame of Reference: \n" << FoR.toString(3,3) << std::endl;
        }
        return res.str();
    }

// empty line to make gcc happy

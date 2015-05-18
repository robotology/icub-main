#include "iCub/skinDynLib/Taxel.h"


/****************************************************************/
/* TAXEL WRAPPER
*****************************************************************/
    void Taxel::init()
    {
        ID      = 0;
        Pos.resize(3,0.0);
        WRFPos.resize(3,0.0);
        Norm.resize(3,0.0);
        px.resize(2,0.0);
        FoR = eye(4);
    }

    Taxel::Taxel()
    {
        init();
    }

    Taxel::Taxel(const Vector &p, const Vector &n)
    {
        init();
        Pos  = p;
        Norm = n;
        setFoR();
    }

    Taxel::Taxel(const Vector &p, const Vector &n, const int &i)
    {
        init();
        ID   = i;
        Pos  = p;
        Norm = n;
        setFoR();
    }

    Taxel & Taxel::operator=(const Taxel &t)
    {
        ID      = t.ID;
        Pos     = t.Pos;
        WRFPos  = t.WRFPos;
        Norm    = t.Norm;
        px      = t.px;
        FoR     = t.FoR;
        return *this;
    }

    void Taxel::setFoR()
    {
        if (Norm == zeros(3))
        {
            FoR=eye(4);
            return;
        }
        
        // Set the proper orientation for the touching end-effector
        Vector x(3,0.0), z(3,0.0), y(3,0.0);

        z = Norm;
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
        FoR.setSubcol(Pos,0,3);
    }

/****************************************************************/
/* TAXEL WRAPPER FOR PWE
*****************************************************************/

    TaxelPWE::TaxelPWE() : Taxel(), Evnt()
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    }

    TaxelPWE::TaxelPWE(const Vector &p, const Vector &n) : Taxel(p,n)
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    };

    TaxelPWE::TaxelPWE(const Vector &p, const Vector &n, const int &i) : Taxel(p,n,i)
    {
        Resp    = 0;
        RFangle = 40*M_PI/180;
    };

    bool TaxelPWE::addSample(IncomingEvent4TaxelPWE ie)
    {
        if (!insideFoRCheck(ie))
            return false;

        std::vector <double> x = ie.getNRMTTC();
        printf("[TaxelPWE::addSample] x %g %g\n",x[0],x[1]);
        
        return pwe->addSample(x);
    }

    bool TaxelPWE::removeSample(IncomingEvent4TaxelPWE ie)
    {
        if (!insideFoRCheck(ie))
            return false;

        std::vector <double> x = ie.getNRMTTC();
        return pwe->removeSample(x);
    }

    bool TaxelPWE::insideFoRCheck(const IncomingEvent4TaxelPWE ie)
    {
        std::vector<double> binWidth = pwe->getBinWidth();
        double binLimit = 8*binWidth[0];

        // the x,y limit of the receptive field at the incoming event's Z
        double RFlimit = ie.Pos(2)/tan(RFangle);

        // the x,y limit of the receptive field in the first bin
        double RFlimit_cyl = binLimit/tan(RFangle);

        // yDebug("binLimit: %g RFlimit_cyl: %g RFangle: %g \n", binLimit, RFlimit_cyl, RFangle);
        // yDebug("ie.Pos\t%s\n", ie.Pos.toString(3,3).c_str());
        // yDebug("Hist:\n%s\n", pwe->getHist().toString(3,3).c_str());

        if (ie.Pos(0)*ie.Pos(0)+ie.Pos(1)*ie.Pos(1) < RFlimit*RFlimit )
        {
            return true;
        }
        // There are two ifs only to let me debug things
        if ( (abs(ie.Pos(2))<=binLimit) && (ie.Pos(0)*ie.Pos(0)+ie.Pos(1)*ie.Pos(1) < RFlimit_cyl*RFlimit_cyl) )
        {
            return true;
        }
        return false;
    }

    void TaxelPWE::print(int verbosity)
    {
        if (verbosity > 4)
            yDebug("ID %i \tPos %s \tNorm %s \n\tPosHst \n%s\n\n\tNegHst \n%s\n", ID,
                    Pos.toString(3,3).c_str(), Norm.toString(3,3).c_str(),
                    pwe->getPosHist().toString(3,3).c_str(),
                    pwe->getNegHist().toString(3,3).c_str());
        else 
            yDebug("ID %i \tPos %s \tNorm %s\n", ID,
                    Pos.toString(3,3).c_str(), Norm.toString(3,3).c_str());
            // yDebug("ID %i \tPos %s \tNorm %s \n\tHst %s\n", ID,
            //         Pos.toString(3,3).c_str(), Norm.toString(3,3).c_str(),
            //         pwe->getHist().toString(3,3).c_str());
    }

    string TaxelPWE::toString(int precision)
    {
        stringstream res;
        res << "ID: " << ID << "\tPos: "<< Pos.toString(3,3) << "\t Norm: "<< Norm.toString(3,3);

        if (precision)
        {
            res << "\n PosHst:\n"<< pwe->getPosHist().toString(3,3);
            res << "\n NegHst:\n"<< pwe->getNegHist().toString(3,3) << endl;
        }
        return res.str();
    }

    bool TaxelPWE::resetParzenWindowEstimator()
    {
        pwe->resetAllHist();
        return true;
    }

    bool TaxelPWE::computeResponse()
    {
        if (!insideFoRCheck(Evnt))
        {
            Resp = 0;
            return false;
        }

        std::vector<double> In = Evnt.getNRMTTC();
        Resp = pwe->computeResponse(In);

        return true;
    }

    Bottle TaxelPWE::TaxelPWEIntoBottle()
    {
        Bottle res;
        res.clear();
        res.addInt(ID);

        Bottle &dataPH = res.addList();
        matrixOfIntIntoBottle(pwe->getPosHist(),dataPH);

        Bottle &dataNH = res.addList();
        matrixOfIntIntoBottle(pwe->getNegHist(),dataNH);

        return res;
    }

    TaxelPWE::~TaxelPWE()
    {
        if (pwe!=NULL)
        {
            delete pwe;
        }
    }

// empty line to make gcc happy

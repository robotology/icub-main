
#include <iCub/utils.hpp>


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    size_t bLen=b.size();
    size_t xdLen=xd.length();
    size_t n=bLen>xdLen ? xdLen : bLen;

    for (unsigned int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();

    isNew=true;
}


/************************************************************************/
void neckCallback::exec(Vector xd, Vector q)
{
    // update neck pitch,yaw
    commData->get_xd()=xd;
    commData->get_qd()[0]=q[0];
    commData->get_qd()[2]=q[1];
}


/************************************************************************/
void eyesCallback::exec(Vector xd, Vector q)
{
    // update eyes tilt,pan,vergence
    commData->get_xd()=xd;
    commData->get_qd()[3]=q[0];
    commData->get_qd()[4]=q[1];
    commData->get_qd()[5]=q[2];
}


/************************************************************************/
bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("cx") && parType.check("cy") &&
                parType.check("fx") && parType.check("fy"))
            {
                double cx=parType.find("cx").asDouble();
                double cy=parType.find("cy").asDouble();
                double fx=parType.find("fx").asDouble();
                double fy=parType.find("fy").asDouble();

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;

                return true;
            }
            else
                cerr << warning << endl;
        }
        else
            cerr << warning << endl;
    }

    return false;
}


/************************************************************************/
bool getAlignLinks(const string &configFile, const string &type,
                   iKinLink **link1, iKinLink **link2)
{
    *link1=*link2=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Aligning parameters for "+type+" group not found";

        if (parType.size())
        {
            Bottle length=parType.findGroup("length");
            Bottle offset=parType.findGroup("offset");
            Bottle twist=parType.findGroup("twist");

            if (length.size()>=2 && offset.size()>=2 && twist.size()>=2)
            {
                *link1=new iKinLink(length.get(1).asDouble(),offset.get(1).asDouble(),
                                    twist.get(1).asDouble(),0.0,0.0,0.0);
                *link2=new iKinLink(length.get(2).asDouble(),offset.get(2).asDouble(),
                                    twist.get(2).asDouble(),0.0,0.0,0.0);

                return true;
            }
            else
                cerr << warning << endl;
        }
        else
            cerr << warning << endl;
    }

    return false;
}


/************************************************************************/
Matrix alignJointsBounds(iKinChain *chain, IControlLimits *limTorso, IControlLimits *limHead,
                         const double eyeTiltMin, const double eyeTiltMax)
{
    double min, max;

    for (int i=0; i<3; i++)
    {   
        limTorso->getLimits(i,&min,&max);

        (*chain)[2-i].setMin(CTRL_DEG2RAD*min);
        (*chain)[2-i].setMax(CTRL_DEG2RAD*max);
    }

    Matrix lim(6,2);

    for (int i=0; i<6; i++)
    {   
        limHead->getLimits(i,&min,&max);

        // limit eye's tilt due to eyelids
        if (i==3)
        {
            if (min<eyeTiltMin)
                min=eyeTiltMin;

            if (max>eyeTiltMax)
                max=eyeTiltMax;
        }

        lim(i,0)=CTRL_DEG2RAD*min;
        lim(i,1)=CTRL_DEG2RAD*max;

        // just one eye's got only 5 dofs
        if (i<5)
        {
            (*chain)[3+i].setMin(lim(i,0));
            (*chain)[3+i].setMax(lim(i,1));
        }
    }

    return lim;
}


/************************************************************************/
void copyJointsBounds(iKinChain *ch1, iKinChain *ch2)
{
    unsigned int N1=ch1->getN();
    unsigned int N2=ch2->getN();
    unsigned int N =N1>N2 ? N2 : N1;

    for (unsigned int i=0; i<N; i++)
    {
        (*ch2)[i].setMin((*ch1)[i].getMin());
        (*ch2)[i].setMax((*ch1)[i].getMax());
    }
}


/************************************************************************/
void updateTorsoBlockedJoints(iKinChain *chain, Vector &fbTorso)
{
    for (int i=0; i<3; i++)
         chain->setBlockingValue(i,fbTorso[i]);
}


/************************************************************************/
void updateNeckBlockedJoints(iKinChain *chain, Vector &fbNeck)
{
    for (int i=0; i<3; i++)
         chain->setBlockingValue(3+i,fbNeck[i]);
}


/************************************************************************/
bool getFeedback(Vector &fbTorso, Vector &fbHead, IEncoders *encTorso, IEncoders *encHead)
{
    Vector fb(6);
    bool ret=true;
    
    if (encTorso->getEncoders(fb.data()))
        for (int i=0; i<3; i++)
            fbTorso[i]=CTRL_DEG2RAD*fb[2-i];    // reversed order
    else
        ret=false;

    if (encHead->getEncoders(fb.data()))
        fbHead=CTRL_DEG2RAD*fb;
    else
        ret=false;

    return ret;
}



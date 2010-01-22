
#include "utils.h"


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    size_t bLen=b.size();
    size_t xdLen=xd.length();
    size_t n=bLen>xdLen ? xdLen : bLen;

    for (unsigned int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();
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
bool getAlignLinks(const string &configFile, const string &type,
                   iKinLink **link1, iKinLink **link2)
{
    *link1=*link2=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string error="unable to find aligning parameters for "+type+" eye";

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
                cerr << error << endl;
        }
        else
            cerr << error << endl;
    }

    return false;
}


/************************************************************************/
Matrix alignJointsBounds(iKinChain *chain, IControlLimits *limTorso, IControlLimits *limHead)
{
    double min, max;

    for (int i=0; i<3; i++)
    {   
        limTorso->getLimits(i,&min,&max);

        (*chain)[2-i].setMin((M_PI/180.0)*min);
        (*chain)[2-i].setMax((M_PI/180.0)*max);
    }

    Matrix lim(6,2);

    for (int i=0; i<6; i++)
    {   
        limHead->getLimits(i,&min,&max);

        // limit eye's tilt due to eyelids
        if (i==3)
        {
            if (min<-28.0)
                min=-28.0;

            if (max>12.0)
                max=12.0;
        }

        lim(i,0)=(M_PI/180.0)*min;
        lim(i,1)=(M_PI/180.0)*max;

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
            fbTorso[i]=(M_PI/180.0)*fb[2-i];    // reversed order
    else
        ret=false;

    if (encHead->getEncoders(fb.data()))
        fbHead=(M_PI/180.0)*fb;
    else
        ret=false;

    return ret;
}



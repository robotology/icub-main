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

#include <iCub/utils.h>
#include <iCub/solver.h>


/************************************************************************/
xdPort::xdPort(const Vector &xd0, void *_slv)
{   
    xdDelayed=xd=xd0;
    isNew=false;
    closing=false;

    slv=_slv;

    if (slv!=NULL)
        start();
}


/************************************************************************/
xdPort::~xdPort()
{
    closing=true;
    syncEvent.signal();

    if (slv!=NULL)
        stop();
}


/************************************************************************/
void xdPort::onRead(Bottle &b)
{
    size_t bLen=b.size();
    size_t xdLen=xd.length();
    size_t n=bLen>xdLen ? xdLen : bLen;

    for (unsigned int i=0; i<n; i++)
        xd[i]=b.get(i).asDouble();

    isNew=true;

    syncEvent.signal();
}


/************************************************************************/
void xdPort::set_xd(const Vector &_xd)
{
    xd=_xd;
    isNew=true;

    syncEvent.signal();
}


/************************************************************************/
void xdPort::run()
{
    while (!isStopping() && !closing)
    {
        syncEvent.reset();
        syncEvent.wait();

        Vector theta=static_cast<Solver*>(slv)->neckTargetRotAngles(xd);
        double timeDelay=0.0;

        if ((theta[0]<NECKSOLVER_RESTORINGANGLE_TRA*CTRL_DEG2RAD) &&
            (theta[1]<NECKSOLVER_RESTORINGANGLE_SAG*CTRL_DEG2RAD))
            timeDelay=NECKSOLVER_ACTIVATIONDELAY;

        Time::delay(timeDelay);

        xdDelayed=xd;
    }
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
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                double cx=parType.find("w").asDouble()/2.0;
                double cy=parType.find("h").asDouble()/2.0;
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
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
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

            fprintf(stdout,"%s: %s\n",type.c_str(),length.toString().c_str());
            fprintf(stdout,"%s: %s\n",type.c_str(),offset.toString().c_str());
            fprintf(stdout,"%s: %s\n",type.c_str(),twist.toString().c_str());

            if ((length.size()>=3) && (offset.size()>=3) && (twist.size()>=3))
            {
                *link1=new iKinLink(length.get(1).asDouble(),offset.get(1).asDouble(),
                                    twist.get(1).asDouble(),0.0,0.0,0.0);
                *link2=new iKinLink(length.get(2).asDouble(),offset.get(2).asDouble(),
                                    twist.get(2).asDouble(),0.0,0.0,0.0);

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}


/************************************************************************/
Matrix alignJointsBounds(iKinChain *chain, PolyDriver *drvTorso, PolyDriver *drvHead,
                         const double eyeTiltMin, const double eyeTiltMax)
{
    IEncoders      *encs;
    IControlLimits *lims;

    double min, max;
    int nJointsTorso=3;    

    if (drvTorso!=NULL)
    {
        drvTorso->view(encs);
        drvTorso->view(lims);        
        encs->getAxes(&nJointsTorso);

        for (int i=0; i<nJointsTorso; i++)
        {   
            lims->getLimits(i,&min,&max);
        
            (*chain)[nJointsTorso-1-i].setMin(CTRL_DEG2RAD*min); // reversed order
            (*chain)[nJointsTorso-1-i].setMax(CTRL_DEG2RAD*max);
        }
    }

    drvHead->view(encs);
    drvHead->view(lims);
    int nJointsHead;
    encs->getAxes(&nJointsHead);
    Matrix lim(nJointsHead,2);

    for (int i=0; i<nJointsHead; i++)
    {   
        lims->getLimits(i,&min,&max);

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
        if (i<nJointsHead-1)
        {
            (*chain)[nJointsTorso+i].setMin(lim(i,0));
            (*chain)[nJointsTorso+i].setMax(lim(i,1));
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
void updateTorsoBlockedJoints(iKinChain *chain, const Vector &fbTorso)
{
    for (int i=0; i<fbTorso.length(); i++)
         chain->setBlockingValue(i,fbTorso[i]);
}


/************************************************************************/
void updateNeckBlockedJoints(iKinChain *chain, const Vector &fbNeck)
{
    for (int i=0; i<3; i++)
         chain->setBlockingValue(3+i,fbNeck[i]);
}


/************************************************************************/
bool getFeedback(Vector &fbTorso, Vector &fbHead, IEncoders *encTorso, IEncoders *encHead)
{
    int nJointsTorso=fbTorso.length();
    int nJointsHead=fbHead.length();

    Vector fb(nJointsTorso<nJointsHead?nJointsHead:nJointsTorso);
    bool ret=true;

    if (encTorso!=NULL)
    {
        if (encTorso->getEncoders(fb.data()))
            for (int i=0; i<nJointsTorso; i++)
                fbTorso[i]=CTRL_DEG2RAD*fb[nJointsTorso-1-i];    // reversed order
        else
            ret=false;
    }
    else
        fbTorso=0.0;

    if (encHead->getEncoders(fb.data()))
        for (int i=0; i<nJointsHead; i++)
            fbHead[i]=CTRL_DEG2RAD*fb[i];
    else
        ret=false;

    // impose vergence != 0.0
    if (fbHead[nJointsHead-1]<MINALLOWED_VERGENCE*CTRL_DEG2RAD)
        fbHead[nJointsHead-1]=MINALLOWED_VERGENCE*CTRL_DEG2RAD;

    return ret;
}



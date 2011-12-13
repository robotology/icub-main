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

#include <iCub/iKin/iKinVocabs.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinHlp.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::iKin;


/************************************************************************/
void CartesianHelper::addVectorOption(Bottle &b, const int vcb, const Vector &v)
{
    Bottle &part=b.addList();
    part.addVocab(vcb);
    Bottle &vect=part.addList();

    for (size_t i=0; i<v.length(); i++)
        vect.addDouble(v[i]);
}


/************************************************************************/
bool CartesianHelper::getDesiredOption(Bottle &reply, Vector &xdhat,
                                       Vector &odhat, Vector &qdhat)
{
    if (reply.size()==0)
        return false;

    if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
    {
        // xdhat and odhat part
        if (reply.check(Vocab::decode(IKINSLV_VOCAB_OPT_X)))
        {
            Bottle *xData=getEndEffectorPoseOption(reply);
            if (xData->size()==0)
                return false;

            xdhat.resize(3);
            for (size_t i=0; i<xdhat.length(); i++)
                xdhat[i]=xData->get(i).asDouble();

            odhat.resize(4);
            for (size_t i=0; i<odhat.length(); i++)
                odhat[i]=xData->get(xdhat.length()+i).asDouble();
        }
        else
            return false;

        // qdhat part
        if (reply.check(Vocab::decode(IKINSLV_VOCAB_OPT_Q)))
        {
            Bottle *qData=getJointsOption(reply);
            if (qData->size()==0)
                return false;

            qdhat.resize(qData->size());
            for (size_t i=0; i<qdhat.length(); i++)
                qdhat[i]=qData->get(i).asDouble();
        }
        else
            return false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
void CartesianHelper::addTargetOption(Bottle &b, const Vector &xd)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_XD,xd);
}


/************************************************************************/
void CartesianHelper::addDOFOption(Bottle &b, const Vector &dof)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_DOF,dof);
}


/************************************************************************/
void CartesianHelper::addJointsResPosOption(Bottle &b, const Vector &restPos)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_REST_POS,restPos);
}


/************************************************************************/
void CartesianHelper::addJointsRestWeightsOption(Bottle &b, const Vector &restWeights)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_REST_WEIGHTS,restWeights);
}


/************************************************************************/
void CartesianHelper::addPoseOption(Bottle &b, const unsigned int pose)
{
    Bottle &posePart=b.addList();
    posePart.addVocab(IKINSLV_VOCAB_OPT_POSE);

    if (pose==IKINCTRL_POSE_FULL)
        posePart.addVocab(IKINSLV_VOCAB_VAL_POSE_FULL);
    else if (pose==IKINCTRL_POSE_XYZ)
        posePart.addVocab(IKINSLV_VOCAB_VAL_POSE_XYZ);
}


/************************************************************************/
void CartesianHelper::addModeOption(Bottle &b, const bool tracking)
{
    Bottle &modePart=b.addList();
    modePart.addVocab(IKINSLV_VOCAB_OPT_MODE);

    if (tracking)
        modePart.addVocab(IKINSLV_VOCAB_VAL_MODE_TRACK);
    else
        modePart.addVocab(IKINSLV_VOCAB_VAL_MODE_SINGLE);
}


/************************************************************************/
void CartesianHelper::addTokenOption(Bottle &b, const double token)
{
    Bottle &tokenPart=b.addList();
    tokenPart.addVocab(IKINSLV_VOCAB_OPT_TOKEN);
    tokenPart.addDouble(token);
}


/************************************************************************/
Bottle *CartesianHelper::getTargetOption(Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_XD)).asList();
}


/************************************************************************/
Bottle *CartesianHelper::getEndEffectorPoseOption(Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_X)).asList();
}


/************************************************************************/
Bottle *CartesianHelper::getJointsOption(Bottle &b)
{
    return b.find(Vocab::decode(IKINSLV_VOCAB_OPT_Q)).asList();
}


/************************************************************************/
bool CartesianHelper::getTokenOption(Bottle &b, double *token)
{
    if (b.check(Vocab::decode(IKINSLV_VOCAB_OPT_TOKEN)) && (token!=NULL))
    {
        *token=b.find(Vocab::decode(IKINSLV_VOCAB_OPT_TOKEN)).asDouble();
        return true;
    }
    else
        return false;
}



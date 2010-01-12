
#include <iCub/iKinVocabs.h>
#include <iCub/iKinInv.h>
#include <iCub/iKinHlp.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iKin;


/************************************************************************/
void CartesianHelper::addVectorOption(Bottle &b, const int vcb, const Vector &v)
{
    Bottle &part=b.addList();
    part.addVocab(vcb);
    Bottle &vect=part.addList();

    for (int i=0; i<v.length(); i++)
        vect.addDouble(v[i]);
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
void CartesianHelper::addTorsoRestOption(Bottle &b, const Vector &torsoRest)
{
    addVectorOption(b,IKINSLV_VOCAB_OPT_TORSO_REST,torsoRest);
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



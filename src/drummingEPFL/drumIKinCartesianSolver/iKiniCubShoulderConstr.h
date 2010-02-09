#define SHOULDER_MAXABDUCTION   (100.0*(M_PI/180.0))

namespace iKin
{
	class iCubShoulderConstr;
}

class iKin::iCubShoulderConstr : public iKinLinIneqConstr
{
protected:    
    double joint1_0, joint1_1;
    double joint2_0, joint2_1;
    double m, n;

    unsigned int numAxes2Shou;

    iKinChain *chain;

public:

    iCubShoulderConstr(iKinChain *_chain) : iKinLinIneqConstr()
    {      
        chain=_chain;

        // number of axes to reach shoulder's ones
        // from root reference  
        numAxes2Shou=3;

        joint1_0= 10.0*(M_PI/180.0);
        joint1_1= 15.0*(M_PI/180.0);
        joint2_0=-33.0*(M_PI/180.0);
        joint2_1= 60.0*(M_PI/180.0);
        m=(joint1_1-joint1_0)/(joint2_1-joint2_0);
        n=joint1_0-m*joint2_0;

        update(NULL);
    }

    virtual void update(void*)
    {
        // if any of shoulder's axes is blocked, skip
        if ((*chain)[numAxes2Shou].isBlocked()   ||
            (*chain)[numAxes2Shou+1].isBlocked() ||
            (*chain)[numAxes2Shou+2].isBlocked())
            setActive(false);   // optimization won't use LinIneqConstr
        else
        {
            // compute offset to shoulder's axes
            // given the blocked/release status of
            // previous link
            unsigned int offs=0;
            for (unsigned int i=0; i<numAxes2Shou; i++)
                if (!(*chain)[i].isBlocked())
                    offs++;

            // linear inequalities matrix
            C.resize(5,chain->getDOF()); C.zero();
            // constraints on the cables length
            C(0,offs)=1.71; C(0,offs+1)=-1.71;
            C(1,offs)=1.71; C(1,offs+1)=-1.71; C(1,offs+2)=-1.71;
                            C(2,offs+1)=1.0;   C(2,offs+2)=1.0;
            // constraints to prevent arm from touching torso
                            C(3,offs+1)=1.0;   C(3,offs+2)=-m;
            // constraints to limit shoulder abduction
                            C(4,offs+1)=1.0;
    
            // lower and upper bounds
            lB.resize(5); uB.resize(5);
            lB[0]=-347.00*(M_PI/180.0); uB[0]=upperBoundInf;
            lB[1]=-366.57*(M_PI/180.0); uB[1]=112.42*(M_PI/180.0);
            lB[2]=-66.600*(M_PI/180.0); uB[2]=213.30*(M_PI/180.0);
            lB[3]=n;                    uB[3]=upperBoundInf;
            lB[4]=lowerBoundInf;        uB[4]=SHOULDER_MAXABDUCTION;

            // optimization will use LinIneqConstr
            setActive(true);
        }
    }
};
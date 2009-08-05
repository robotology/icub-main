/**
iCubArmKinematics - kinematics of the iCub head

 @author Manuel Lopes  macl@isr.ist.utl.pt
*/
// iCubArmKinematics.h: interface for the iCubArmKinematics class.
//
//////////////////////////////////////////////////////////////////////

#include <iCub/kinematics/robmatrix.h>
#include <iCub/kinematics/robot.h>


#if !defined(_iCubArmKinematics_H_)
#define _iCubArmKinematics_H_

namespace iCub{
	namespace contrib{
		class iCubArmKinematics;
	}
}

using namespace iCub::contrib;

class iCub::contrib::iCubArmKinematics  
{
public:

    /**
    forward kinematics for the left/right arm

    @param neckposition - head joints position (degrees)
	@param arm			- l - left; r - right
    @return             - RobMatrix with frame of the arm written in the neck base frame
    */
    RobMatrix fkine(const double *neckposition, char arm);
	RobMatrix fkine(const double *neckposition, int joint);
	
	void jacob0( double *pos, gsl_matrix *J);

	iCubArmKinematics();
	virtual ~iCubArmKinematics();


	/**
		structure Robot that includes information about the kinematics, inspired on the matlab robotics toolbox
		 Rl - corresponds to the left arm
		 Rr - corresponds to the right arm
	*/
	Robot* Rl;
	Robot* Rr;

	
	protected:	

    /**
    forward kinematics for the left arm

    @param neckposition - head joints position (degrees)
    @return             - RobMatrix with frame of the arm written in the neck base frame
    */
    RobMatrix   fkine(const double *neckposition, Robot *R);

};

#endif // !defined(_iCubArmKinematics_H_)

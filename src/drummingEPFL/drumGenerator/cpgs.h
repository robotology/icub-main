
#ifndef cpg_manager_h
#define cpg_manager_h

#ifndef M_PI
#define M_PI        3.14159265358979323846   /* pi */
#endif

#ifndef SIGN
#define SIGN(a)((a)>0? 1 : -1)
#endif

#include <math.h>
#include <vector>

using namespace std;

#define matrix vector<vector<int>>

class cpg_manager
{

public:

	cpg_manager(int nbDOFs);
	~cpg_manager();

	//parameters of the equations
	double nu;	//frequency of the oscillations 
	double next_nu;
	
	double *ampl; //output amplitude of the oscillations	

	double *parameters; //2 parameters per dof: mu and g 
	int drumID;
	
	double **epsilon; //internal coupling strength
	double **theta; //coupling phase in radians
	double **theta_init;
	double next_theta;
	
	double *dydt; //derivatives
	double *r; //radius
	double *r2;
	double r_clock;
	double r_clock2;
	double nuStance;

	//open parameters
	double *g;//targets 
	double *m; //amplitudes
	
	//feedback
	double y_stuck;
	int drumHit;
	int stuckCounter;
	double *stuckPos;
	double *y_nofeed;
	int up_down;

	void integrate_step(double *y, double *at_states);		
	void printInternalVariables();

	double get_dt()
	{
	   return dt;
	}

	
private:

	int nbDOFs;
	int cpgs_size;
	int controlled_param;

	//equations parameters 
	double a; //rate of convergence of the rhythmic system
	double b;//rate of convergence of the discrete system
	double m_off; //value to turn of the oscillations 
	double m_on; // value to turn on the oscillations
	double b_go;//rate of convergence of the go command
	double u_go;// max value of the go command
	double dt; //integration step in seconds
	double alpha_x;//gain of the feedback for the position
	double alpha_y;//gain of the feedback for the speed
	double c;
};

#endif


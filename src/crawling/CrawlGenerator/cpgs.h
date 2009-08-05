
#ifndef cpgs_h
#define cpgs_h

#ifndef M_PI
#define M_PI        3.14159265358979323846   /* pi */
#endif

#include <math.h>
#include <vector>

using namespace std;

#define matrix vector<vector<int>>

class cpgs
{

public:

	cpgs(int nbDOFs);
	~cpgs();

	//parameters of the equations
	double om_stance, om_swing;//frequency of the oscillations 
	//double next_om_stance, next_om_swing;
	
	double *ampl; //output amplitude of the oscillations	

	double *parameters; //2 parameters per dof: mu and g 
	//double *next_parameters;
	
	double **epsilon; //internal coupling strength
	double **theta; //coupling phase in radians
	double **theta_init;
	
	double external_coupling[3];
	double next_external_coupling[3];

	double *dydt; //derivatives
	double *r; //radius

	//open parameters
	double *g;//targets 
	double *m; //amplitudes
	

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
	double c; //param for sw/st switching
};

#endif


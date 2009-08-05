#include "iCub/predMatrix.h"

#include <yarp/String.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Module.h>


#define NMODELS 7
#define NCORR 100

class predictors{
private:
	predMatrix<double>	A[NMODELS], 
						B[NMODELS], 
						C[NMODELS], 
						Q[NMODELS], 
						P[NMODELS], 
						Pe[NMODELS], 
						R[NMODELS], 
						k[NMODELS], 
						x[NMODELS],
						xe[NMODELS], 
						xaux[NMODELS],
						covR[NMODELS], 
						corrm[3];

	predMatrix<double>	predperiod[3],
						t1[3],
						t2[3];

	double				realPos[3], 
						u, 
						p[NMODELS],			//prior probability for each model
						predPos[3];

	double				_Ts; //the sampling time

	int _actM[NMODELS+1];
	int aT, Max;
	int VERBOSE;
	double cf; //collision lose factor
	int updt; //a colision has been detected
	double pT; //predicted collision time
	int up;
	int nact; //number of active models
	int corr; //correlation model flag
	int itac;
	int spM, bllM;
	bool detected,tr[3];

public:
	predictors(){
		itac=0;
		detected=false;
		up=0;
		predMatrix<double> a(15,1);
		predMatrix<double> b(NCORR,1);
		for(int i=0;i<3;i++){
			t1[i]=a;
			t2[i]=a;
			predperiod[i]=b;
		}
	};
	~predictors(){};

    bool open(yarp::os::Searchable& config);
	void init(int actM[NMODELS], int _nact, double v_eta11, double v_eta21,double v_eta31,double v_eta12,double v_eta22,
			double v_eta32,double v_eta13,double v_eta23,double v_eta33,double v_eta14,double v_eta24,double v_eta34,
			double v_eta15,double v_eta25,double v_eta35,double v_eta16,double v_eta26,double v_eta36,double v_eta17,
			double v_eta27,double v_eta37,double v_w1,double v_w2,double v_w3,double v_w4,double v_w5,double v_w6,double v_w7, 
			double T);
	void mmae();
	void update_state(int j, predMatrix<double> &xe);
	void restore_state(int j);
	bool chisq(predMatrix<double> rPos, predMatrix<double> out);
	bool reset(double _realPos[3]);
	double* predict(double _realPos[3], int nSteps, int _aT, FILE *f1=NULL);
	void Lpredict(double _realPos[3], int nSteps, FILE *f1=NULL);
	predMatrix<double> corrPred(predMatrix<double> x);
};

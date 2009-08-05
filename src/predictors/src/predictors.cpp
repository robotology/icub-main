#include "iCub/predictors.h"

#include <yarp/os/Searchable.h>

using namespace std;

#define NMODELS 7
#define PMODELS 8
#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#define y0 0.73 //table plan used in the adapted ballistic model
#define SVERBOSE 0
#define thM 0.4 //correlation threshold: corr(m)>thM => period detected
#define thm 0.1 //the correlation must be smaller than thm to start detecting the maximum, avoiding the first peak
#define NCORR 100 //size of window used in correlation model

using namespace std;

bool predictors::open(yarp::os::Searchable& config) 
{
    int nact;
    bool use_ca, use_wiener, use_singer, 
         use_circular, use_curvilin, 
         use_cv, use_ballistic, use_correlation;

	double v_eta11, v_eta21, v_eta31, v_eta12, v_eta22, v_eta32, v_eta13, v_eta23, v_eta33, 
		v_eta14, v_eta24, v_eta34, v_eta15, v_eta25, v_eta35, v_eta16, v_eta26, v_eta36, 
		v_eta17, v_eta27, v_eta37, v_w1, v_w2, v_w3, v_w4, v_w5, v_w6, v_w7;

	double sampleperiod;

    //which motion models to use
    use_ca = config.check("constaccel", 1, "Use constant aceleration model ? (0/1)").asInt();
    use_wiener = (bool)config.check("wiener", 1, "Use Wiener model ? (0/1)").asInt();
    use_singer = (bool)config.check("singer", 1, "Use Singer model ? (0/1)").asInt();
    use_circular = (bool)config.check("circular", 1, "Use circular model ? (0/1)").asInt();
    use_curvilin = (bool)config.check("circular", 1, "Use curvilinear model ? (0/1)").asInt();
    use_cv = (bool)config.check("constvel", 1, "Use constant velocity model ? (0/1)").asInt();
    use_ballistic = (bool)config.check("ballistic", 1, "Use ballistic model ? (0/1)").asInt();
    use_correlation = (bool)config.check("correl", 1, "Use correlation (periodic) model ? (0/1)").asInt();
    _actM[0] = (int)use_ca;
    _actM[1] = (int)use_wiener;
    _actM[2] = (int)use_singer;
    _actM[3] = (int)use_circular;
    _actM[4] = (int)use_curvilin;
    _actM[5] = (int)use_cv;
    _actM[6] = (int)use_ballistic;
    _actM[7] = (int)use_correlation;
    nact = _actM[0]+_actM[1]+_actM[2]+_actM[3]+_actM[4]+_actM[5]+_actM[6]+_actM[7];

	sampleperiod = config.check("sampleperiod", 1, "Measurements sampling period (double)").asDouble();
    
    //covariances
    v_eta11=config.check("v_eta11", 0.07, "x covariance for constant aceleration").asDouble();
	v_eta21=config.check("v_eta21", 0.07, "y covariance for constant aceleration").asDouble();
	v_eta31=config.check("v_eta31", 0.1, "z covariance for constant aceleration").asDouble();
	v_eta12=config.check("v_eta12", /*0.03*/0.07, "x covariance for wiener model").asDouble();
	v_eta22=config.check("v_eta22", /*0.03*/0.07, "y covariance for wiener model").asDouble();
	v_eta32=config.check("v_eta32", 0.12, "z covariance for wiener model").asDouble();
	v_eta13=config.check("v_eta13", /*0.05*/6, "x covariance for singer model").asDouble();
	v_eta23=config.check("v_eta23", 6, "y covariance for singer model").asDouble();
	v_eta33=config.check("v_eta33", 6, "z covariance for singer model").asDouble();
	v_eta14=config.check("v_eta14", 0.04, "x covariance for circular model").asDouble();
	v_eta24=config.check("v_eta24", 0.04, "y covariance for circular model").asDouble();
	v_eta34=config.check("v_eta34", 0.22, "z covariance for circular model").asDouble();
	v_eta15=config.check("v_eta15", 0.04, "x covariance for curvilinear model").asDouble();
	v_eta25=config.check("v_eta25", 0.04, "y covariance for curvilinear model").asDouble();
	v_eta35=config.check("v_eta35", 0.22, "z covariance for curvilinear model").asDouble();
	v_eta16=config.check("v_eta16", 0.1, "x covariance for constant velocity model").asDouble();
	v_eta26=config.check("v_eta26", 0.1, "y covariance for constant velocity model").asDouble();
	v_eta36=config.check("v_eta36", 0.1, "z covariance for constant velocity model").asDouble();
	v_eta17=config.check("v_eta17", /*0.005*/0.1, "x covariance for ballistic model").asDouble();
	v_eta27=config.check("v_eta27", /*0.005*/0.03, "y covariance for ballistic model").asDouble();
	v_eta37=config.check("v_eta37", /*0.005*/0.1, "z covariance for ballistic model").asDouble();
	
    //noise variances
    v_w1=config.check("v_w1", 5, "constant aceleration model noise").asDouble();
	v_w2=config.check("v_w2", /*0.07*/0.3, "wiener model noise").asDouble();
	v_w3=config.check("v_w3", 4, "singer model noise").asDouble();
	v_w4=config.check("v_w4", 0.7, "circular model chanel").asDouble();
	v_w5=config.check("v_w5", 0.7, "curvilinear model noise").asDouble();
	v_w6=config.check("v_w6", /*0.3*/2, "constant velocity model noise").asDouble();
	v_w7=config.check("v_w7", /*0.005*/5, "ballistic model noise").asDouble();

    init(	_actM, nact, 
			v_eta11,v_eta21,v_eta31,
			v_eta12,v_eta22,v_eta32,
			v_eta13,v_eta23,v_eta33,
			v_eta14,v_eta24,v_eta34,
			v_eta15,v_eta25,v_eta35,
			v_eta16,v_eta26,v_eta36,
			v_eta17,v_eta27,v_eta37,
			v_w1,v_w2,v_w3,v_w4,
			v_w5,v_w6,v_w7,
			sampleperiod);

    return true;
}


predMatrix<double> predictors::corrPred(predMatrix<double> x){ 
	int N=x.nLins(), n=0;
	predMatrix<double> x_b=x;
	 predMatrix<double> res(N,1),M(N,N);
	 double media=x.media();
	for(int i=0;i<N;i++)
		x.setValue(i,x.getValue(i)-media);

	 //correlation computation
	for(int k=N;k>0;k--)
		for(int i=0; i<k;i++)
			M.setValue((N-k)*N+i,x.getValue((N-k)+i));
	res=M*x; 
	res=res/res.getValue(0);

	//------------maximum detection-------------//
	double max=-2;
	int min=0, Tout=0, T;
	predMatrix<double> out(N,1);
	n=0;

	/*FILE *f1=fopen("correl.txt","w");
	for(int i=0;i<N;i++)
		fprintf(f1, "%f\n",res.getValue(i));*/

	for(int i=0;i<N;i++){
		if(res.getValue(i)<thm)
			min=1;
		if(res.getValue(i)>max&&min==1){
			T=i;
			max=res.getValue(i);
		}
	}
	if(max>thM)
		Tout=T;
	else
		Tout=0;

	//--------------\maximum detection------------//
	//---------------signal prediction------------//
	if(Tout!=0){
		n=0;
		int k;
		while(n<N){
			for(k=N%Tout;k<Tout&&n<N;k++,n++)
				out.setValue(n,x_b.getValue(k));
			for(int i=0;i<N%Tout&&n<N;i++,k++,n++)
				out.setValue(n,x_b.getValue(i));
		}
	}
	else{ //no period detected
		out=x_b;
	}

	/*FILE *f1=fopen("correl.txt","w");
	for(int i=0;i<N;i++)
		fprintf(f1, "%f\n",out.getValue(i));
	fclose(f1);*/
	return out;
	//-------------\signal prediction-------------//
}


bool predictors::chisq(predMatrix<double> rPos, predMatrix<double> out){ 
	predMatrix<double> aux=rPos,aux1=out,aux3=rPos-out;
	for(int i=0;i<rPos.nLins();i++){
		aux.setValue(i,rPos.getValue(i)-rPos.media());
		aux1.setValue(i,out.getValue(i)-out.media());
	}
	aux=aux.Transposta()*aux1;
	double cov=(double)fabs((double)aux.getValue(0)/(out.nLins()-1.0));
	predMatrix<double> aux4=(rPos-out)*cov*aux3.Transposta();
	//cout<<aux4.getValue(0)<<endl;
	if(aux4.getValue(0)>1.3e-4/*0.0623*/&&aux4.getValue(0)<0.2419){ //ChiSq_3(0.025)<aux4<ChiSq_3(0.975)
		//cout<<aux4.getValue(0)<<endl;
		return true;
	}
	else
		return false;
}


void predictors::init(int actM[NMODELS+1], int _nact, 
					  double v_eta11, double v_eta21,double v_eta31,
					  double v_eta12,double v_eta22, double v_eta32,
					  double v_eta13,double v_eta23,double v_eta33,
					  double v_eta14,double v_eta24,double v_eta34,
					  double v_eta15,double v_eta25,double v_eta35,
					  double v_eta16,double v_eta26,double v_eta36,
					  double v_eta17,double v_eta27,double v_eta37,
					  double v_w1,double v_w2,double v_w3,
					  double v_w4,double v_w5,double v_w6,
					  double v_w7, double T)
{

	predMatrix<double> At[NMODELS], Bt[NMODELS], Ct[NMODELS], Qt[NMODELS], Pt[NMODELS], Pet[NMODELS], Rt[NMODELS], kt[NMODELS], xt[NMODELS],xet[NMODELS], xauxt[NMODELS];
	predMatrix<double> Ia(9,9,1);
	predMatrix<double> I2a(6,6,1);
	predMatrix<double> A1(9,9);
	predMatrix<double> B1(9,1);
	predMatrix<double> C1(3,9);
	predMatrix<double> Q1(9,9);
	predMatrix<double> P1(9,9);
	predMatrix<double> Pe1=Ia; 
	predMatrix<double> R1(3,3);
	predMatrix<double> k1(9,3);
	predMatrix<double> x1(9,1);
	predMatrix<double> xe1(9,1);
	predMatrix<double> xaux1(9,1);

	predMatrix<double> corraux(NCORR,1);
	corrm[0]=corraux;corrm[1]=corraux;corrm[2]=corraux;

	_Ts = T;

	if(actM[NMODELS]==1){
		corr=1;
		nact=_nact-1;
	}
	else{
		corr=0;
		nact=_nact;
	}
	u=1; 

	int maux=-1;
	spM=-1;
	bllM=-1;
	for(int i=0;i<NMODELS; i++){
		if(actM[i]==1)
			maux++;
		if(i==5&&actM[i]==1)
			spM=maux;
		if(i==6&&actM[i]==1)
			bllM=maux;
	}

	//prior model probabilities - start uniform
	for (int i=0;i<min(nact,NMODELS);i++) 
		p[i]=(1.f/nact);

	//***********************************
	//CA1 <---> 0
	//CONSTANT ACCELERATION (X,Y,Z)-same parameters to all directions!
	//double v_w1=5; //model noise

	/*bons valores para a combinação de probabilidades
	double v_eta11=0.05,v_eta21=0.05,v_eta31=0.05;
	double v_eta12=0.05,v_eta22=0.05,v_eta32=0.05;
	double v_eta13=0.05,v_eta23=0.05,v_eta33=0.05;
	double v_eta14=0.04,v_eta24=0.04,v_eta34=0.1;
	double v_eta15=0.04,v_eta25=0.04,v_eta35=0.1;
	double v_eta16=0.05,v_eta26=0.05,v_eta36=0.06;
	double v_eta17=0.005,v_eta27=0.005,v_eta37=0.005;*/
	
	 
	
	
	

	//melhor combinação de ruídos:
	/*double v_eta11=0.07,v_eta21=0.07,v_eta31=0.1;
	double v_eta12=0.07,v_eta22=0.03,v_eta32=0.12;
	double v_eta13=0.05,v_eta23=0.1,v_eta33=0.1;
	//v_eta14,24 e v_eta15,25 tinham 0.04. v_eta34,35 tinham 0.22!!!
	double v_eta14=0.04,v_eta24=0.04,v_eta34=0.22;
	double v_eta15=0.04,v_eta25=0.04,v_eta35=0.22;
	double v_eta16=0.1,v_eta26=0.1,v_eta36=0.1;
	double v_eta17=0.005,v_eta27=0.005,v_eta37=0.005;*/
	
	A1.setValue(0,1);A1.setValue(1,T);A1.setValue(2,T*T/2);
	A1.setValue(9,0);A1.setValue(10,1);A1.setValue(11,T);
	A1.setValue(18,0);A1.setValue(19,0);A1.setValue(20,1);
	A1.setValue(30,1);A1.setValue(31,T);A1.setValue(32,T*T/2);
	A1.setValue(39,0);A1.setValue(40,1);A1.setValue(41,T);
	A1.setValue(48,0);A1.setValue(49,0);A1.setValue(50,1);
	A1.setValue(60,1);A1.setValue(61,T);A1.setValue(62,T*T/2);
	A1.setValue(69,0);A1.setValue(70,1);A1.setValue(71,T);
	A1.setValue(78,0);A1.setValue(79,0);A1.setValue(80,1);
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);B1.setValue(6,0);B1.setValue(7,0);B1.setValue(8,0);
	C1.setValue(0,1);C1.setValue(1,0);C1.setValue(2,0);
	C1.setValue(12,1);C1.setValue(13,0);C1.setValue(14,0);
	C1.setValue(24,1);C1.setValue(25,0);C1.setValue(26,0);
	Q1.setValue(0,pow(T,5)/20);Q1.setValue(1,pow(T,4)/8);Q1.setValue(2,pow(T,3)/6);
	Q1.setValue(9,pow(T,4)/8);Q1.setValue(10,pow(T,3)/3);Q1.setValue(11,pow(T,2)/2);
	Q1.setValue(18,pow(T,3)/6);Q1.setValue(19,T*T/2);Q1.setValue(20,T);
	Q1.setValue(30,pow(T,5)/20);Q1.setValue(31,pow(T,4)/8);Q1.setValue(32,pow(T,3)/6);
	Q1.setValue(39,pow(T,4)/8);Q1.setValue(40,pow(T,3)/3);Q1.setValue(41,pow(T,2)/2);
	Q1.setValue(48,pow(T,3)/6);Q1.setValue(49,T*T/2);Q1.setValue(50,T);
	Q1.setValue(60,pow(T,5)/20);Q1.setValue(61,pow(T,4)/8);Q1.setValue(62,pow(T,3)/6);
	Q1.setValue(69,pow(T,4)/8);Q1.setValue(70,pow(T,3)/3);Q1.setValue(71,pow(T,2)/2);
	Q1.setValue(78,pow(T,3)/6);Q1.setValue(79,T*T/2);Q1.setValue(80,T);
	Q1=Q1*v_w1;
	//assuming no covariance between the sensored data
	R1.setValue(0,v_eta11);R1.setValue(4,v_eta21);R1.setValue(8,v_eta31);
	
	At[0]=A1;
	Bt[0]=B1;
	Ct[0]=C1;
	Qt[0]=Q1;
	Pt[0]=P1;
	Pet[0]=Pe1;
	Rt[0]=R1;
	kt[0]=k1;
	xt[0]=x1;
	xet[0]=xe1;
	xauxt[0]=xaux1;

	//xe=[0 0 0 0 0 0 0 0 0]'
	/***********************************/
	
	
	
	
	//***********************************
	//CA2 <---> 1
	//WIENER-SEQUENCE ACCELERATION MODEL (X,Y,Z)
	//double v_w2=0.07; //model noise
	//double v_eta12=0.01,v_eta22=0.3,v_eta32=0.3; //sensoring noise FAN:double v_eta1=0.1,v_eta2=0.003,v_eta3=0.3;
	
	A1.setValue(0,1);A1.setValue(1,T);A1.setValue(2,T*T/2);
	A1.setValue(9,0);A1.setValue(10,1);A1.setValue(11,T);
	A1.setValue(18,0);A1.setValue(19,0);A1.setValue(20,1);
	A1.setValue(30,1);A1.setValue(31,T);A1.setValue(32,T*T/2);
	A1.setValue(39,0);A1.setValue(40,1);A1.setValue(41,T);
	A1.setValue(48,0);A1.setValue(49,0);A1.setValue(50,1);
	A1.setValue(60,1);A1.setValue(61,T);A1.setValue(62,T*T/2);
	A1.setValue(69,0);A1.setValue(70,1);A1.setValue(71,T);
	A1.setValue(78,0);A1.setValue(79,0);A1.setValue(80,1);
	/*B1.setValue(0,T*T/2);B1.setValue(1,T);B1.setValue(2,1);
	B1.setValue(3,T*T/2);B1.setValue(4,T);B1.setValue(5,1);
	B1.setValue(6,T*T/2);B1.setValue(7,T);B1.setValue(8,1);*/
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);B1.setValue(6,0);B1.setValue(7,0);B1.setValue(8,0);
	C1.setValue(0,1);C1.setValue(1,0);C1.setValue(2,0);
	C1.setValue(12,1);C1.setValue(13,0);C1.setValue(14,0);
	C1.setValue(24,1);C1.setValue(25,0);C1.setValue(26,0);
	Q1.setValue(0,pow(T,4)/4);Q1.setValue(1,pow(T,3)/2);Q1.setValue(2,pow(T,2)/2);
	Q1.setValue(9,pow(T,3)/2);Q1.setValue(10,pow(T,2)/2);Q1.setValue(11,T);
	Q1.setValue(18,pow(T,2)/2);Q1.setValue(19,T);Q1.setValue(20,1);
	Q1.setValue(30,pow(T,4)/4);Q1.setValue(31,pow(T,3)/2);Q1.setValue(32,pow(T,2)/2);
	Q1.setValue(39,pow(T,3)/2);Q1.setValue(40,pow(T,2)/2);Q1.setValue(41,T);
	Q1.setValue(48,pow(T,2)/2);Q1.setValue(49,T);Q1.setValue(50,1);
	Q1.setValue(60,pow(T,4)/4);Q1.setValue(61,pow(T,3)/2);Q1.setValue(62,pow(T,2)/2);
	Q1.setValue(69,pow(T,3)/2);Q1.setValue(70,pow(T,2)/2);Q1.setValue(71,T);
	Q1.setValue(78,pow(T,2)/2);Q1.setValue(79,T);Q1.setValue(80,1);
	Q1=Q1*v_w2;
	R1.setValue(0,v_eta12);R1.setValue(4,v_eta22);R1.setValue(8,v_eta32);
	At[1]=A1;
	Bt[1]=B1;
	Ct[1]=C1;
	Qt[1]=Q1;
	Pt[1]=P1;
	Pet[1]=Pe1;
	Rt[1]=R1;
	kt[1]=k1;
	xt[1]=x1;
	xet[1]=xe1;
	xauxt[1]=xaux1;

	//xe1=[0 0 0 0 0 0 0 0 0]'
	/***********************************/


	


	//***********************************
	//CA3 <---> 2
	//SINGER ACCELERATION MODEL-ZERO MEAN FIRST ORDER MARKOV MODEL (X,Y,Z)
	//double v_w3=0.002; //model noise
	//double v_eta13=0.1,v_eta23=0.5,v_eta33=0.5; //sensoring noise FAN:double v_eta1=0.05,v_eta2=0.5,v_eta3=0.5;
	double alfa=0.3; //reciprocal of the tracked object time constant

	A1.setValue(0,1);A1.setValue(1,T);A1.setValue(2,(alfa*T-1+exp(-alfa*T))/(alfa*alfa));
	A1.setValue(9,0);A1.setValue(10,1);A1.setValue(11,(1-exp(-alfa*T))/alfa);
	A1.setValue(18,0);A1.setValue(19,0);A1.setValue(20,exp(-alfa*T));
	A1.setValue(30,1);A1.setValue(31,T);A1.setValue(32,(alfa*T-1+exp(-alfa*T))/(alfa*alfa));
	A1.setValue(39,0);A1.setValue(40,1);A1.setValue(41,(1-exp(-alfa*T))/alfa);
	A1.setValue(48,0);A1.setValue(49,0);A1.setValue(50,exp(-alfa*T));
	A1.setValue(60,1);A1.setValue(61,T);A1.setValue(62,(alfa*T-1+exp(-alfa*T))/(alfa*alfa));
	A1.setValue(69,0);A1.setValue(70,1);A1.setValue(71,(1-exp(-alfa*T))/alfa);
	A1.setValue(78,0);A1.setValue(79,0);A1.setValue(80,exp(-alfa*T));
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);B1.setValue(6,0);B1.setValue(7,0);B1.setValue(8,0);
	C1.setValue(0,1);C1.setValue(1,0);C1.setValue(2,0);
	C1.setValue(12,1);C1.setValue(13,0);C1.setValue(14,0);
	C1.setValue(24,1);C1.setValue(25,0);C1.setValue(26,0);
	//This matrix is filled as presented in [26]
	Q1.setValue(0,(1/(2*pow(alfa,5)))*(1-exp(-2*alfa*T)+2*alfa*T+(2*pow(alfa,3)*pow(T,3))/3-2*pow(alfa,2)*pow(T,2)-4*alfa*T*exp(-alfa*T)));
	Q1.setValue(1,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(2,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(9,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(10,(1/(2*pow(alfa,3)))*(4*exp(-alfa*T)-3-exp(-2*alfa*T)+2*alfa*T));
	Q1.setValue(11,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(18,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(19,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(20,(1/(2*alfa))*(1-exp(-2*alfa*T)));
	Q1.setValue(30,(1/(2*pow(alfa,5)))*(1-exp(-2*alfa*T)+2*alfa*T+(2*pow(alfa,3)*pow(T,3))/3-2*pow(alfa,2)*pow(T,2)-4*alfa*T*exp(-alfa*T)));
	Q1.setValue(31,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(32,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(39,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(40,(1/(2*pow(alfa,3)))*(4*exp(-alfa*T)-3-exp(-2*alfa*T)+2*alfa*T));
	Q1.setValue(41,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(48,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(49,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(50,(1/(2*alfa))*(1-exp(-2*alfa*T)));
	Q1.setValue(60,(1/(2*pow(alfa,5)))*(1-exp(-2*alfa*T)+2*alfa*T+(2*pow(alfa,3)*pow(T,3))/3-2*pow(alfa,2)*pow(T,2)-4*alfa*T*exp(-alfa*T)));
	Q1.setValue(61,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(62,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(69,(1/(2*pow(alfa,4)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)+2*alfa*T*exp(-alfa*T)-2*alfa*T+pow(alfa,2)*pow(T,2)));
	Q1.setValue(70,(1/(2*pow(alfa,3)))*(4*exp(-alfa*T)-3-exp(-2*alfa*T)+2*alfa*T));
	Q1.setValue(71,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(78,(1/(2*pow(alfa,3)))*(1-exp(-2*alfa*T)-2*alfa*T*exp(-alfa*T)));
	Q1.setValue(79,(1/(2*pow(alfa,2)))*(exp(-2*alfa*T)+1-2*exp(-alfa*T)));
	Q1.setValue(80,(1/(2*alfa))*(1-exp(-2*alfa*T)));
	double amax=1, Pm=0.1, P0=0.15;  //this are model's design paramters
	v_w3=2*alfa*(amax*amax/3)*(1+4*Pm-P0); //using the ternary-uniform misture
	Q1=Q1*v_w3;
	R1.setValue(0,v_eta13);R1.setValue(4,v_eta23);R1.setValue(8,v_eta33);
	At[2]=A1;
	Bt[2]=B1;
	Ct[2]=C1;
	Qt[2]=Q1;
	Pt[2]=P1;
	Pet[2]=Pe1;
	Rt[2]=R1;
	kt[2]=k1;
	xt[2]=x1;
	xet[2]=xe1;
	xauxt[2]=xaux1;

	//xe1=[0 0 0 0 0 0 0 0 0]'
	/***********************************/

	


	//***********************************
	//CIRC <---> 3
	//CIRCULAR MOTION MODEL (CONSTANT TURN MODEL)- 3D
	//This model needs a good estimation for w!
	//double v_w4=0.7; //model noise
	//double v_eta14=0.1,v_eta24=0.5,v_eta34=0.9; //sensoring noise
	double w=1.75; //This frequency couples the directions!
	//w2=1.95
	
	A1.setValue(0,1);A1.setValue(1,sin(w*T)/w);A1.setValue(2,(1-cos(w*T))/(w*w));
	A1.setValue(9,0);A1.setValue(10,cos(w*T));A1.setValue(11,sin(w*T)/w);
	A1.setValue(18,0);A1.setValue(19,-w*sin(w*T));A1.setValue(20,cos(w*T));
	A1.setValue(30,1);A1.setValue(31,sin(w*T)/w);A1.setValue(32,(1-cos(w*T))/(w*w));
	A1.setValue(39,0);A1.setValue(40,cos(w*T));A1.setValue(41,sin(w*T)/w);
	A1.setValue(48,0);A1.setValue(49,-w*sin(w*T));A1.setValue(50,cos(w*T));
	A1.setValue(60,1);A1.setValue(61,sin(w*T)/w);A1.setValue(62,(1-cos(w*T))/(w*w));
	A1.setValue(69,0);A1.setValue(70,cos(w*T));A1.setValue(71,sin(w*T)/w);
	A1.setValue(78,0);A1.setValue(79,-w*sin(w*T));A1.setValue(80,cos(w*T));
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);B1.setValue(6,0);B1.setValue(7,0);B1.setValue(8,0);
	C1.setValue(0,1);C1.setValue(1,0);C1.setValue(2,0);
	C1.setValue(12,1);C1.setValue(13,0);C1.setValue(14,0);
	C1.setValue(24,1);C1.setValue(25,0);C1.setValue(26,0);
	Q1.setValue(0,(6*w*T-8*sin(w*T)+sin(2*w*T))/(4*pow(w,5)));Q1.setValue(1,(2*pow(sin(w*T/2),4))/pow(w,4));Q1.setValue(2,(-2*w*T+4*sin(w*T)-sin(w*T))/(4*pow(w,3)));
	Q1.setValue(9,2*pow(sin(w*T/2),4)/pow(w,4));Q1.setValue(10,(2*w*T-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(11,pow(sin(w*T),2)/(2*pow(w,2)));
	Q1.setValue(18,(-2*w*T+4*sin(w*T)-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(19,pow(sin(w*T),2)/(2*w*w));Q1.setValue(20,(2*w*T+sin(2*w*T))/(4*w));
	Q1.setValue(30,(6*w*T-8*sin(w*T)+sin(2*w*T))/(4*pow(w,5)));Q1.setValue(31,(2*pow(sin(w*T/2),4))/pow(w,4));Q1.setValue(32,(-2*w*T+4*sin(w*T)-sin(w*T))/(4*pow(w,3)));
	Q1.setValue(39,2*pow(sin(w*T/2),4)/pow(w,4));Q1.setValue(40,(2*w*T-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(41,pow(sin(w*T),2)/(2*pow(w,2)));
	Q1.setValue(48,(-2*w*T+4*sin(w*T)-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(49,pow(sin(w*T),2)/(2*w*w));Q1.setValue(50,(2*w*T+sin(2*w*T))/(4*w));
	Q1.setValue(60,(6*w*T-8*sin(w*T)+sin(2*w*T))/(4*pow(w,5)));Q1.setValue(61,(2*pow(sin(w*T/2),4))/pow(w,4));Q1.setValue(62,(-2*w*T+4*sin(w*T)-sin(w*T))/(4*pow(w,3)));
	Q1.setValue(69,2*pow(sin(w*T/2),4)/pow(w,4));Q1.setValue(70,(2*w*T-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(71,pow(sin(w*T),2)/(2*pow(w,2)));
	Q1.setValue(78,(-2*w*T+4*sin(w*T)-sin(2*w*T))/(4*pow(w,3)));Q1.setValue(79,pow(sin(w*T),2)/(2*w*w));Q1.setValue(80,(2*w*T+sin(2*w*T))/(4*w));
	Q1=Q1*v_w4; //using the same noise in all directions
	//assuming no covariance between the sensored data
	R1.setValue(0,v_eta14);R1.setValue(4,v_eta24);R1.setValue(8,v_eta34);
	At[3]=A1;
	Bt[3]=B1;
	Ct[3]=C1;
	Qt[3]=Q1;
	Pt[3]=P1;
	Pet[3]=Pe1;
	Rt[3]=R1;
	kt[3]=k1;
	xt[3]=x1;
	xet[3]=xe1;
	xauxt[3]=xaux1;
	/***********************************/


	//***********************************
	//CURV <---> 4
	//CURVILINEAR MOTION MODEL (VARAIBLE TURN MODEL)- 3D
	//double v_w5=0.7; //model noise
	//double v_eta15=0.009,v_eta25=0.009,v_eta35=2; //sensoring noise
	//double v_eta15=0.2,v_eta25=0.2,v_eta35=0.2;
	//double w=1.95,a=0.03; //a-damping coefficient; wc actual frequency
	double w1=1.7,w2=1.95,w3=10,a=0.28;
	//each coordinate may have its own w and a!!!
	
	A1.setValue(0,1);A1.setValue(1,(2*a*w1-exp(-a*T)*(2*a*w1*cos(w1*T)+(a+a-w1*w1)*sin(w1*T)))/(w1*(a*a+w1*w1)));A1.setValue(2,(w1-exp(-a*T)*(w1*cos(w1*T)+a*sin(w1*T)))/(w1*(a*a+w1*w1)));
	A1.setValue(9,0);A1.setValue(10,exp(-a*T)*(w1*cos(w1*T)+a*sin(w1*T))/w1);A1.setValue(11,exp(-a*T)*sin(w1*T)/w1);
	A1.setValue(18,0);A1.setValue(19,-(a*a+w1*w1)*exp(-a*T)*sin(w1*T)/w1);A1.setValue(20,exp(-a*T)*(w1*cos(w1*T)-a*sin(w1*T))/w1);	
	A1.setValue(30,1);A1.setValue(31,(2*a*w2-exp(-a*T)*(2*a*w2*cos(w2*T)+(a+a-w2*w2)*sin(w2*T)))/(w2*(a*a+w2*w2)));A1.setValue(32,(w2-exp(-a*T)*(w2*cos(w2*T)+a*sin(w2*T)))/(w2*(a*a+w2*w2)));
	A1.setValue(39,0);A1.setValue(40,exp(-a*T)*(w2*cos(w2*T)+a*sin(w2*T))/w2);A1.setValue(41,exp(-a*T)*sin(w2*T)/w2);
	A1.setValue(48,0);A1.setValue(49,-(a*a+w2*w2)*exp(-a*T)*sin(w2*T)/w2);A1.setValue(50,exp(-a*T)*(w2*cos(w2*T)-a*sin(w2*T))/w2);
	A1.setValue(60,1);A1.setValue(61,(2*a*w3-exp(-a*T)*(2*a*w3*cos(w3*T)+(a+a-w3*w3)*sin(w3*T)))/(w3*(a*a+w3*w3)));A1.setValue(62,(w3-exp(-a*T)*(w3*cos(w3*T)+a*sin(w3*T)))/(w3*(a*a+w3*w3)));
	A1.setValue(69,0);A1.setValue(70,exp(-a*T)*(w3*cos(w3*T)+a*sin(w3*T))/w3);A1.setValue(71,exp(-a*T)*sin(w3*T)/w3);
	A1.setValue(78,0);A1.setValue(79,-(a*a+w3*w3)*exp(-a*T)*sin(w3*T)/w3);A1.setValue(80,exp(-a*T)*(w3*cos(w3*T)-a*sin(w3*T))/w3);
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);B1.setValue(6,0);B1.setValue(7,0);B1.setValue(8,0);
	C1.setValue(0,1);C1.setValue(1,0);C1.setValue(2,0);
	C1.setValue(12,1);C1.setValue(13,0);C1.setValue(14,0);
	C1.setValue(24,1);C1.setValue(25,0);C1.setValue(26,0);
	double G1,G2,G3,H1,H2,H3,J1,J2,J3,c1,c2,c3,s1,s2,s3,co1,co2,co3,so1,so2,so3;
	c1=a*a*cos(2*w1*T);c2=a*a*cos(2*w2*T);c3=a*a*cos(2*w3*T);
	s1=a*w1*sin(2*w1*T);s2=a*w2*sin(2*w2*T);s3=a*w3*sin(2*w3*T);
	co1=cos(w1*T);co2=cos(w2*T);co3=cos(w3*T);
	so1=sin(w1*T);so2=sin(w2*T);so3=sin(w3*T);
	G1=exp(-2*a*T)*((a*a-3*w1*w1)*c1+(w1*w1-3*a*a)*s1-pow((a*a+w1*w1),2));G2=exp(-2*a*T)*((a*a-3*w2*w2)*c2+(w2*w2-3*a*a)*s2-pow((a*a+w2*w2),2));G3=exp(-2*a*T)*((a*a-3*w3*w3)*c3+(w3*w3-3*a*a)*s3-pow((a*a+w3*w3),2));
	H1=8*exp(-a*T)*a*w1*(2*a*w1*co1+(a*a-w1*w1)*so1);H2=8*exp(-a*T)*a*w2*(2*a*w2*co2+(a*a-w2*w2)*so2);H3=8*exp(-a*T)*a*w3*(2*a*w3*co3+(a*a-w3*w3)*so3);
	J1=a*a*w1*w1*(4*a*T-11)+pow(w1,4)*(1+4*a*T);J2=a*a*w2*w2*(4*a*T-11)+pow(w2,4)*(1+4*a*T);J3=a*a*w3*w3*(4*a*T-11)+pow(w3,4)*(1+4*a*T);
	Q1.setValue(0,(G1+H1+J1)/(4*a*w1*w1*pow((a*a+w1*w1),3)));Q1.setValue(1,(exp(-2*a*T)*pow((w1*co1+a*so1-exp(-a*T)*w1),2))/(2*w1*w1*pow((a*a+w1*w1),2)));Q1.setValue(2,(exp(-2*a*T)*(c1-s1-a*a+w1*w1)+4*exp(-a*T)*a*w1*so1-w1*w1)/(4*a*w1*w1*(a*a+w1*w1)));
	Q1.setValue(9,(exp(-2*a*T)*pow((w1*co1+a*so1-exp(-a*T)*w1),2))/(2*w1*w1*pow((a*a+w1*w1),2)));Q1.setValue(10,(exp(-2*a*T)*(c1-s1-a*a-w1*w1)+w1*w1)/(4*a*w1*w1*(a*a+w1*w1)));Q1.setValue(11,exp(-2*a*T)*so1*so1/(2*w1*w1));
	Q1.setValue(18,(exp(-2*a*T)*(c1-s1-a*a+w1*w1)+4*exp(-a*T)*a*w1*so1-w1*w1)/(4*a*w1*w1*(a*a+w1*w1)));Q1.setValue(19,exp(-2*a*T)*so1*so1/(2*w1*w1));Q1.setValue(20,(exp(-2*a*T)*(c1+s1-a*a-w1*w1)+w1*w1)/(4*a*w1*w1));
	Q1.setValue(30,(G2+H2+J2)/(4*a*w2*w2*pow((a*a+w2*w2),3)));Q1.setValue(31,(exp(-2*a*T)*pow((w2*co2+a*so2-exp(-a*T)*w2),2))/(2*w2*w2*pow((a*a+w2*w2),2)));Q1.setValue(32,(exp(-2*a*T)*(c2-s2-a*a+w2*w2)+4*exp(-a*T)*a*w2*so2-w2*w2)/(4*a*w2*w2*(a*a+w2*w2)));
	Q1.setValue(39,(exp(-2*a*T)*pow((w2*co2+a*so2-exp(-a*T)*w2),2))/(2*w2*w2*pow((a*a+w2*w2),2)));Q1.setValue(40,(exp(-2*a*T)*(c2-s2-a*a-w2*w2)+w2*w2)/(4*a*w2*w2*(a*a+w2*w2)));Q1.setValue(41,exp(-2*a*T)*so2*so2/(2*w2*w2));
	Q1.setValue(48,(exp(-2*a*T)*(c2-s2-a*a+w2*w2)+4*exp(-a*T)*a*w2*so2-w2*w2)/(4*a*w2*w2*(a*a+w2*w2)));Q1.setValue(49,exp(-2*a*T)*so2*so2/(2*w2*w2));Q1.setValue(50,(exp(-2*a*T)*(c2+s2-a*a-w2*w2)+w2*w2)/(4*a*w2*w2));
	Q1.setValue(60,(G3+H3+J3)/(4*a*w3*w3*pow((a*a+w3*w3),3)));Q1.setValue(61,(exp(-2*a*T)*pow((w3*co3+a*so3-exp(-a*T)*w3),2))/(2*w3*w3*pow((a*a+w3*w3),2)));Q1.setValue(62,(exp(-2*a*T)*(c3-s3-a*a+w3*w3)+4*exp(-a*T)*a*w3*so3-w3*w3)/(4*a*w3*w3*(a*a+w3*w3)));
	Q1.setValue(69,(exp(-2*a*T)*pow((w3*co3+a*so3-exp(-a*T)*w3),2))/(2*w3*w3*pow((a*a+w3*w3),2)));Q1.setValue(70,(exp(-2*a*T)*(c3-s3-a*a-w3*w3)+w3*w3)/(4*a*w3*w3*(a*a+w3*w3)));Q1.setValue(71,exp(-2*a*T)*so3*so3/(2*w3*w3));
	Q1.setValue(78,(exp(-2*a*T)*(c3-s3-a*a+w3*w3)+4*exp(-a*T)*a*w3*so3-w3*w3)/(4*a*w3*w3*(a*a+w3*w3)));Q1.setValue(79,exp(-2*a*T)*so3*so3/(2*w3*w3));Q1.setValue(80,(exp(-2*a*T)*(c3+s3-a*a-w3*w3)+w3*w3)/(4*a*w3*w3));
	Q1=Q1*v_w5; //using the same noise in all directions
	//assuming no covariance between the sensored data
	R1.setValue(0,v_eta15);R1.setValue(4,v_eta25);R1.setValue(8,v_eta35);  
	At[4]=A1;
	Bt[4]=B1;
	Ct[4]=C1;
	Qt[4]=Q1;
	Pt[4]=P1;
	Pet[4]=Pe1;
	Rt[4]=R1;
	kt[4]=k1;
	xt[4]=x1;
	xet[4]=xe1;
	xauxt[4]=xaux1;
	
	/***********************************/

	//***********************************
	//CV <---> 5
	//CONSTANT VELOCITY (X,Y,Z) - same velocity to all coordinates!
	//double v_w6=0.3;
	predMatrix<double> A2(6,6);
	predMatrix<double> B2(6,1);
	predMatrix<double> C2(3,6);
	predMatrix<double> Q2(6,6);
	predMatrix<double> P2(6,6);
	predMatrix<double> Pe2=I2a; 
	predMatrix<double> k2(6,3);
	predMatrix<double> x2(6,1);
	predMatrix<double> xaux2(6,1);
	predMatrix<double> xe2(6,1);
	A2.setValue(0,1);A2.setValue(1,T);
	A2.setValue(6,0);A2.setValue(7,1);
	A2.setValue(14,1);A2.setValue(15,T);
	A2.setValue(20,0);A2.setValue(21,1);
	A2.setValue(28,1);A2.setValue(29,T);
	A2.setValue(34,0);A2.setValue(35,1);
	/*B2.setValue(0,T*T/2);
	B2.setValue(1,T);
	B2.setValue(2,T*T/2);
	B2.setValue(3,T);
	B2.setValue(4,T*T/2);
	B2.setValue(5,T);*/
	B1.setValue(0,0);B1.setValue(1,0);B1.setValue(2,0);B1.setValue(3,0);B1.setValue(4,0);B1.setValue(5,0);
	C2.setValue(0,1);C2.setValue(1,0);
	C2.setValue(8,1);C2.setValue(9,0);
	C2.setValue(16,1);C2.setValue(17,0);
	Q2.setValue(0,pow(T,4)/4);Q2.setValue(1,pow(T,3)/2);
	Q2.setValue(6,pow(T,3)/2);Q2.setValue(7,pow(T,2));
	Q2.setValue(14,pow(T,4)/4);Q2.setValue(15,pow(T,3)/2);
	Q2.setValue(20,pow(T,3)/2);Q2.setValue(21,pow(T,2));
	Q2.setValue(28,pow(T,4)/4);Q2.setValue(29,pow(T,3)/2);
	Q2.setValue(34,pow(T,3)/2);Q2.setValue(35,pow(T,2));	
	Q2=Q2*v_w6;
	R1.setValue(0,v_eta16);R1.setValue(4,v_eta26);R1.setValue(8,v_eta36);
	At[5]=A2;
	Bt[5]=B2;
	Ct[5]=C2;
	Qt[5]=Q2;
	Pt[5]=P2;
	Pet[5]=Pe2;
	Rt[5]=R1;
	kt[5]=k2;
	xt[5]=x2;
	xet[5]=xe2;
	xauxt[5]=xaux2;
	
	/***********************************/


	//***********************************
	//BL <---> 6
	//BALLISTIC [LINEAR](X,Y,Z)
	//double v_w7=0.005;
	A2.setValue(0,1);A2.setValue(1,T);A2.setValue(2,0);A2.setValue(3,0);A2.setValue(4,0);A2.setValue(5,0);
	A2.setValue(6,0);A2.setValue(7,0);A2.setValue(8,0);A2.setValue(9,0);A2.setValue(10,0);A2.setValue(11,0);
	A2.setValue(12,0);A2.setValue(13,0);A2.setValue(14,1);A2.setValue(15,T);A2.setValue(16,0);A2.setValue(17,0);
	A2.setValue(18,0);A2.setValue(19,0);A2.setValue(20,0);A2.setValue(21,1);A2.setValue(22,0);A2.setValue(23,0);
	A2.setValue(24,0);A2.setValue(25,0);A2.setValue(26,0);A2.setValue(27,0);A2.setValue(28,1);A2.setValue(29,T);
	A2.setValue(30,0);A2.setValue(31,0);A2.setValue(32,0);A2.setValue(33,0);A2.setValue(34,0);A2.setValue(35,0);
	B2.setValue(0,0);B2.setValue(1,0);B2.setValue(2,0);B2.setValue(3,-0.45*9.8*T*T);B2.setValue(4,0);B2.setValue(5,0);
	Q2.setValue(0, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(1, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(2, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(3, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(4, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(5, 0.25*9.8*9.8*T*pow((2+T), 2));
	Q2.setValue(6, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(7, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(8, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(9, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(10, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(11, 0.25*9.8*9.8*T*pow((2+T), 2));
	Q2.setValue(12, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(13, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(14, 0.25*9.8*9.8*(-4*T+pow(T,3)+4*exp(T)*(T+(exp(T)-exp(-T))/2)));Q2.setValue(15, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(16, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(17, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));
	Q2.setValue(18, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(19, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(20, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(21, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(22, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(23, 0.25*9.8*9.8*T*pow((2+T), 2));
	Q2.setValue(24, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(25, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(26, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(27, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(28, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(29, 0.25*9.8*9.8*T*pow((2+T), 2));
	Q2.setValue(30, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(31, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(32, 0.25*9.8*9.8*(2+T)*(-2 +2+exp(T)+T*T));Q2.setValue(33, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(34, 0.25*9.8*9.8*T*pow((2+T), 2));Q2.setValue(35, 0.25*9.8*9.8*T*pow((2+T), 2));
	Q2=Q2*v_w7;
	C2.setValue(0,1);C2.setValue(1,0);
	C2.setValue(8,1);C2.setValue(9,0);
	C2.setValue(16,1);C2.setValue(17,0);
	R1.setValue(0,v_eta17);R1.setValue(4,v_eta27);R1.setValue(8,v_eta37);
	At[6]=A2;
	Bt[6]=B2;
	Ct[6]=C2;
	Qt[6]=Q2;
	Pt[6]=P2;
	Pet[6]=Pe2;
	Rt[6]=R1;
	kt[6]=k2;
	xt[6]=x2;
	xet[6]=xe2;
	xauxt[6]=xaux2;

	/***********************************/


	/***********************************
	//BL <---> 6
	//BALLISTIC [LINEAR](X,Y,Z)
	//double v_w7=0.005;
	A2.setValue(0,1);A2.setValue(1,T);
	A2.setValue(6,0);A2.setValue(7,1);
	A2.setValue(14,1);A2.setValue(15,T);
	A2.setValue(20,0);A2.setValue(21,1);
	A2.setValue(28,1);A2.setValue(29,T);
	A2.setValue(34,0);A2.setValue(35,1);
	B2.setValue(0,0);B2.setValue(1,0);B2.setValue(2,9.8*T*T);B2.setValue(3,T);B2.setValue(4,0);B2.setValue(5,0);
	//Q2.setValue(0, (-1+4*T*exp(T+T*T)+(1+T)*T*exp(2*T*T*T)-T*(6+T))/(2*T*(1+T)));Q2.setValue(1,-1+exp(T)+(-1+exp(T*T))/T+(-1+exp(T+T*T))/(1+T)+0.5*exp(T)*(exp(T)-exp(-T)) );
	//Q2.setValue(6, -1+exp(T)+(-1+exp(T*T))/T+(-1+exp(T+T*T))/(1+T)+0.5*exp(T)*(exp(T)-exp(-T)));Q2.setValue(7,-2+T+exp(T)*(2+0.5*exp(T)-exp(-T)) );
	//Q2.setValue(14, 0.5*(-1+exp(2*T)+exp(2*T*T)*9.8*9.8*T-(4*exp(T+T*T)*9.8*T)/(1+T)+9.8*T*(-9.8+4/(1+T))));Q2.setValue(15,-1+exp(T)+9.8-exp(T*T)*9.8*(9.8*9.8*T*T*(-1+exp(T+T*T)))/(1+T)-0.5*9.8*T*exp(T)*(exp(T)-exp(-T)));
	//Q2.setValue(20, -1+exp(T)+9.8-exp(T*T)*9.8*(9.8*9.8*T*T*(-1+exp(T+T*T)))/(1+T)-0.5*9.8*T*exp(T)*(exp(T)-exp(-T)));Q2.setValue(21, 0.5*T*9.8*(2+(-1+exp(T))*(-4+9.8*T*(1+exp(T)))));
	//Q2.setValue(28, (-1+4*T*exp(T+T*T)+(1+T)*T*exp(2*T*T*T)-T*(6+T))/(2*T*(1+T)));Q2.setValue(29,-1+exp(T)+(-1+exp(T*T))/T+(-1+exp(T+T*T))/(1+T)+0.5*exp(T)*(exp(T)-exp(-T)) );
	//Q2.setValue(34, -1+exp(T)+(-1+exp(T*T))/T+(-1+exp(T+T*T))/(1+T)+0.5*exp(T)*(exp(T)-exp(-T)));Q2.setValue(35,-2+T+exp(T)*(2+0.5*exp(T)-exp(-T)) );Q2=Q2*v_w7;
	Q2.setValue(0,9*pow(T,4)/4);Q2.setValue(1,3*pow(T,3)/2);
	Q2.setValue(6,3*pow(T,3)/2);Q2.setValue(7,T*T);
	Q2.setValue(14,pow(T,4)/4);Q2.setValue(15,-pow(T,3)/2);
	Q2.setValue(20,-pow(T,3)/2);Q2.setValue(21,T*T);
	Q2.setValue(28,9*pow(T,4)/4);Q2.setValue(29,3*pow(T,3)/2);
	Q2.setValue(34,3*pow(T,3)/2);Q2.setValue(35,T*T);
	
	C2.setValue(0,1);C2.setValue(1,0);
	C2.setValue(8,1);C2.setValue(9,0);
	C2.setValue(16,1);C2.setValue(17,0);
	R1.setValue(0,v_eta17);R1.setValue(4,v_eta27);R1.setValue(8,v_eta37);
	At[6]=A2;
	Bt[6]=B2;
	Ct[6]=C2;
	Qt[6]=Q2;
	Pt[6]=P2;
	Pet[6]=Pe2;
	Rt[6]=R1;
	kt[6]=k2;
	xt[6]=x2;
	xet[6]=xe2;
	xauxt[6]=xaux2;

	/***********************************/


	int aux1=0;
	for(int i=0;i<NMODELS;i++){
		if(actM[i]==1){
			A[aux1]=At[i];
			B[aux1]=Bt[i];
			C[aux1]=Ct[i];
			Q[aux1]=Qt[i];
			P[aux1]=Pt[i];
			Pe[aux1]=Pet[i];
			R[aux1]=Rt[i];
			k[aux1]=kt[i];
			x[aux1]=xt[i];
			xe[aux1]=xet[i];
			xaux[aux1]=xauxt[i];
			aux1++;
		}
	}
}

bool predictors::reset(double *realPos)
{
	predMatrix<double> Pt[NMODELS], Pet[NMODELS], kt[NMODELS], 
		               xt[NMODELS],xet[NMODELS], xauxt[NMODELS];

	predMatrix<double> I9(9,9,1);
	predMatrix<double> I6(6,6,1);
	predMatrix<double> P9(9,9);
	predMatrix<double> P6(6,6);
	predMatrix<double> x6(6,1);
	predMatrix<double> x9(9,1);


	//prior model probabilities - start uniform
	for (int i=0;i<min(nact,NMODELS);i++) 
		p[i]=(1.f/nact);

	//ca
	Pt[0]=P9;
	Pet[0]=I9;
	xet[0] = x9;
	xet[0].setValue(0,realPos[0]);
	xet[0].setValue(3,realPos[1]);
	xet[0].setValue(6,realPos[2]);


	//w
	Pt[1]=P9;
	Pet[1]=I9;
	xet[1] = x9;
	xet[1].setValue(0,realPos[0]);
	xet[1].setValue(3,realPos[1]);
	xet[1].setValue(6,realPos[2]);

	//s
	Pt[2]=P9;
	Pet[2]=I9;
	xet[2] = x9;
	xet[2].setValue(0,realPos[0]);
	xet[2].setValue(3,realPos[1]);
	xet[2].setValue(6,realPos[2]);

	//circ
	Pt[3]=P9;
	Pet[3]=I9;
	xet[3] = x9;
	xet[3].setValue(0,realPos[0]);
	xet[3].setValue(3,realPos[1]);
	xet[3].setValue(6,realPos[2]);

	//curv
	Pt[4]=P9;
	Pet[4]=I9;
	xet[4] = x9;
	xet[4].setValue(0,realPos[0]);
	xet[4].setValue(3,realPos[1]);
	xet[4].setValue(6,realPos[2]);

	//ca
	Pt[5]=P6;
	Pet[5]=I6;
	xet[5] = x6;
	xet[5].setValue(0,realPos[0]);
	xet[5].setValue(2,realPos[1]);
	xet[5].setValue(4,realPos[2]);

	//bll
	Pt[6]=P6;
	Pet[6]=I6;
	xet[6] = x6;
	xet[6].setValue(0,realPos[0]);
	xet[6].setValue(2,realPos[1]);
	xet[6].setValue(4,realPos[2]);

	int aux1=0;
	for(int i=0;i<NMODELS;i++){
		if(_actM[i]==1){
			P[aux1]=Pt[i];
			Pe[aux1]=Pet[i];
			k[aux1].Inicializa();
			x[aux1].Inicializa();
			xe[aux1]=xet[i];
			xaux[aux1].Inicializa();
			aux1++;
		}
	}
	return true;
}

void predictors::update_state(int j, predMatrix<double> &xe){
	double T=_Ts;
	if(up==0)
		return;
	updt=1;
	if(!cf){
		cf=1;
		aT=0;
	}
	else{
		if(xe.getValue(3)>0)  //need to performm the kalman filter adptation phase
			cf=-1;//abs(xe[j].getValue(3)/lsp);
		else
			cf=1;
		aT=0;
	}
	//pT=abs(2*xe[j].getValue(3)/(9.8*T));
	//lsp=abs(xe[j].getValue(3));
	//cout<<cf<<endl;
	A[j].setValue(15,-cf);
	//A[j].setValue(15,0);
	//A[j].setValue(21,-cf);
	xe.setValue(3,-cf*xe.getValue(3));
	//lsp=abs(xe[j].getValue(3));
	up=0;
}

void predictors::restore_state(int j){
	double T=_Ts;
	if(updt){
		//A[j].setValue(15,T);
		//A[j].setValue(21,1);
		A[j].setValue(15,1);
		updt=0;
	}
}


double*  predictors::predict(double _realPos[3], int nSteps, int _aT, FILE *f1){

	//////////////////////////////////////////////////////////////////////////////////////////
	//											MMAE										//
	//////////////////////////////////////////////////////////////////////////////////////////


	double q=0;
	predMatrix<double> xb(9,1),xb1(6,1); //estimated state at step k-1 (used for prediction)
	predMatrix<double> r(3,1); // filter residual vector
	predMatrix<double> e;
	predMatrix<double> Im(9,9,1),I2m(6,6,1),z(3,1);
	aT=_aT;
	realPos[0]=_realPos[0];realPos[1]=_realPos[1];realPos[2]=_realPos[2];

	if(aT<150)
		for (int i=0;i<min(nact,NMODELS);i++) 
			p[i]=(1.f/nact);
	
	


	if(SVERBOSE)
		for (int i=0;i<NMODELS;i++)
			cout<<"A"<<A[i]<<"B"<<B[i]<<"C"<<C[i]<<"Q"<<Q[i]<<"R"<<R[i]<<endl;


	double den=0;

	predMatrix<double> num(9,1);

	

	q=0;
	for (int j=0;j<nact;j++){
		//time update
		P[j]=A[j]*Pe[j]*A[j].Transposta()+Q[j];
		x[j]=A[j]*xe[j]+B[j]*u;
		if(j==bllM && xe[j].getValue(2)-y0<0.1){
			update_state(bllM,xe[j]);
			P[j]=A[j]*Pe[j]*A[j].Transposta()+Q[j];
			x[j]=A[j]*xe[j]+B[j]*u;
		}

		//measuremnent update
		k[j]=(P[j]*C[j].Transposta())/(C[j]*P[j]*C[j].Transposta()+R[j]);
		if(j==spM||j==bllM)
			Pe[j]=(I2m-k[j]*C[j])*P[j];
		else
			Pe[j]=(Im-k[j]*C[j])*P[j];
		z.setValue(0,realPos[0]);
		z.setValue(1,realPos[1]);
		z.setValue(2,realPos[2]);
		xe[j]=x[j]+k[j]*(z-C[j]*x[j]);

		if(j==bllM){
			if(xe[j].getValue(2)>1.f)
				up=1;
		}

					
		covR[j]=C[j]*P[j]*C[j].Transposta()+R[j];
		r=z-C[j]*x[j]; 
		e=(r.Transposta()/covR[j])*r;
		q+=(1/(pow(2*M_PI,4.5)*pow(fabs(covR[j].Determinante()),0.5)))*exp(-0.5*e.getValue(0))*p[j];
		if(bllM>=0)
			restore_state(bllM);
	}


	Max=0;
	double E=-1;
	int md=0;
	int up1=up;
	double cf1=cf;
	for(int l=0;l<nact;l++){
		//conditional probability
		r=z-C[l]*x[l]; 
		e=(r.Transposta()/covR[l])*r;
		p[l]=((1/(pow(2*M_PI,4.5)*pow(fabs(covR[l].Determinante()),0.5)))*exp(-0.5*e.getValue(0))*p[l])/q;
		if(p[l]>E){ //chooses the bigest probability - IT SHOULD BE IMPROVED!!!
			E=p[l];
			Max=l;
		}
		//using bayesian blending:
		//N steps prediction for each model
		xaux[l]=xe[l];
		for (int j=0;j<nSteps;j++){
			if(l==bllM&&j==3)
				break;
			if(l==spM||(l==bllM&&xaux[l].getValue(2)-y0>0.1)){
				xb1=A[l]*xaux[l]+B[l]*u;
				xaux[l]=xb1;
			}else{
				if(l==bllM && xaux[l].getValue(2)-y0<0.1){
					update_state(bllM,xaux[l]);
					xaux[l]=A[l]*xaux[l]+B[l]*u;
				}else{
					xb=A[l]*xaux[l]+B[l]*u;
					xaux[l]=xb;
				}
			}
			if(l==bllM){
				if(xaux[l].getValue(2)>1.f)
					up=1;
			}
			if(bllM>=0)
				restore_state(bllM);
		}
		if(p[l]>0.25){
			md++;
			if(l==spM||l==bllM){
				predMatrix<double> xaux1(9,1);
				xaux1.setValue(0,xaux[l].getValue(0));xaux1.setValue(1,xaux[l].getValue(1));xaux1.setValue(3,xaux[l].getValue(2));
				xaux1.setValue(4,xaux[l].getValue(3));xaux1.setValue(6,xaux[l].getValue(4));xaux1.setValue(7,xaux[l].getValue(5));
				num=num+xaux1*p[l];
			}
			else
				num=num+xaux[l]*p[l];

			den+=p[l];
		}
	}
	up=up1;cf=cf1;
	if(md==0){
		if(Max==spM||Max==bllM){
			num.setValue(0,xaux[Max].getValue(0));num.setValue(1,xaux[Max].getValue(1));num.setValue(3,xaux[Max].getValue(2));
			num.setValue(4,xaux[Max].getValue(3));num.setValue(6,xaux[Max].getValue(4));num.setValue(7,xaux[Max].getValue(5));
		}else
			num=xaux[Max];
		den=1;
	}

	//using bayesian blending:
	//(for compatibilty reasons the output vector is assigned to the position of the bigest conditional probability model)
	//xaux[Max]=num/den;


	/*//Making a hard dicision
	xaux[Max]=xe[Max];
	for (int j=0;j<nSteps;j++){
		if(Max==5||Max==6){
			xb1=A[Max]*xaux[Max];
			xaux[Max]=xb1;
		}else{
			xb=A[Max]*xaux[Max];
			xaux[Max]=xb;
		}
	}*/


	
	int auxtr=0;
	tr[0]=false;tr[1]=false;tr[2]=false;
	if(corr==1){//compute the correlation model and do the chi-sqaure test
		for(int i=0;i<3;i++) //fill the vector with 3*NCORR values
			corrm[i].setValue(itac,realPos[i]);
		itac++;
		
		if(itac==NCORR){ //the vector is filled
			itac=0; //restart counter
			for(int i=0; i<3;i++){
				predperiod[i]=corrPred(corrm[i]);  //compute the prediction using the correlation method
				detected=true;
			}	
		}

		for(int k=0;k<3;k++){
			for(int i=14;i>0;i--){
				t1[k].setValue(i,t1[k].getValue(i-1));
				t2[k].setValue(i,t2[k].getValue(i-1));
			}
			t1[k].setValue(0,realPos[k]);
			if(detected==true)
				t2[k].setValue(0,predperiod[k].getValue(itac));
		}

		if(detected==true){
			for(int i=0;i<3;i++){
				tr[i]=chisq(t1[i],t2[i]);  //do the chi-square test
			}
		}
	}

	if(tr[0]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[0]=predperiod[0].getValue(itac);
			else
				predPos[0]=predperiod[0].getValue(NCORR-1);
			auxtr++;
		}
		else
			predPos[0]=num.getValue(0)/den;
		
		if(tr[1]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[1]=predperiod[1].getValue(itac);
			else
				predPos[1]=predperiod[1].getValue(NCORR-1);
			auxtr++;
		}
		else
			predPos[1]=num.getValue(3)/den;

		if(tr[2]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[2]=predperiod[2].getValue(itac);
			else
				predPos[2]=predperiod[2].getValue(NCORR-1);
			auxtr++;
		}
		else
			predPos[2]=num.getValue(6)/den;

			//inverse kinematics with the predicted position
			/*if(Max==spM||Max==bllM){
				predPos[0]=xaux[Max].getValue(0);
				predPos[1]=xaux[Max].getValue(2);
				predPos[2]=xaux[Max].getValue(4);
			}else{*/
			//predPos[0]=num.getValue(0)/den;
			//predPos[1]=num.getValue(3)/den;
			//predPos[2]=num.getValue(6)/den;

			/*if(p[Max]>0.4){
				for(int i=0;i<nact;i++){
					if(i==bllM||i==spM){
						for(int j=0;j<3;j++){
							xe[i].setValue(2*j,num.getValue(3*j));
							xe[i].setValue(2*j+1,num.getValue(3*j+1));
						}
					}
					else{
						for(int j=0;j<3;j++){
							xe[i].setValue(3*j,num.getValue(3*j));
							xe[i].setValue(3*j+1,num.getValue(3*j+1));
							if(Max!=spM&&Max!=bllM)
								xe[i].setValue(3*j+2,num.getValue(3*j+2));
						}
					}
				}
			}*/

			
		

	if(f1!=NULL){
		for(int i=0;i<min(nact,NMODELS);i++)
			fprintf(f1, "%f ",p[i]);
		if(corr==1)
			fprintf(f1, "%d ",auxtr);
		if(auxtr==1)
			cout<<"corr"<<endl;
		fprintf(f1, "\n");
	}


	return predPos;
	
}

void predictors::Lpredict(double _realPos[3], int nSteps, FILE *f1){

	this->predict(_realPos, 0,300);

	predMatrix<double> xb(9,1),xb1(6,1);
	predMatrix<double> num(9,1);
	double den=0;
	int md=0;

	for(int l=0;l<nact;l++)
		xaux[l]=xe[l];

	for (int v=0;v<nSteps;v++){
		for(int k=0;k<num.nLins();k++)
			num.setValue(k,0);
		den=0;
		for(int l=0;l<nact;l++){
			if(l==spM||(l==bllM&&xaux[l].getValue(2)-y0>0.1)){
				xb1=A[l]*xaux[l]+B[l]*u;
				xaux[l]=xb1;
				//cout<<xaux[l]<<endl;
			}else{
				if(l==bllM && xaux[l].getValue(2)-y0<0.1){
					update_state(bllM,xaux[l]);
					xaux[l]=A[l]*xaux[l]+B[l]*u;
					//cout<<xaux[l]<<endl;
				}else{
					xb=A[l]*xaux[l]+B[l]*u;
					xaux[l]=xb;
				}
			}
			if(l==bllM){
				if(xaux[l].getValue(2)>1.f)
					up=1;
			}
			if(bllM>=0)
				restore_state(bllM);

			if(p[l]>0.25){
				md++;
				if(l==spM||l==bllM){
					predMatrix<double> xaux1(9,1);
					xaux1.setValue(0,xaux[l].getValue(0));xaux1.setValue(1,xaux[l].getValue(1));xaux1.setValue(3,xaux[l].getValue(2));
					xaux1.setValue(4,xaux[l].getValue(3));xaux1.setValue(6,xaux[l].getValue(4));xaux1.setValue(7,xaux[l].getValue(5));
					num=num+xaux1*p[l];
				}
				else
					num=num+xaux[l]*p[l];

				den+=p[l];
			}
		}
		if(md==0){
			if(Max==spM||Max==bllM){
				num.setValue(0,xaux[Max].getValue(0));num.setValue(1,xaux[Max].getValue(1));num.setValue(3,xaux[Max].getValue(2));
				num.setValue(4,xaux[Max].getValue(3));num.setValue(6,xaux[Max].getValue(4));num.setValue(7,xaux[Max].getValue(5));
			}else
				num=xaux[Max];
			den=1;
		}
		//xaux[Max]=num/den;


		int auxtr=0;
		itac++;
		if(itac==NCORR)
			exit(0);
		if(tr[0]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[0]=predperiod[0].getValue(itac);
			else
				predPos[0]=predperiod[0].getValue(NCORR);
			auxtr++;
		}
		else
			predPos[0]=num.getValue(0)/den;
		
		if(tr[1]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[1]=predperiod[1].getValue(itac);
			else
				predPos[1]=predperiod[1].getValue(NCORR);
			auxtr++;
		}
		else
			predPos[1]=num.getValue(3)/den;

		if(tr[2]==true||nact==0){  //if the test is favorable the prediction is made with this model!
			if(itac+nSteps<NCORR)
				predPos[2]=predperiod[2].getValue(itac);
			else
				predPos[2]=predperiod[2].getValue(NCORR);
			auxtr++;
		}
		else
			predPos[2]=num.getValue(6)/den;

			//inverse kinematics with the predicted position
			/*if(Max==spM||Max==bllM){
				predPos[0]=xaux[Max].getValue(0);
				predPos[1]=xaux[Max].getValue(2);
				predPos[2]=xaux[Max].getValue(4);
			}else{*/
	


		if(f1!=NULL){
			fprintf(f1, "%f %f %f\n",predPos[0],predPos[1],predPos[2]);
		}
	}

}

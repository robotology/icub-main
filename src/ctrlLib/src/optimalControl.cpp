
#include <iostream>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/optimalControl.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Riccati::Riccati(const Matrix &_A, const Matrix &_B, const Matrix &_V,
                 const Matrix &_P, const Matrix &_VN, bool verb) 
{
	A = _A; At = A.transposed();
	B = _B; Bt = B.transposed();
	V = _V;
	P = _P;
	VN = _VN;

	Ti = new Matrix[1]; Ti[0].resize(1,1); Ti[0].zero();
	Li = new Matrix[1]; Li[0].resize(1,1); Li[0].zero();

	n=A.rows();
	m=B.rows();
	N=-1;

	verbose=verb;
	if(verbose) cout<<"Riccati: problem defined, unsolved."<<endl;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::setVerbose(bool verb)
{
	verbose=verb;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix Riccati::L(int step)
{
	if(N<0) 
	{
		if(verbose) cout<<"Riccati: warning. DARE has not been solved yet."<<endl;
		return Li[0];
	}
	if(step>=0 && step>=N)
	{
		if(verbose) cout<<"Riccati: warning. Index for gain matrix out of bound."<<endl;
		return Li[0];
	}
	return Li[step];
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix Riccati::T(int step)
{
	if(N<0) 
	{
		if(verbose) cout<<"Riccati: error. DARE has not been solved yet."<<endl;
		return Ti[0];
	}
	if(step>=0 && step>N)
	{
		if(verbose) cout<<"Riccati: error. Index for DARE matrix out of bound."<<endl;
		return Ti[0];
	}
	return Ti[step];
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::setProblemData(const Matrix &_A, const Matrix &_B, const Matrix &_V,
                             const Matrix &_P, const Matrix &_VN)
{
	A = _A; At = A.transposed();
	B = _B; Bt = B.transposed();
	V = _V;
	P = _P;
	VN = _VN;

	n=A.rows();
	m=B.rows();
	N=-1;

	if(verbose) cout<<"Riccati: problem defined, unsolved."<<endl;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::solveRiccati(int steps)
{
	int i;
	N = steps;
	delete [] Ti;
	delete [] Li;
	Ti = new Matrix[steps+1];
	Li = new Matrix[steps];
	for(i=0; i<=steps; i++)
		Ti[i].resize(VN.rows(),VN.cols());		
	//init TN=VN
	Ti[steps]=VN;
	//compute backward all Ti
	for(i=steps-1; i>=0; i--)
	{
		lastT=Ti[i+1]; 
		//Ti = V + A' * (Ti+1 - Ti+1 * B * (P + B' * Ti+1 * B)^-1 * B' * Ti+1 )* A;
		Ti[i] = V + At *(lastT - lastT * B* pinv(P+Bt*lastT*B)*Bt*lastT )* A; 
	}
	//compute forward all Li
	for(i=0;i<steps; i++)
	{
		//Li = (P + B' * Ti+1 * B)^-1 * B' * Ti+1 * A
		Li[i] = pinv(P + Bt*Ti[i+1]*B) *Bt * Ti[i+1] * A;	
	}
	if(verbose) cout<<"Riccati: DARE solved, matrices Li and Ti computed and stored."<<endl;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector Riccati::doLQcontrol(int step, const Vector &x)
{
	if(N<0) 
	{
		if(verbose) cout<<"Riccati: error. DARE has not been solved yet."<<endl;
		Vector ret(1); ret.zero();
		return ret;
	}
	if(step>=0 && step>N)
	{
		if(verbose) cout<<"Riccati: error. Index for DARE matrix out of bound."<<endl;
		Vector ret(1); ret.zero();
		return ret;
	}
	return (Li[step] * (-1.0*x));
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::doLQcontrol(int step, const Vector &x, Vector &ret)
{
	if(N<0) 
	{
		if(verbose) cout<<"Riccati: error. DARE has not been solved yet."<<endl;
		ret.zero();
	}
	else if(step>=0 && step>N)
	{
		if(verbose) cout<<"Riccati: error. Index for DARE matrix out of bound."<<endl;
		ret.zero();
	}
	else
	{
		ret.resize(m);
		ret = Li[step] * (-1.0*x);
	}
}



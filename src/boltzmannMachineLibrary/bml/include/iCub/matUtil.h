// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _MATUTIL_H_
#define _MATUTIL_H_



#include <iCub/Connection.h>
#include <yarp/math/Rand.h>
#include <yarp/sig/all.h>
#include <yarp/os/Random.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <map>
#include <string>
#include <math.h>


using namespace std;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;


//--------- static functions ---------------
static double sum(const yarp::sig::Matrix &mat){
	double sum=0.0;
	for(int col=0;col<mat.cols();col++)
		for(int row=0;row< mat.rows();row++)
			sum+=mat(row,col);
	return sum;
}

static double sum(const yarp::sig::Vector &vec){
	double sum=0.0;
	for(int i=0;i<vec.length();i++)
			sum+=vec(i);
	return sum;
}

static Matrix repmat(Vector vec,int rows,int cols){
	//printf("The vector is %s", vec.toString().c_str());
	Matrix res(rows,vec.length());
	for(int row=0;row<rows;row++)
		for(int col=0;col<cols;col++)
			res(row,col)=vec(col);
	return res;
}

static Matrix exp(const Matrix &m){
	//printf("The matrix is %s", m.toString().c_str());
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=exp(m(row,col));
	return res;
}


static Matrix operator^(const yarp::sig::Matrix &m, double b){
	//printf("the matrix is multiplied by %f ", b);
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=pow(m(row,col),b);
	return res;
}

static Matrix operator+(const yarp::sig::Matrix &m, double c){
	//printf("the matrix is multiplied by %f ", c);
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)+c;
	return res;
}

static Vector operator+(const yarp::sig::Vector &m, double c){
	//printf("the matrix is multiplied by %f ", c);
	Vector res(m.length());
	for (int i=0;i<m.length();i++)
			res(i)=m(i)+c;
	return res;
}

static Matrix operator*(const yarp::sig::Matrix &m,double c){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)*c;
	return res;
}
static Matrix operator*(double c,const yarp::sig::Matrix &m){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)*c;
	return res;
}

static Matrix operator/(double c,const yarp::sig::Matrix &m){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=c/m(row,col);
	return res;
}
static Matrix operator/(const yarp::sig::Matrix &m,double c){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)/c;
	return res;
}

static Matrix operator>(const yarp::sig::Matrix &a,const yarp::sig::Matrix &b){
	Matrix res(a.rows(),a.cols());
	for (int row=0;row<a.rows();row++)
		for(int col=0;col<a.cols();col++)
			res(row,col)=a(row,col)>b(row,col);
	return res;
}

//--------- CLASS BMLMATRIX --------------

class bmlMatrix:public Matrix{
public:
	bmlMatrix();
	bmlMatrix(int r,int c);
	/*Matrix operator^(const yarp::sig::Matrix &m, double b);
	Matrix operator/(double c,yarp::sig::Matrix &m);
	Matrix operator*(yarp::sig::Matrix &m,double c);
	Matrix operator+(const yarp::sig::Matrix &m, double c);*/
};

//_--------- CLASS RANDOMMATRIX --------------

class randomMatrix:public Matrix{
public:
	randomMatrix();
	randomMatrix(int r,int c);	
};



#endif //_UNIT_H_
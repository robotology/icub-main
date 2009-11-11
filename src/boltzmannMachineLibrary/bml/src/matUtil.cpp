// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -
#include <iCub/matUtil.h>
#include <yarp/sig/all.h>


// --------- methods of randomMatrix ------------
randomMatrix::randomMatrix():Matrix(){
	double *p=this->data();
	for(int row=0;row< this->rows();row++){
		for(int col=0;col<this->cols();col++){
			*p=yarp::math::Rand().scalar();
			p++;
		}
	}
};

randomMatrix::randomMatrix(int r, int c):Matrix(r,c){
	double *p=this->data();
	for(int row=0;row< this->rows();row++){
		for(int col=0;col<this->cols();col++){
			*p=yarp::math::Rand().scalar();
			p++;
		}
	}
}


//--------- methods of bmlMatrix ---------

/*static bmlMatrix bmlMatrix::operator+(const yarp::sig::Matrix &m, double c){
	printf("the matrix is multiplied by %f ", c);
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)+c;
	return res;
}

static bmlMatrix bmlMatrix::operator*(yarp::sig::Matrix &m,double c){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=m(row,col)*c;
	return res;
}

static bmlMatrix bmlMatrix::operator/(double c,yarp::sig::Matrix &m){
	Matrix res(m.rows(),m.cols());
	for (int row=0;row<m.rows();row++)
		for(int col=0;col<m.cols();col++)
			res(row,col)=c/m(row,col);
	return res;
}

static bool bmlMatrix::operator^(const yarp::sig::Matrix &m, double b){
	printf("the matrix is multiplied by %f ", b);
	return true;
}*/
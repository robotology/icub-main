#ifndef YARPMATHLIBINTERFACE_H_
#define YARPMATHLIBINTERFACE_H_

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include "MathLib/MathLib.h"

using namespace MathLib;

Vector              YarpVectorToVector  (const yarp::sig::Vector &yvec);
Vector&             YarpVectorToVector  (const yarp::sig::Vector &yvec, Vector& result);
Matrix              YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat);
Matrix&             YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat, Matrix& result);

yarp::sig::Vector   VectorToYarpVector  (const Vector &vec);
yarp::sig::Vector&  VectorToYarpVector  (const Vector &vec, yarp::sig::Vector& result);
yarp::sig::Matrix   MatrixToYarpMatrix  (const Matrix &mat);
yarp::sig::Matrix&  MatrixToYarpMatrix  (const Matrix &mat, yarp::sig::Matrix& result);

bool                LoadYarpMatrix      (const char* filename, yarp::sig::Matrix &result);
bool                SaveYarpMatrix      (const char* filename, yarp::sig::Matrix &mat);


#endif /*YARPMATHLIBINTERFACE_H_*/

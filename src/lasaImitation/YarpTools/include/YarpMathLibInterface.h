#ifndef YARPMATHLIBINTERFACE_H_
#define YARPMATHLIBINTERFACE_H_

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include "MathLib/MathLib.h"

//using namespace MathLib;

MathLib::Vector     YarpVectorToVector  (const yarp::sig::Vector &yvec);
MathLib::Vector&    YarpVectorToVector  (const yarp::sig::Vector &yvec, MathLib::Vector& result);
MathLib::Matrix     YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat);
MathLib::Matrix&    YarpMatrixToMatrix  (const yarp::sig::Matrix &ymat, MathLib::Matrix& result);

yarp::sig::Vector   VectorToYarpVector  (const MathLib::Vector &vec);
yarp::sig::Vector&  VectorToYarpVector  (const MathLib::Vector &vec, yarp::sig::Vector& result);
yarp::sig::Matrix   MatrixToYarpMatrix  (const MathLib::Matrix &mat);
yarp::sig::Matrix&  MatrixToYarpMatrix  (const MathLib::Matrix &mat, yarp::sig::Matrix& result);

bool                LoadYarpMatrix      (const char* filename, yarp::sig::Matrix &result);
bool                SaveYarpMatrix      (const char* filename, yarp::sig::Matrix &mat);


#endif /*YARPMATHLIBINTERFACE_H_*/

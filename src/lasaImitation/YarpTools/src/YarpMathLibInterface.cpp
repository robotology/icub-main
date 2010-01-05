#include "YarpMathLibInterface.h"


MathLib::Matrix& YarpMatrixToMatrix(const yarp::sig::Matrix &ymat, MathLib::Matrix& result){
    int r = ymat.rows();
    int c = ymat.cols();
    result.Resize(r,c,false);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            result(i,j) = ymat(i,j);       
        }
    }
    return result;
}

MathLib::Matrix YarpMatrixToMatrix(const yarp::sig::Matrix &ymat){
    MathLib::Matrix mat;
    return YarpMatrixToMatrix(ymat,mat);
}

MathLib::Vector& YarpVectorToVector(const yarp::sig::Vector &yvec, MathLib::Vector& result){
    int r = yvec.size();
    result.Resize(r,false);
    for(int i=0;i<r;i++){
        result(i) = yvec(i);       
    }    
    return result;
}

MathLib::Vector YarpVectorToVector(const yarp::sig::Vector &yvec){
    MathLib::Vector vec;
    return YarpVectorToVector(yvec,vec);
}

yarp::sig::Matrix MatrixToYarpMatrix(const MathLib::Matrix &mat){
    yarp::sig::Matrix ymat;
    return MatrixToYarpMatrix(mat,ymat);
}

yarp::sig::Matrix& MatrixToYarpMatrix(const MathLib::Matrix &mat, yarp::sig::Matrix& result){
    int r = mat.RowSize();
    int c = mat.ColumnSize();
    result.resize(r,c);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            result(i,j) = mat.At(i,j);
        }
    }    
    return result;
}

yarp::sig::Vector VectorToYarpVector(const MathLib::Vector &vec){
    yarp::sig::Vector yvec;
    return VectorToYarpVector(vec,yvec);    
}
yarp::sig::Vector& VectorToYarpVector(const MathLib::Vector &vec, yarp::sig::Vector &result){
    int r = vec.Size();
    result.resize(r);
    for(int i=0;i<r;i++){
        result(i) = vec.At(i);       
    }    
    return result;        
}

bool LoadYarpMatrix(const char* filename, yarp::sig::Matrix &result){
    MathLib::Matrix tmp;
    bool res = tmp.Load(filename);
    result = MatrixToYarpMatrix(tmp);
    return res;
}
bool SaveYarpMatrix(const char* filename, yarp::sig::Matrix &mat){
    MathLib::Matrix tmp = YarpMatrixToMatrix(mat);
    bool res = tmp.Save(filename);
    return res;
}

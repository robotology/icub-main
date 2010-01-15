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


MathLib::Vector& YarpPose7ToPose6(yarp::sig::Vector &pose, MathLib::Vector &result){
    result.Resize(6);
    result(0) = pose(0);
    result(1) = pose(1);
    result(2) = pose(2);
    result(3) = pose(3)*pose(6);
    result(4) = pose(4)*pose(6);
    result(5) = pose(5)*pose(6);
}

void YarpPose7ToPose6(yarp::sig::Vector &pose, MathLib::Vector3 &pos, MathLib::Vector3 &ori){
    pos(0) = pose(0);
    pos(1) = pose(1);
    pos(2) = pose(2);
    ori(0) = pose(3)*pose(6);
    ori(1) = pose(4)*pose(6);
    ori(2) = pose(5)*pose(6);
}
void YarpPose6ToPose6(yarp::sig::Vector &pose, MathLib::Vector3 &pos, MathLib::Vector3 &ori){
    pos(0) = pose(0);
    pos(1) = pose(1);
    pos(2) = pose(2);
    ori(0) = pose(3);
    ori(1) = pose(4);
    ori(2) = pose(5);
}
void Pose6ToYarpPose6(MathLib::Vector3 &pos, MathLib::Vector3 &ori, yarp::sig::Vector &pose){
    pose(0) = pos(0);
    pose(1) = pos(1);
    pose(2) = pos(2);
    pose(3) = ori(0);
    pose(4) = ori(1);
    pose(5) = ori(2);
}
void AddPose6ToYarpPose6(MathLib::Vector3 &pos, MathLib::Vector3 &ori, yarp::sig::Vector &pose){
    pose(0) += pos(0);
    pose(1) += pos(1);
    pose(2) += pos(2);
    
    MathLib::Matrix3 srcM,oriM,res;
    MathLib::Vector3 srcV(pose(3),pose(4),pose(5));
    oriM.RotationV(ori);
    srcM.RotationV(srcV);
    oriM.Mult(srcM,res);
    MathLib::Vector3 resV;
    res.GetExactRotationAxis(resV);
    
    pose(3) = resV(0);
    pose(4) = resV(1);
    pose(5) = resV(2);
}
void YarpPose7ToYarpPose6(yarp::sig::Vector &pose, yarp::sig::Vector &result){
    result.resize(6);
    result(0) = pose(0);
    result(1) = pose(1);
    result(2) = pose(2);
    result(3) = pose(3)*pose(6);
    result(4) = pose(4)*pose(6);
    result(5) = pose(5)*pose(6);
}

MathLib::Matrix3&    YarpMatrix4ToMatrix3  (const yarp::sig::Matrix &ymat, MathLib::Matrix3& result){
    result.Zero();
    int r = MIN(ymat.rows(),3);
    int c = MIN(ymat.cols(),3);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            result(i,j) = ymat(i,j);       
        }
    }
    return result;    
}
MathLib::Matrix4&    YarpMatrix4ToMatrix4  (const yarp::sig::Matrix &ymat, MathLib::Matrix4& result){
    result.Zero();
    int r = MIN(ymat.rows(),4);
    int c = MIN(ymat.cols(),4);
    for(int i=0;i<r;i++){
        for(int j=0;j<c;j++){
            result(i,j) = ymat(i,j);       
        }
    }
    return result;    
}

MathLib::Vector3&   YarpVector3ToVector3  (const yarp::sig::Vector &vec, MathLib::Vector3& result){
    result.Zero();
    int r = MIN(3,vec.size());
    for(int i=0;i<r;i++){
        result(i) = vec(i);       
    }    
    return result;
}
yarp::sig::Vector&  Vector3ToYarpVector3  (const MathLib::Vector3 &vec, yarp::sig::Vector &result){
    result.resize(3);
    result(0) = vec.cx();       
    result(1) = vec.cy();       
    result(2) = vec.cz();       
    return result;       
}

MathLib::Vector &   AddPose6ToPose6(MathLib::Vector &pose, const MathLib::Vector &offset){
    pose(0) += offset.At(0);
    pose(1) += offset.At(1);
    pose(2) += offset.At(2);
    
    MathLib::Matrix3 srcM,oriM,res;
    MathLib::Vector3 srcV(pose(3),pose(4),pose(5));
    MathLib::Vector3 oriV(offset.At(3),offset.At(4),offset.At(5));
    oriM.RotationV(oriV);
    srcM.RotationV(srcV);
    oriM.Mult(srcM,res);
    MathLib::Vector3 resV;
    res.GetExactRotationAxis(resV);
    
    pose(3) = resV(0);
    pose(4) = resV(1);
    pose(5) = resV(2);
    return pose;
}
MathLib::Vector &   GetDeltaPose6FromPose6(const MathLib::Vector &src, const MathLib::Vector &target, MathLib::Vector &delta){
    delta.Resize(6);
    delta(0) = target.At(0) - src.At(0);    
    delta(1) = target.At(1) - src.At(1);    
    delta(2) = target.At(2) - src.At(2);    

    MathLib::Matrix3 srcM,trgM,res;
    MathLib::Vector3 srcV(src.At(3),src.At(4),src.At(5));
    MathLib::Vector3 trgV(target.At(3),target.At(4),target.At(5));
    trgM.RotationV(trgV);
    srcM.RotationV(srcV);
    res = trgM*srcM.Transpose();
    MathLib::Vector3 resV;
    res.GetExactRotationAxis(resV);
    
    delta(3) = resV(0);
    delta(4) = resV(1);
    delta(5) = resV(2);
    return delta;
}


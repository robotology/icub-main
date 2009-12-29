#ifndef SPATIAL_INERTIA_H
#define SPATIAL_INERTIA_H

#include "Matrix3.h"
#include "SpatialVelocity.h"
#include "SpatialForce.h"

/**
 * \class SpatialInertia
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TMatrix class
 * 
 * This template square matrix class can be used for doing various matrix manipulation.
 * This should be combined with the TVector class for doing almost anything
 * you ever dreamt of. 
 */

class SpatialInertia
{
public:
  REALTYPE    mMass;
  Vector3     mLinearMoment;
  Vector3     mCenterOfMass;
  Matrix3     mInertiaMoment;
      
public:
  /// Empty constructor
  inline SpatialInertia(){};
  /// Copy constructor
  inline SpatialInertia(const SpatialInertia &matrix){
    Set(matrix);
  };  
  /// Copy Constructor 
  inline SpatialInertia(const REALTYPE mass, const Vector3& linearMoment, const Matrix3& inertiaMoment){
    Set(mass,linearMoment,inertiaMoment);
  };

  /// Destructor
  inline virtual ~SpatialInertia(){}

  inline SpatialInertia& Zero(){
    mMass = R_ZERO;
    mLinearMoment.Zero();
    mInertiaMoment.Zero();
    return *this;      
  } 

  inline SpatialInertia& Set(const SpatialInertia& inertia){
    mMass = inertia.mMass;
    mLinearMoment.Set(inertia.mLinearMoment);
    mInertiaMoment.Set(inertia.mInertiaMoment);
    return *this;  
  }

  inline SpatialInertia& Set(const REALTYPE mass, const Vector3& linearMoment, const Matrix3& inertiaMoment){
    mMass = mass;
    mLinearMoment.Set(linearMoment);
    mInertiaMoment.Set(inertiaMoment);
    return *this;
  }
  


  inline SpatialForce operator * (const SpatialVelocity &vector) const
  {
    SpatialForce result;
    return Mult(vector,result);
  }

  inline SpatialForce Mult(const SpatialVelocity &vector) const
  {
    SpatialForce result;
    return Mult(vector,result);
  }

  inline SpatialForce& Mult(const SpatialVelocity &vector, SpatialForce& result) const
  {
    Vector3 tmp;
    mInertiaMoment.Mult(vector.mAngular,result.mAngular);
    mLinearMoment.Cross(vector.mLinear,tmp);
    result.mAngular.SAdd(tmp);
    
    result.mLinear = vector.mLinear;
    result.mLinear*= mMass;
    vector.mAngular.Cross(mLinearMoment,tmp);
    result.mLinear.SAdd(tmp);
    return result;
  }

  
  inline SpatialInertia operator + (const SpatialInertia &inertia) const
  {
    SpatialInertia result;
    return Add(inertia,result);
  }

  inline SpatialInertia Add(const SpatialInertia &inertia) const
  {
    SpatialInertia result;
    return Add(inertia,result);
  }

  inline SpatialInertia& Add(const SpatialInertia &inertia, SpatialInertia & result) const
  {
    result.mMass = mMass + inertia.mMass;
    mLinearMoment.Add(inertia.mLinearMoment,result.mLinearMoment);
    mInertiaMoment.Add(inertia.mInertiaMoment,result.mInertiaMoment);
    return result;
  }
  
  inline SpatialVector GetColumn(unsigned int i){
    SpatialVector result;
    GetColumn(i,result);
    return result;
  }
  
  inline SpatialVector& GetColumn(unsigned int i, SpatialVector & result){
    if(i<3){
      mInertiaMoment.GetColumn(i,result.mAngular);
      Matrix3 tmp; tmp.SkewSymmetric(mLinearMoment);
      tmp.STranspose();      
      tmp.GetColumn(i,result.mLinear);
    }else if(i<6){
      Matrix3 tmp; tmp.SkewSymmetric(mLinearMoment);
      tmp.GetColumn(i-3,result.mAngular);
      result.mLinear.Zero();
      result.mLinear[i-3] = mMass;
    }
    return result;
  }
  
  inline TMatrix<6>& ToTMatrix6(TMatrix<6> & result){
    Matrix3 tmpM; 
    tmpM.SkewSymmetric(mCenterOfMass);
    Matrix3 tmpM2;
    tmpM.Transpose(tmpM2);
    Matrix3 tmpM3;
    tmpM.Mult(tmpM2,tmpM3);
    
    //rbi = [ I + m*C*C', m*C; m*C', m*eye(3) ];    
    
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        result._[i  ][j]   = mInertiaMoment._[i][j] + tmpM3._[i][j] *mMass;
        result._[i+3][j]   = tmpM.          _[j][i] *mMass;
        result._[i  ][j+3] = tmpM.          _[i][j] *mMass;
        result._[i+3][j+3] = R_ZERO;
      }
      result._[i+3][i+3] = mMass;
    }
    return result;
  }
  
  /*
  inline TMatrix<6>& ToTMatrix6(TMatrix<6> & result){
    Matrix3 tmpM; tmpM.SkewSymmetric(mLinearMoment);
    
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        result._[i  ][j]   = mInertiaMoment._[i][j];
        result._[i+3][j]   = tmpM.          _[j][i];
        result._[i  ][j+3] = tmpM.          _[i][j];
        result._[i+3][j+3] = R_ZERO;
      }
      result._[i+3][i+3] = mMass;
    }
    return result;
  }
  */

};


#endif

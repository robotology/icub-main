#ifndef __ReferenceFrame_H__
#define __ReferenceFrame_H__

#include "MathLibCommon.h"

#include "Vector3.h"
#include "Matrix3.h"
#include "Matrix4.h"

#define REFFRAME_NONE       0
#define REFFRAME_BASEREF   -1
#define REFFRAME_HMATRIX   +1

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib{
#endif

class ReferenceFrame;
typedef ReferenceFrame *pReferenceFrame;

/**
 * \class ReferenceFrame
 * 
 * \ingroup MathLib
 * 
 * \brief A wrapper class to consider frame of references
 * 
 */


class ReferenceFrame
{ 
protected:

  Vector3           mOrigin;
  Matrix3           mOrient;
  Matrix4           mHMatrix;

  ReferenceFrame   *mInverse;

  int               mInvalidRepresentation;
  bool              bInverseChanged;

public:

  inline ReferenceFrame(){
    mInverse = new ReferenceFrame(this);    
    Init();
  }
  
  inline ReferenceFrame(const ReferenceFrame & referenceFrame){
    ReferenceFrame();
    Set(referenceFrame);
  }
  
  inline ReferenceFrame(const Vector3 &origin, const Matrix3 &orient){
    ReferenceFrame();
    Set(origin,orient);
  }
  
  inline ReferenceFrame(const Matrix4 & hMatrix){
    ReferenceFrame();
    SetHMatrix(hMatrix);
  }

  inline ~ReferenceFrame(){
    if(mInverse!=NULL){
      mInverse->mInverse = NULL;
      delete mInverse;
      mInverse = NULL;
    }
  }

  inline ReferenceFrame& Set(const ReferenceFrame& referenceFrame){    
    Set(referenceFrame.mOrigin,referenceFrame.mOrient);
    return *this;
  }

  inline ReferenceFrame& Set(const Vector3& origin, const Matrix3& orient){
    SetOrigin(origin);
    SetOrient(orient);
    return *this;
  }
  
  inline ReferenceFrame& Identity(){
    SetOrigin(Vector3::ZERO);
    SetOrient(Matrix3::IDENTITY);
    return *this;
  }




  inline const Vector3& GetOrigin(){
    if(bInverseChanged) UpdateFromInverse();
    else if (mInvalidRepresentation == REFFRAME_BASEREF)  UpdateBaseRef();
    return mOrigin;
  }
  inline const Matrix3& GetOrient(){
    if(bInverseChanged) UpdateFromInverse();
    else if (mInvalidRepresentation == REFFRAME_BASEREF)  UpdateBaseRef();
    return mOrient;
  }
  inline const Matrix4& GetHMatrix(){
    if(bInverseChanged) UpdateFromInverse();
    else if (mInvalidRepresentation == REFFRAME_HMATRIX)  UpdateHMatrix();
    return mHMatrix;  
  }


  
  inline Vector3& SetOrigin(){
    mInvalidRepresentation    = REFFRAME_HMATRIX;
    mInverse->bInverseChanged = true;
    return mOrigin;
  }
  inline Matrix3& SetOrient(){
    mInvalidRepresentation    = REFFRAME_HMATRIX;
    mInverse->bInverseChanged = true;
    return mOrient;
  }
  inline Matrix4& SetHMatrix(){
    mInvalidRepresentation    = REFFRAME_BASEREF;
    mInverse->bInverseChanged = true;
    return mHMatrix;  
  }

  inline ReferenceFrame& SetOrigin(const Vector3& origin){
    SetOrigin().Set(origin);
    return *this;
  }
  inline ReferenceFrame& SetOrient(const Matrix3& orient){
    SetOrient().Set(orient);
    return *this;
  }
  inline ReferenceFrame& SetHMatrix(const Matrix4& hMatrix){
    SetHMatrix().Set(hMatrix);
    return *this;
  }


  inline Vector3& GetSetOrigin(){
    GetOrigin();
    return SetOrigin();
  }
  inline Matrix3& GetSetOrient(){
    GetOrient();
    return SetOrient();
  }
  inline Matrix4& GetSetHMatrix(){
    GetHMatrix();
    return SetHMatrix();  
  }
  
  inline const Vector3& IGetOrigin(){
    return mOrigin;
  }
  inline const Matrix3& IGetOrient(){
    return mOrient;
  }
  inline const Matrix4& IGetHMatrix(){
    return mHMatrix;  
  }
  


  inline ReferenceFrame& GetInverse(){
    return *mInverse;  
  }

  
  inline ReferenceFrame Mult(ReferenceFrame& referenceFrame){
    ReferenceFrame result;
    Mult(referenceFrame,result);
    return result;
  }
  
  inline ReferenceFrame& Mult(ReferenceFrame& referenceFrame, ReferenceFrame& result){
    if(bInverseChanged) UpdateFromInverse();
    else if(mInvalidRepresentation == REFFRAME_BASEREF)  UpdateBaseRef();

    mOrient.Mult(referenceFrame.GetOrient(),result.mOrient);
    mOrient.Mult(referenceFrame.GetOrigin(),result.mOrigin);
    result.SetOrigin().SAdd(mOrigin);
    return *this;
  }
  

  



  inline void  Update(){

    if(bInverseChanged){
      UpdateFromInverse();
    }else{      
      if(!mInvalidRepresentation){
        if(mInvalidRepresentation==REFFRAME_BASEREF){
          UpdateBaseRef();
        }else{              
          UpdateHMatrix();
        }
      }
    }
    mInverse->Update();
  }
  

protected:
  
  inline void  UpdateBaseRef(){
    mHMatrix.GetTranslation(mOrigin);
    mHMatrix.GetOrientation(mOrient);
    
    mInvalidRepresentation  = REFFRAME_NONE;
  }
  inline void  UpdateHMatrix(){
    mHMatrix.Transformation(mOrient,mOrigin);    
    
    mInvalidRepresentation  = REFFRAME_NONE;
  }

  inline void UpdateFromInverse(){
    if(mInverse->mInvalidRepresentation == REFFRAME_BASEREF)
      mInverse->UpdateBaseRef();
      
    mInverse->mOrient.Transpose(mOrient);
    mOrient.Mult(mInverse->mOrigin,mOrigin);
    mOrigin.SMinus();
    mHMatrix.Transformation(mOrient,mOrigin);
    
    mInvalidRepresentation  = REFFRAME_NONE;
    bInverseChanged         = false;
  }


  
 
private:

  inline ReferenceFrame(ReferenceFrame *inverse){
    mInverse = inverse; 
    Init();
  }
  
  inline void  Init(){
    Identity();  
    bInverseChanged         = false;
  }

};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif

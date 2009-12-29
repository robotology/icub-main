#ifndef SPATIAL_FRAME_H
#define SPATIAL_FRAME_H

#include "Vector3.h"
#include "Matrix3.h"
#include "SpatialVector.h"
#include "SpatialVelocity.h"
#include "SpatialForce.h"
#include "SpatialInertia.h"
#include "Referential.h"

#include "ReferenceFrame.h"



/**
 * \class SpatialFrame
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TMatrix class
 * 
 * This template square matrix class can be used for doing various matrix manipulation.
 * This should be combined with the TVector class for doing almost anything
 * you ever dreamt of. 
 */

class SpatialFrame
{
public:

  ReferenceFrame *mRefFrame;
  ReferenceFrame *mRefFramePtr;

  
public:
  /// Empty constructor
  inline SpatialFrame(){
    mRefFrame    = new ReferenceFrame();
    mRefFramePtr = mRefFrame; 
  };
  /// Copy constructor
  inline SpatialFrame(const SpatialFrame &spatialFrame){
    SpatialFrame();
    Set(spatialFrame);
  };  
  inline SpatialFrame(ReferenceFrame &refFrame){
    SpatialFrame();
    Set(refFrame);
  };  

  /// Destructor
  inline virtual ~SpatialFrame(){
    if(mRefFrame!=NULL) delete mRefFrame; 
    mRefFrame    = NULL;
    mRefFramePtr = NULL;    
  }

 
  inline SpatialFrame& Set(const SpatialFrame& spatialFrame){
    mRefFrame->Set(*(spatialFrame.mRefFramePtr));
    mRefFramePtr = mRefFrame; 
    return *this;  
  }

  inline SpatialFrame& Set(ReferenceFrame &refFrame){
    mRefFramePtr = &refFrame; 
    return *this;
  };  

  inline ReferenceFrame& GetReferenceFrame(){
    return *mRefFramePtr;
  }
  

  inline SpatialFrame& Identity(){
    mRefFramePtr->Identity();
    return *this;
  }


  /// Assignment operator
  inline SpatialFrame& operator = (const SpatialFrame &frame){
    return Set(frame);    
  }    
  
  /// Assignment data-wise operations
  inline SpatialFrame& operator *= (const SpatialFrame &frame){
    return SMult(frame);
  }

  inline SpatialFrame& SMult(const SpatialFrame &frame){
    SpatialFrame copy(*this);
    copy.Mult(frame,*this);
    return *this;
  }

  inline SpatialFrame operator * (const SpatialFrame &frame) const
  {
    SpatialFrame result;
    return Mult(frame,result);
  }

  inline SpatialFrame Mult(const SpatialFrame &frame) const
  {
    SpatialFrame result;
    return Mult(frame,result);
  }


  inline SpatialFrame& Mult(const SpatialFrame &frame, SpatialFrame &result) const
  {
    mRefFramePtr->Mult(*frame.mRefFramePtr,*result.mRefFramePtr);
    return result;
  }


  inline SpatialFrame Inverse() const
  {
    SpatialFrame result;
    return Inverse(result);
  }

  inline SpatialFrame& Inverse(SpatialFrame& result) const
  {
    result.Set(mRefFramePtr->GetInverse());
    return result;
  }


  
  
  inline SpatialVelocity operator * (const SpatialVelocity &vector) const
  {
    SpatialVelocity result;
    return Mult(vector,result);
  }

  inline SpatialVelocity Mult(const SpatialVelocity &vector) const
  {
    SpatialVelocity result;
    return Mult(vector,result);
  }

  inline SpatialVelocity& Mult(const SpatialVelocity &vector, SpatialVelocity &result) const
  {    
    vector.mAngular.Cross(mRefFramePtr->GetOrigin(),result.mAngular); // -(p x w)
    result.mAngular.SAdd(vector.mLinear);        // (v0 - (p x w))
    mRefFramePtr->GetOrient().Mult(result.mAngular,result.mLinear);
    
    mRefFramePtr->GetOrient().Mult(vector.mAngular,result.mAngular);
    return result;
  }




  inline SpatialForce operator * (const SpatialForce &vector) const
  {
    SpatialForce result;
    return Mult(vector,result);
  }

  inline SpatialForce Mult(const SpatialForce &vector) const
  {
    SpatialForce result;
    return Mult(vector,result);
  }

  inline SpatialForce& Mult(const SpatialForce &vector, SpatialForce &result) const
  {
    vector.mLinear.Cross(mRefFramePtr->GetOrigin(),result.mLinear);   // -(p x f)
    result.mLinear.SAdd(vector.mAngular);        // (v0 - (p x w))
    mRefFramePtr->GetOrient().Mult(result.mLinear,result.mAngular);
    
    mRefFramePtr->GetOrient().Mult(vector.mLinear,result.mLinear);
    return result;
  }



  inline SpatialInertia operator * (const SpatialInertia &inertia) const
  {
    SpatialInertia result;
    return Mult(inertia,result);
  }

  inline SpatialInertia Mult(const SpatialInertia &inertia) const
  {
    SpatialInertia result;
    return Mult(inertia,result);
  }

  inline SpatialInertia& Mult(const SpatialInertia &inertia, SpatialInertia &result) const
  {
    /*
    vector.mLinear.Cross(mRefFramePtr->GetOrigin(),result.mLinear);   // -(p x f)
    result.mLinear.SAdd(vector.mAngular);        // (v0 - (p x w))
    mRefFramePtr->GetOrient().Mult(result.mLinear,result.mAngular);
    
    mRefFramePtr->GetOrient().Mult(vector.mLinear,result.mLinear);
    return result;
    */
    result.mMass = inertia.mMass;
    
    mRefFramePtr->GetOrient().MultTranspose(inertia.mLinearMoment,result.mLinearMoment);

    Matrix3 sPos; sPos.SkewSymmetric(mRefFramePtr->GetOrigin());
    Matrix3 sMom; sMom.SkewSymmetric(result.mLinearMoment);

    Vector3 tmp;
    tmp = mRefFramePtr->GetOrigin();
    tmp *= inertia.mMass;
    result.mLinearMoment.SAdd(tmp);

    Matrix3 sMom2; sMom2.SkewSymmetric(result.mLinearMoment);
    
    Matrix3 tmpMat;
    mRefFramePtr->GetInverse().GetOrient().Mult(inertia.mInertiaMoment,tmpMat);
    tmpMat.Mult(mRefFramePtr->GetOrient(),result.mInertiaMoment);

    sPos.Mult(sMom,tmpMat);
    result.mInertiaMoment.SSub(tmpMat);

    sMom2.Mult(sPos,tmpMat);
    result.mInertiaMoment.SSub(tmpMat);
    
    return result;
  }

  inline TMatrix<6>& ToTMatrix6(TMatrix<6> & result){
    Matrix3 tmpM;  tmpM.Set(mRefFramePtr->GetOrient());
    Matrix3 tmpM2; tmpM2.SkewSymmetric(mRefFramePtr->GetOrigin()); tmpM2.STranspose();
    Matrix3 tmpM3; tmpM.Mult(tmpM2,tmpM3);
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        result._[i  ][j]   = tmpM.          _[i][j];
        result._[i+3][j]   = tmpM3.         _[i][j];
        result._[i  ][j+3] = R_ZERO;
        result._[i+3][j+3] = tmpM.          _[i][j];
      }
    }
    return result;
  }






      

};


#endif

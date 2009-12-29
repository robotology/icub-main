#include "Referential.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif


Referential::Referential(){
  Identity();
}

Referential::Referential(const Referential & ref){
  Set(ref);
}
Referential::Referential(const Vector3 &vec, const Matrix3 &mat){
  Set(vec,mat);
}

Referential::Referential(const Matrix4 & mat){
  Set(mat);
}

Referential::~Referential(){}


Referential& Referential::Set(const Referential & ref){
  mOrigin.Set(ref.mOrigin);
  mOrient.Set(ref.mOrient);
  Update();
  return *this;  
}

Referential& Referential::Set(const Vector3 &vec, const Matrix3 &mat){
  mOrigin.Set(vec);
  mOrient.Set(mat);
  Update();    
  return *this;  
}

Referential& Referential::Set(const Matrix4 &mat){
  mOrigin.Set(Vector3(mat._[0][3],mat._[1][3],mat._[2][3]));
  mOrient.Set(Matrix3(mat._[0][0],mat._[0][1],mat._[0][2],
                      mat._[1][0],mat._[1][1],mat._[1][2],
                      mat._[2][0],mat._[2][1],mat._[2][2]));
  mOrient.Normalize();
  Update();            
  return *this;  
}

Referential& Referential::Identity(){
  mOrigin.Zero();
  mOrient.Identity();  
  Update();
  return *this;  
}

Referential& Referential::SetOrigin(const Vector3 &vec){
  bIsValid = 1;
  mOrigin.Set(vec);  
  return *this;  
}
Referential& Referential::SetOrient(const Matrix3 &mat){
  bIsValid = 1;
  mOrient.Set(mat);
  return *this;  
}
Referential& Referential::SetInvOrigin(const Vector3 &vec){
  bIsValid = -1;
  mInvOrigin.Set(vec);  
  return *this;  
}
Referential& Referential::SetInvOrient(const Matrix3 &mat){
  bIsValid = -1;
  mInvOrient.Set(mat);
  return *this;  
}


const Vector3& Referential::GetOrigin() const {
  return mOrigin;
}
const Matrix3& Referential::GetOrient() const {
  return mOrient;
}
const Vector3& Referential::GetInvOrigin() const {
  return mInvOrigin;
}
const Matrix3& Referential::GetInvOrient() const {
  return mInvOrient;
}

Vector3& Referential::GetSetOrigin(){
  bIsValid = 1;
  return mOrigin;
}
Matrix3& Referential::GetSetOrient(){
  bIsValid = 1;
  return mOrient;
}
Vector3& Referential::GetSetInvOrigin(){
  bIsValid = -1;
  return mInvOrigin;
}
Matrix3& Referential::GetSetInvOrient(){
  bIsValid = -1;
  return mInvOrient;
}


const Matrix4& Referential::GetMatrix(){
  if(bIsValid) Update(bIsValid==1);
  return mOutput;  
}

const Matrix4& Referential::GetInverse(){
  if(bIsValid) Update(bIsValid==1);
  return mInverse;  
}


Referential  Referential::Mult(const Referential & ref){
  Referential result;
  Mult(ref,result);
  return result;
}

Referential& Referential::Mult(const Referential & ref, Referential & result){
  mOrient.Mult(ref.mOrient,result.mOrient);
  mOrient.Mult(ref.mOrigin,result.mOrigin);
  result.mOrigin.SAdd(mOrigin);
  result.Update();
  return *this;
}


void Referential::Update(bool fwd)
{
  if(fwd){
    mOutput.Transformation(mOrient,mOrigin);
    
    mOrient.Transpose(mInvOrient);
    mInvOrient.Mult(mOrigin,mInvOrigin);
    mInvOrigin.SMinus();    
    mInverse.Transformation(mInvOrient,mInvOrigin);
  }else{
    mInverse.Transformation(mInvOrient,mInvOrigin);
    
    mInvOrient.Transpose(mOrient);
    mOrient.Mult(mInvOrigin,mOrigin);
    mOrigin.SMinus();    
    mOutput.Transformation(mOrient,mOrigin);
  }
  bIsValid = 0;
}


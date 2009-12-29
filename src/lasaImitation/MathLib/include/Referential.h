#ifndef __REFERENTIAL_H__
#define __REFERENTIAL_H__

#include "MathLibCommon.h"

#include "Vector3.h"
#include "Matrix3.h"
#include "Matrix4.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib{
#endif

class Referential;
typedef Referential *pReferential, Ref, *pRef;

/**
 * \class Referential
 * 
 * \ingroup MathLib
 * 
 * \brief A wrapper class to consider frame of references
 * 
 */

class Referential
{
protected:
  int           bIsValid;

  Vector3       mOrigin;
  Matrix3       mOrient;

  Vector3       mInvOrigin;
  Matrix3       mInvOrient;


  Matrix4       mOutput;
  Matrix4       mInverse;

public:
  Referential();
  Referential(const Referential & ref);
  Referential(const Vector3 &vec, const Matrix3 &mat);
  Referential(const Matrix4 & mat);
  ~Referential();


  Referential&      Set(const Referential & ref);
  Referential&      Set(const Vector3 &vec, const Matrix3 &mat);
  Referential&      Set(const Matrix4 &mat);

  Referential&      Identity();
  
  Referential&      SetOrigin(const Vector3 &vec);
  Referential&      SetOrient(const Matrix3 &mat);
  Referential&      SetInvOrigin(const Vector3 &vec);
  Referential&      SetInvOrient(const Matrix3 &mat);
  
  const Vector3&    GetOrigin() const;
  const Matrix3&    GetOrient() const;
  const Vector3&    GetInvOrigin() const;
  const Matrix3&    GetInvOrient() const;

  
  Vector3&          GetSetOrigin();
  Matrix3&          GetSetOrient();
  Vector3&          GetSetInvOrigin();
  Matrix3&          GetSetInvOrient();

  const Matrix4&    GetMatrix();
  const Matrix4&    GetInverse();

  Referential       Mult(const Referential & ref);
  Referential&      Mult(const Referential & ref, Referential & result);

  void              Update(bool fwd=true);
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif

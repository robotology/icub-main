#include "Vector.h"
#include "Matrix.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

REALTYPE Vector::undef = R_ZERO;


Matrix& Vector::MultTranspose(const Vector & vec, Matrix& result){
  const int r = MIN(row,vec.row);
  result.Resize(r,r);
  for(int i=0;i<r;i++){
    for(int j=0;j<r;j++){
      result._[i*r+j] = _[i]*vec._[j];
    }
  }
  return result;
}

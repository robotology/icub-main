#ifndef SPATIAL_FORCE_H
#define SPATIAL_FORCE_H

#include "SpatialVector.h"


/**
 * \class SpatialForce
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TVector class for Spatial Vector notation
 */
class SpatialForce : public SpatialVector
{
  
public:
  /// Empty constructor
  inline SpatialForce():SpatialVector(){};
  /// Copy constructor
  inline SpatialForce(const SpatialForce &vector):SpatialVector(vector){};
  /// Copy constructor
  inline SpatialForce(const SpatialVector &vector):SpatialVector(vector){};
  /// Data-based constructor
  inline SpatialForce(REALTYPE wx, REALTYPE wy, REALTYPE wz, REALTYPE x, REALTYPE y, REALTYPE z):SpatialVector(wx,wy,wz,x,y,z){}
  /// Vector component-based constructor
  inline SpatialForce(const Vector3& angular, const Vector3& linear):SpatialVector(angular,linear){} 
  /// Destructor
  inline ~SpatialForce(){};




};


#endif

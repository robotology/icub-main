#ifndef SPATIAL_VELOCITY_H
#define SPATIAL_VELOCITY_H

#include "SpatialVector.h"
#include "SpatialForce.h"

/**
 * \class SpatialVelocity
 * 
 * \ingroup MathLib
 * 
 * \brief An implementation of the template TVector class for Spatial Vector notation
 */
class SpatialVelocity : public SpatialVector
{
  
public:
  /// Empty constructor
  inline SpatialVelocity():SpatialVector(){};
  /// Copy constructor
  inline SpatialVelocity(const SpatialVelocity &vector):SpatialVector(vector){};
  /// Data-based constructor
  inline SpatialVelocity(REALTYPE wx, REALTYPE wy, REALTYPE wz, REALTYPE x, REALTYPE y, REALTYPE z):SpatialVector(wx,wy,wz,x,y,z){}
  /// Vector component-based constructor
  inline SpatialVelocity(const Vector3& angular, const Vector3& linear):SpatialVector(angular,linear){} 
  /// Destructor
  inline ~SpatialVelocity(){};
 
  inline SpatialVelocity Cross(SpatialVelocity& motion){
    SpatialVelocity result;
    Cross(motion,result);
    return result;
  }
  inline SpatialVelocity& Cross(SpatialVelocity& motion, SpatialVelocity& result){
    mLinear.Cross(motion.mAngular,result.mAngular);
    mAngular.Cross(motion.mLinear,result.mLinear);
    result.mLinear.SAdd(result.mAngular);
    mAngular.Cross(motion.mAngular,result.mAngular);
    return result;
  }

  inline SpatialForce Cross(SpatialForce& force){
    SpatialForce result;
    Cross(force,result);
    return result;
  }
  inline SpatialForce& Cross(SpatialForce& force, SpatialForce& result){
    mAngular.Cross(force.mAngular,result.mAngular);
    mLinear.Cross(force.mLinear,result.mLinear);    
    result.mAngular.SAdd(result.mLinear);
    
    mAngular.Cross(force.mLinear,result.mLinear);    
    return result;
  }

};


#endif

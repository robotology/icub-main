#ifndef PATCH_H
#define PATCH_H

#include "Thing.h"
#include "GlobalStuff.h"

#define CIRCLE_STEP 20
#define EPSILON 0.1

#define d0 1
#define kp 5
#define kn 4
#define POSITIVE_GRADIENT_EXPRESSION (1/distance - 1/d0)*(1 / pow(distance,2)) // the gradient at point P is inversely proportional to distance^kp.
#define NEGATIVE_GRADIENT_EXPRESSION (1 / pow(distance, kn)) // the gradient at point P is inversely proportional to distance^kn  (kn <= 1).

class Potential :
    public Thing
{
public:
protected:
    double radius;
    double potential;
public:
    Potential(double myX, double myY, double myRadius, double potential);
    virtual ~Potential(void);
    void TransformToWindowCoordinates(int originX, int originY, double scalingFactor);
    virtual void Draw(Cairo::RefPtr<Cairo::Context> myCr) const;
    double GetPotential(void) const;

};

#endif //PATCH_H

#ifndef THING_H
#define THING_H

#include <yarp/sig/Vector.h>
using namespace yarp::sig;

#include <cairomm/context.h>

class Thing
{
protected:
    Vector position;
	double orientationAngle;
    double scalingFactor;

public:
    Thing(void);
    Thing(double myX, double myY, double myOrientationAngle = 0);
    ~Thing(void);
    virtual void Draw(Cairo::RefPtr<Cairo::Context> myCr) const = 0;
    virtual void TransformToWindowCoordinates(int originX, int originY, double scalingFactor);
	virtual void TransformContext(Cairo::RefPtr<Cairo::Context> myCr) const;
	virtual void RestoreContext(Cairo::RefPtr<Cairo::Context> myCr) const;
    const Vector &GetPosition(void) const;
    void SetPosition(const Vector &newPosition);
    virtual void Scale(double myScalingFactor);
	void Rotate(double angle);
	void Translate(const Vector &vector);

};

#endif //THING_H
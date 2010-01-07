#pragma once
#include <cairomm/context.h>
#include <glm/glm.hpp>
#include <glm/gtc/double_float.hpp>
using namespace glm;


class Thing
{
protected:
    dvec2 position;
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
    const dvec2 &GetPosition(void) const;
    void SetPosition(const dvec2 &newPosition);
    virtual void Scale(double myScalingFactor);
	void Rotate(double angle);
	void Translate(const dvec2 &vector);

};

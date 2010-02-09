#include "Thing.h"

Thing::Thing(void)
{
}

Thing::Thing(double myX, double myY, double myOrientationAngle) :
	orientationAngle(myOrientationAngle), scalingFactor(1), position(Vector(2))
{
	position[0] = myX;
	position[1] = myY;
}

Thing::~Thing(void)
{
}

void Thing::TransformToWindowCoordinates(int originX, int originY, double scalingFactor)
{
    position[0] = originX + (position[0] * scalingFactor);
    position[1] = originY - (position[1] * scalingFactor);
}


void Thing::TransformContext(Cairo::RefPtr<Cairo::Context> myCr) const
{
    myCr->scale(scalingFactor, scalingFactor);
	myCr->translate(position[0], position[1]);
	myCr->rotate_degrees(-orientationAngle);
}

void Thing::RestoreContext(Cairo::RefPtr<Cairo::Context> myCr) const
{
	myCr->rotate_degrees(-orientationAngle);
	myCr->translate(-position[0], -position[1]);
    myCr->scale(1/scalingFactor, 1/scalingFactor);
}
    
const Vector &Thing::GetPosition(void) const
{
    return position;
}


void Thing::SetPosition(const Vector &newPosition)
{
    position = newPosition;
}

void Thing::Scale(double myScalingFactor)
{
    scalingFactor = myScalingFactor;
}

void Thing::Rotate(double angle)
{
	orientationAngle += angle;
	if(orientationAngle > 360 )
	{
		orientationAngle -= 360;
	}
	else if (orientationAngle < 0)
	{
		orientationAngle += 360;
	}
}

void Thing::Translate(const Vector &vector)
{
	position[0] += vector[0];
	position[1] += vector[1];
}

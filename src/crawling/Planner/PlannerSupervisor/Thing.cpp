#include "Thing.h"

Thing::Thing(void)
{
}

Thing::Thing(double myX, double myY, double myOrientationAngle) :
	orientationAngle(myOrientationAngle), scalingFactor(1), position(myX, myY)
{
}

Thing::~Thing(void)
{
}

void Thing::TransformToWindowCoordinates(int originX, int originY, double scalingFactor)
{
    position.x = originX + (position.x*scalingFactor);
    position.y = originY - (position.y*scalingFactor);
}


void Thing::TransformContext(Cairo::RefPtr<Cairo::Context> myCr) const
{
    myCr->scale(scalingFactor, scalingFactor);
	myCr->translate(position.x, position.y);
	myCr->rotate_degrees(-orientationAngle);
}

void Thing::RestoreContext(Cairo::RefPtr<Cairo::Context> myCr) const
{
	myCr->rotate_degrees(-orientationAngle);
	myCr->translate(-position.x, -position.y);
    myCr->scale(1/scalingFactor, 1/scalingFactor);
}
    
const dvec2 &Thing::GetPosition(void) const
{
    return position;
}


void Thing::SetPosition(const dvec2 &newPosition)
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

void Thing::Translate(const dvec2 &vector)
{
	position.x += vector.x;
	position.y += vector.y;
}

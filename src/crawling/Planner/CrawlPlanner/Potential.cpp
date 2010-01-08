#include "Potential.h"
#include <math.h>

#include <glm/gtx.hpp>
using namespace gtx::norm;

Potential::Potential(void)
{
}

Potential::Potential(double x, double y, double myRadius, double myPotential) : position(x,y), radius(myRadius), potential(myPotential)//, reached(false)
{
}


Potential::~Potential(void)
{
}

const dvec2 &Potential::GetPosition(void) const
{
	return position;
}

dvec2 Potential::GetPotentialVector(void) const
{
    dvec2 potentialVector;

    //if(reached)
    //{
    //    potentialVector.x = 0;
    //    potentialVector.y = 0;
    //    return potentialVector;
    //}

	dvec2 normalizedPotentialVector;
    normalizedPotentialVector = -potential * normalize(position);

    double distance = sqrt(length2(position)) - radius;
    if(potential < 0)
    {
        potentialVector = normalizedPotentialVector * NEGATIVE_GRADIENT_EXPRESSION;
    }
    else
    {
        if(distance > d0)
        {
            potentialVector = dvec2(0,0);
        }
        else
        {
            potentialVector = normalizedPotentialVector * POSITIVE_GRADIENT_EXPRESSION;
        }
    }

    //if(length2(potentialVector)<EPSILON_LENGTH)
    //{
    //    return dvec2(0,0);
    //}

	return potentialVector;
}

void Potential::Translate(const dvec2 &t)
{
    position += t;
}

void Potential::Rotate(double angle)
{
	double teta = radians(angle);
	double xPrime, yPrime;
	xPrime = position.x * cos(teta) - position.y * sin(teta);
	yPrime = position.y * cos(teta) + position.x * sin(teta);

	position.x = xPrime;
	position.y = yPrime;
}

void Potential::SetReached(void)
{
    /*reached = true;*/
    potential = 0;
}

double Potential::GetRadius(void) const
{
    return radius;
}


double Potential::GetPotential(void) const
{
    return potential;
}

void Potential::MoveTo(double x, double y)
{
	position.x = x;
	position.y = y;
}

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/double_float.hpp>
using namespace glm;

//#define POSITIVE_GRADIENT_EXPRESSION (1 / pow(distance,5)) // the gradient at point P is inversely proportional to distance^2.
//#define NEGATIVE_GRADIENT_EXPRESSION (1 / pow(distance,2)) // the gradient at point P is inversely proportional to distance^2.
#define d0 1
#define kp 2
#define kn 2
#define POSITIVE_GRADIENT_EXPRESSION (1/distance - 1/d0)*(1 / pow(distance,2)) // the gradient at point P is inversely proportional to distance^kp.
#define NEGATIVE_GRADIENT_EXPRESSION (1 / pow(distance, kn)) // the gradient at point P is inversely proportional to distance^kn  (kn <= 1).
#define EPSILON_LENGTH 0.01

class Potential
{
protected:
	dvec2 position;
    double radius;
	double potential;

	bool reachingMode;
    //bool reached;
public:
	Potential(void);
	Potential(double x, double y, double myRadius, double myPotential);
	~Potential(void);
	const dvec2 &GetPosition(void) const;
    double GetRadius(void) const;
    double GetPotential(void) const;
	dvec2 GetPotentialVector(void) const;
	void Translate(const dvec2 &t);
	void Rotate(double angle);
    void SetReached(void);
	void MoveTo(double x, double y);
};

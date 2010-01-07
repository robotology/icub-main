#include "Patch.h"


Potential::Potential(double myX, double myY, double myRadius, double myPotential) 
    : Thing(myX, myY), radius(myRadius), potential(myPotential)
{
}

Potential::~Potential(void)
{
}

void Potential::TransformToWindowCoordinates(int originX, int originY, double scalingFactor)
{
    Thing::TransformToWindowCoordinates(originX, originY, scalingFactor);
    radius *= scalingFactor;
}


void Potential::Draw(Cairo::RefPtr<Cairo::Context> myCr) const
{
	TransformContext(myCr);
    myCr->set_line_width(LINE_WIDTH);
    if(potential > 0)
    {
        myCr->set_source_rgb(1,0,0);
    }
    else if (potential < 0)
    {
        myCr->set_source_rgb(0,1,0);
    }
    myCr->arc(0, 0, radius, 0.0, 2 * M_PI);
    myCr->fill();

    if(potential>0)
    {
        double currentRadius = radius + CIRCLE_STEP;
        for(;;)
        {
            double distance = (currentRadius - radius) / 100;
            double currentPotential = POSITIVE_GRADIENT_EXPRESSION;

            if(currentPotential<EPSILON)
            {
                break;
            }

            if(distance>d0)
            {
                break;
            }

            myCr->set_source_rgba(1,0,0,currentPotential);
            myCr->arc(0, 0, currentRadius, 0.0, 2 * M_PI);
            myCr->stroke();

            currentRadius+=CIRCLE_STEP;
        }
    }
    else
    {
        double currentRadius = radius + CIRCLE_STEP;
        for(;;)
        {
            double distance = (currentRadius - radius) / 100;
            double currentPotential = NEGATIVE_GRADIENT_EXPRESSION;

            if(currentPotential<EPSILON)
            {
                break;
            }
            
            myCr->set_source_rgba(0,1,0,currentPotential);
            myCr->arc(0, 0, currentRadius, 0.0, 2 * M_PI);
            myCr->stroke();

            currentRadius+=CIRCLE_STEP;
        }
    }

	RestoreContext(myCr);
}

//void Patch::Scale(double scalingFactor)
//{
//    radius = radius * scalingFactor;
//}

double Potential::GetPotential(void) const
{
    return potential;
}

#include "WorldDrawingArea.h"
#include <cairomm/context.h>
#include <math.h>

#include <glm/glm.hpp>
#include <glm/gtc/double_float.hpp>
using namespace glm;
#include <glm/gtx/norm.hpp>
using namespace gtx::norm;


#include "Patch.h"

WorldDrawingArea::WorldDrawingArea(int myWidth, int myHeight)
: vision(NULL), potentialVector(0,0)
{
}

WorldDrawingArea::~WorldDrawingArea(void)
{
	if(vision != NULL)
	{
		for(unsigned int i=0; i<vision->size(); ++i)
		{
			delete (*vision)[i];
		}
		delete vision;
	}
}

void WorldDrawingArea::SetVision(Vision *myVision)
{
	if(vision != NULL)
	{
		for(unsigned int i=0; i<vision->size(); ++i)
		{
			delete (*vision)[i];
		}
		delete vision;
	}
	vision = myVision;
}

void WorldDrawingArea::SetPotentialVector(const dvec2 &myPotentialVector)
{
    potentialVector = myPotentialVector;
}


bool WorldDrawingArea::on_expose_event(GdkEventExpose* event)
{
    // This is where we draw on the window
    Glib::RefPtr<Gdk::Window> window = get_window();
    if(window)
    {
        Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();
        int visionOriginY = get_height() - 150;
        int visionOriginX = get_width()/2;
        double scalingFactor = (get_height() - visionOriginY - 10) / VIEW_FAR;

        potentialVector.x = visionOriginX + scalingFactor*potentialVector.x;
        potentialVector.y = visionOriginY - scalingFactor*potentialVector.y;

        cr->arc(visionOriginX, visionOriginY, 5, 0.0, 2 * M_PI);
        cr->fill();
        cr->move_to(visionOriginX, visionOriginY);
        cr->line_to(potentialVector.x, potentialVector.y);
        cr->stroke();

        if(vision != NULL)
        {
            for(unsigned int i=0; i < vision->size(); ++i)
            {
                (* vision)[i]->TransformToWindowCoordinates(visionOriginX, visionOriginY, scalingFactor); 
                (* vision)[i]->Draw(cr);
            }
        }

    }

    return true;
}


        /*if(vision != NULL)
        {
            for(unsigned int i=0; i < vision->size(); ++i)
            {
                (* vision)[i]->TransformToWindowCoordinates(visionOriginX, visionOriginY, scalingFactor); 
            }
            int x=0; 
            int width = get_width();
            int height = get_height();
            while(x<width)
            {
                int y=0;
                while(y<height)
                {
                    dvec2 position(x,y);
                    double totalPotential=0;
                    for(unsigned int i=0; i < vision->size(); ++i)
                    {
                        dvec2 patchPosition = (* vision)[i]->GetPosition();
                        double distance = sqrt(distance2(patchPosition, position))  / (scalingFactor);
                        if(((Potential *)(* vision)[i])->GetPotential()>0)
                        {
                            if(distance<d0)
                            {
                                totalPotential += POSITIVE_GRADIENT_EXPRESSION;
                            }
                        }
                        else
                        {
                            totalPotential -= NEGATIVE_GRADIENT_EXPRESSION;
                        }
                    }
                    
					if(totalPotential<-EPSILON)
					{
						cr->set_source_rgb(0,-totalPotential,1);
					}
					else if(totalPotential>EPSILON)
					{
						cr->set_source_rgb(totalPotential,0,1);
					}
					else
					{
						cr->set_source_rgb(0,0,1);
					}

                    
                    cr->arc(x, y, 5, 0.0, 2 * M_PI);
                    cr->fill();

                    y+=DRAWING_STEP;
                }
                x+=DRAWING_STEP;
            }
        }*/

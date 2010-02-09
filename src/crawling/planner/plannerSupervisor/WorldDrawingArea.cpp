#include "WorldDrawingArea.h"
#include <cairomm/context.h>
#include <math.h>



#include "Patch.h"

WorldDrawingArea::WorldDrawingArea(int myWidth, int myHeight)
: vision(NULL), potentialVector(Vector(2))
{
	potentialVector.zero();
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

void WorldDrawingArea::SetPotentialVector(const Vector &myPotentialVector)
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

        potentialVector[0] = visionOriginX + scalingFactor * potentialVector[0];
        potentialVector[1] = visionOriginY - scalingFactor * potentialVector[1];

        cr->arc(visionOriginX, visionOriginY, 5, 0.0, 2 * M_PI);
        cr->fill();
        cr->move_to(visionOriginX, visionOriginY);
        cr->line_to(potentialVector[0], potentialVector[1]);
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
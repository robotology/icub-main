#pragma once
#include <gtkmm\drawingarea.h>
#include "GlobalStuff.h"

#define DRAWING_STEP 10
#define VIEW_FAR 1

class WorldDrawingArea :
    public Gtk::DrawingArea
{
public:
    WorldDrawingArea(int myWidth, int myHeight);
    virtual ~WorldDrawingArea(void);
    void SetVision(Vision *myVision);
    void SetPotentialVector(const Vector &myPotentialVector);
    

protected:
    Vision *vision;
    Vector potentialVector;
    virtual bool on_expose_event(GdkEventExpose* event);
    Cairo::RefPtr<Cairo::Context> cr;
};

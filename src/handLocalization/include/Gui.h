#ifndef _GUI_
#define _GUI_

#include <includes.h>

//#include <cv.h>
//#include <highgui.h>
#include <math.h>

// namespaces 
namespace thesis {
    /**
     * GUI to display information (of more complex stuff).
     */
    namespace gui {
		class Gui;
    }
}


// ***************************************************************************
// ********                   namespace thesis::gui                   ********
// ***************************************************************************

/**
 *  Class Gui: definition 
 *
 */
class thesis::gui::Gui {
private:
	// Declaration of hidden class variables
	ImageOf<PixelRgb>	tmp_rgb;
	ImageOf<PixelMono>	tmp_mono;
	struct Colors {
		PixelRgb azzuro;
		PixelRgb black;
		PixelRgb blue;
		PixelRgb darkgreen;
		PixelRgb darkred;
		PixelRgb lightblue;
		PixelRgb orange;
		PixelRgb red;
		PixelRgb green;
		PixelRgb white;
		PixelRgb yellow;
	};
	struct MonoColors {
		PixelMono black;
		PixelMono grey;
		PixelMono white;
	};

	// Declaration of hidden  methods

public:
	// Declaration of class variables
	bool withinBorders(int _i, int _j, int _w, int _h);
	Colors c;
	MonoColors c_m;

	// Declaration of Constructor, Destructor and methods
	Gui();
	~Gui();

	void	placeAxis(ImageOf<PixelRgb> *_img_ptr, string _axis, PixelRgb _color, int _pos, bool _steps);
	void	connectPixels(ImageOf<PixelRgb> *_img_ptr, PixelRgb _pixcolor, int _x_i, int _y_i, int _x_ip1, int _y_ip1);

	void grid(ImageOf<PixelMono> *_img_ptr, int _interval, PixelRgb _color);
	void grid(ImageOf<PixelRgb> *_img_ptr, int _interval, PixelRgb _color);
};

#endif

// include header
#include <Gui.h>

// namespaces
using namespace halode::gui;

// ***************************************************************************

/**
 * Implementation of Gui class
 *
 * This "module" is working on array data to visualize the different information. 
 * Position in 2D-space or velocity and more.
 *
 */

// Default constructor
Gui::Gui() {

	this->c.azzuro		= PixelRgb (220,242,231);
	this->c.black		= PixelRgb (0,0,0);
	this->c.blue		= PixelRgb (0,0,255);
	this->c.lightblue	= PixelRgb (27,181,219);
	this->c.darkgreen	= PixelRgb (27,130,4);
	this->c.green		= PixelRgb (0,255,0);
	this->c.orange		= PixelRgb (250,168,26);
	this->c.red			= PixelRgb (255, 0, 0);
	this->c.darkred		= PixelRgb (247, 8, 8);
	this->c.white		= PixelRgb (255,255,255);
	this->c.yellow		= PixelRgb (255,222,59);


	this->c_m.black		= PixelMono (0);
	this->c_m.grey		= PixelMono (125);
	this->c_m.white		= PixelMono (255);
}

// Destructor
Gui::~Gui() {
	//printf("Quit:\t[Gui]\n");
}

/**
* this method connects to pixel in a 2D-grid with each other. 
*
* pixels have are connected in the mode p(i) -> p(i+1) : (x_i,y_i) -> (x_ip1, y_p1)
*/
void Gui::connectPixels(ImageOf<PixelRgb> *_img_ptr, PixelRgb _pixcolor, int _x_i, int _y_i, int _x_ip1, int _y_ip1) {
	int step = 0;
	int x_diff, y_diff;
	int vx, vy;
	double tan = 1.0;
	PixelRgb punto;
	punto = _pixcolor;

	x_diff = (_x_ip1 - _x_i);
	y_diff = (_y_ip1 - _y_i);
	if (x_diff != 0) {
		if (x_diff < 0 ) {
			vx = -1;
		}
		else {
			vx = 1;
		}
	}
	else {
		vx = 1;
	}
	if (y_diff != 0) {
		if (y_diff < 0 ) {
			vy = -1;
		}
		else {
			vy = 1;
		}
	}
	else {
		vy = 1;
	}
	if (abs(y_diff) > abs(x_diff)) {
		// tangens, gradient of the connection between two pixels
		tan = (double)x_diff/(double)y_diff;
		for (int k=0; k < (abs(y_diff) + 1); k++) {
			step = abs( ROUNDTOINT( tan * (double)k ) );
			_img_ptr->safePixel(_x_i + vx*step, _y_i + k*vy) = punto;
		}
	}
	else if (abs(x_diff) > abs(y_diff)) {
		// tangens, gradient of the connection between two pixels
		tan = (double)y_diff/(double)x_diff;
		for (int k=0; k < (abs(x_diff)+1); k++) {
			step = abs( ROUNDTOINT( tan * (double)k ) );
			_img_ptr->safePixel(_x_i + k*vx, _y_i + vy*step) = punto;
		}
	} 
	else if (abs(x_diff) == abs(y_diff)) {
		for (int k=0; k < (abs(x_diff)+1); k++) {
			_img_ptr->safePixel( _x_i + k*vx, _y_i + k*vy) = punto;
		}
	}
}

/**
* this method connects to pixel in a 2D-grid with each other. 
*
* pixels have are connected in the mode p(i) -> p(i+1) : (x_i,y_i) -> (x_ip1, y_p1)
*
* for ImageOf<PixelMono>
*
*/
void Gui::connectPixels(ImageOf<PixelMono> *_img_ptr, PixelMono _pixcolor, int _x_i, int _y_i, int _x_ip1, int _y_ip1) {
	int step = 0;
	int x_diff, y_diff;
	int vx, vy;
	double tan = 1.0;
	PixelMono punto = _pixcolor;

	x_diff = (_x_ip1 - _x_i);
	y_diff = (_y_ip1 - _y_i);
	if (x_diff != 0) {
		if (x_diff < 0 ) {
			vx = -1;
		}
		else {
			vx = 1;
		}
	}
	else {
		vx = 1;
	}
	if (y_diff != 0) {
		if (y_diff < 0 ) {
			vy = -1;
		}
		else {
			vy = 1;
		}
	}
	else {
		vy = 1;
	}
	if (abs(y_diff) > abs(x_diff)) {
		// tangens, gradient of the connection between two pixels
		tan = (double)x_diff/(double)y_diff;
		for (int k=0; k < (abs(y_diff) + 1); k++) {
			step = abs( ROUNDTOINT( tan * (double)k ) );
			_img_ptr->safePixel(_x_i + vx*step, _y_i + k*vy) = punto;
		}
	}
	else if (abs(x_diff) > abs(y_diff)) {
		// tangens, gradient of the connection between two pixels
		tan = (double)y_diff/(double)x_diff;
		for (int k=0; k < (abs(x_diff)+1); k++) {
			step = abs( ROUNDTOINT( tan * (double)k ) );
			_img_ptr->safePixel(_x_i + k*vx, _y_i + vy*step) = punto;
		}
	} 
	else if (abs(x_diff) == abs(y_diff)) {
		for (int k=0; k < (abs(x_diff)+1); k++) {
			_img_ptr->safePixel( _x_i + k*vx, _y_i + k*vy) = punto;
		}
	}
}


void Gui::grid(ImageOf<PixelRgb> *_img_ptr, int _interval, PixelRgb _color) {
	int grid_x = ROUNDTOINT((double)_img_ptr->width()  / (double)_interval);
	int grid_y = ROUNDTOINT((double)_img_ptr->height() / (double)_interval);

	for (int j=0; j<_img_ptr->height(); j++) {
		for (int i =0 ; i<_img_ptr->width(); i++) {
			if ((j % _interval) == 0) {
				_img_ptr->safePixel(i,j) = _color;
			}
			if ((i % _interval) == 0) {
				_img_ptr->safePixel(i,j) = _color;
			}
		}
	}
}


/**
*
*
**/
void Gui::placeAxis(ImageOf<PixelRgb> *_img_ptr, string _axis, PixelRgb _color, int _pos, bool _steps) {

	int			height, len_h;
	int			width, len_w;
	const int	step	= 10;

	height		= _img_ptr->height();
	width		= _img_ptr->width();
	len_h		= ROUNDTOINT((height - 2*step) / 2);
	len_w		= ROUNDTOINT((width  - 2*step) / 2);


	if (_axis == "horizontal") {
		// x-axis
		addRectangle(*_img_ptr, _color, (int)floor(width/2), _pos, len_w, 0);
		if (_steps) {
			for (int i=1; i<(width/step); i++) {
				addRectangle(*_img_ptr, _color, i*step, _pos, 0, 3);
			}
		}
	}
	else if (_axis == "vertical") {
		// y-axis
		addRectangle(*_img_ptr, _color, _pos, (int)floor(height/2), 0, len_h);
		if (_steps) {
			for (int i=1; i<(height/step); i++) {
				addRectangle(*_img_ptr, _color, _pos, i*step, 3, 0);
			}
		}
	}
}

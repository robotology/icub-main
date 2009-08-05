#ifndef _HL_INCLUDES_
#define _HL_INCLUDES_

//------------------------------------------------------------------------------
// general definitions
//------------------------------------------------------------------------------
//#define PIXEL PixelMono
#define MAX(a, b)    ((a < b) ? b : a)
#define MIN(a, b)    ((a > b) ? b : a)
#define ROUNDTOINT(a)	 ((int)floor((double)a + 0.5))
#define PI	3.141592653589793234846

// BICS06 here we define the fix size of the footprints
#define FOOTPRINT_SIZE 11

//includes
// YARP2
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>

// including own includes

// including standard includes
#include <string>
#include <stdio.h>
#include <iostream>
#include <math.h>

// even if it is not very beautiful style, usings are already defined here.
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::os;

using std::string;
using std::cout;
using std::cin;
using std::endl;

// constructs of data types for all classes
struct dPoint2D {
	double x, y;
};

struct dPoint3D {
	double x, y, z;
};

struct iPoint2D {
	int x, y;
};

struct iPoint3D {
	int x, y, z;
};

struct dBottle2D {
	Bottle x;
	Bottle y;
};

struct dBottle3D {
	Bottle x;
	Bottle y;
	Bottle z;
};

struct iBottle2D {
	Bottle x;
	Bottle y;
};

struct patchInfo {
	iBottle2D	patch_id;
	dBottle2D	vector_value;
	Bottle		time;
};

struct bgRange {
	iPoint2D row;
	iPoint2D col;
	bool launch;
};


class Parameters {
private:
	string		stat_str, halo_str, hade_str;
public:
	Property	properties, dumpprops;

	Parameters (string _config_file);
	~Parameters ();

	// operating system
	string		config_file;
	bool		linuxFlag;

	// execution settings
	string		execution;
	//string		execution_mode;
	bool		stereovision;
	int			nof_loops;

	// recording and display settings
	bool		rec_perception, rec_result, rec_statistics, rec_steps, display;

	// I/O settings (files and locations)
	const char	*statisticsFile;
	const char	*result_dir_1;
	const char	*result_dir_2;

	// Parameter settings

	// File dump settings (left eye)
	string		dump1_source, dump1_dir, dump1_name, dump1_encs, dump1_end;
	int			dump1_cnt_len, dump1_max_filenum, dump1_min_filenum;
	bool		dump1_zerofilling;

	// File dump settings (right eye)
	string		dump2_source, dump2_dir, dump2_name, dump2_encs, dump2_end;
	int			dump2_cnt_len, dump2_max_filenum, dump2_min_filenum;
	bool		dump2_zerofilling;

	// runtime-generic settings
	int			startTime, netCounter, exec_loop;
	int			img_width, img_height, nof_joints;
	int			merger_counter, interrupt_count;

	// interna:

	// ---------------------------------
	// ports
	// ---------------------------------
	// source in the world
	string		cam1_src, cam2_src, encs_src;

	//destination in the world
	string		viewer1_dst, viewer2_dst;

	// interface from the outside world
	string		cam1_in_port;
	string		cam2_in_port;
	string		enc_in_port;

	// interface to the outside world
	string		cam1_out_port;
	string		cam2_out_port;

	// not to change:
	string		dataLeft_port;
	string		dataRight_port;

	// markerTracker position ports (for the outside world)
	string		trackerLeft_out_port;
	string		trackerRight_out_port;
	
	// methods
	int			getIntParameter(const char *_groupName_ptr, const char *_paramName_ptr);
	double		getDoubleParameter(const char *_groupName_ptr, const char *_paramName_ptr);
	string		getTextParameter(const char *_groupName_ptr, const char *_paramName_ptr);
	bool		getBool(const char *_onOff_ptr);

	void		setIntParameter(const char *_paramName_ptr, int _paramVal);
	void		setIntParameter(int *_paramName_ptr, int _paramValue);
	void		setDoubleParameter(double *_paramName_ptr, double _paramValue);

	void		changeParameter(string _paramName);

	void		printAndAsk(const char *_param_ptr, int *_value_ptr);
	void		printAndAsk(const char *_param_ptr, bool *_value_ptr);
	void		printAndAsk(const char *_param_ptr, double *_value_ptr);
	void		printAndAsk(const char *_param_ptr, string *_value_ptr);

	void		loadParameters();
	
};



#endif


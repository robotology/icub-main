#include "Defn.h"

// File: FlowJob.h
// Author: Chris McCarthy (cdmcc@rsise.anu.edu.au)
// Date: May 2006
//
// Defines a flow job to be executed on the input image stream
//

#ifndef FLOWJOB_H
#define FLOWJOB_H

class FlowJob {
	public:

	 FlowJob(int image_width, int image_height, int offset=5);

	 FlowJob(int image_width, int image_height, float tau_d, int vector_skip, int id, int offset);

	 FlowJob(int image_width, int image_height, int x_centre, int y_centre, 
				  int vector_width, int vector_height, float tau_D, 
				  int vector_skip, int id, float vector_scale, int offset);

	   int flowjob_id;
		// image dimensions
		int image_width;
		int image_height;
		int image_offset;  // account for filter size 
		int flowimage_u_start;
		int flowimage_u_end;
		int flowimage_v_start;
		int flowimage_v_end;

		// vector dimensions
		int vector_width;
		int vector_height;
		int vector_skip;
		float vector_scale;

		//image coordinates of flow job box
		int x_centre;
		int y_centre;
		int x_start;
		int x_end;
		int y_start;
		int y_end;

		float tau_D;
		int within_bounds;
};
#endif

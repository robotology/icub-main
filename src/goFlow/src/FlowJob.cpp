// File: FlowJob.h
// Author: Chris McCarthy (cdmcc@rsise.anu.edu.au)
// Date: May 2006
// 
//  Defines a flow job to be executed on the input image stream

#include <FlowJob.h>

// compute flow over full image

FlowJob::FlowJob(int image_width, int image_height, int offset)
{
		this->flowjob_id = 0; 
   	this->image_width = image_width;
   	this->image_height = image_height;
      this->image_offset = offset;
      this->flowimage_u_start = offset;
      this->flowimage_u_end = image_width - offset;
      this->flowimage_v_start = offset;
      this->flowimage_v_end = image_height - offset;

      this->x_centre = image_width/2;
      this->y_centre = image_height/2;
      this->vector_width = image_width - 2*offset;
      this->vector_height = image_height - 2*offset;
      this->vector_scale = 1;
      this->vector_skip = 1;

      int half_width = (int) (vector_width / 2);
      int half_height = (int) (vector_height / 2);

      this->x_start = x_centre - (half_width-2);
      this->x_end  = x_centre + (half_width-2);
      this->y_start = y_centre - (half_height-2);
      this->y_end = y_centre + (half_height-2);

      this->tau_D = 0.0;
      this->within_bounds = TRUE;
}

FlowJob::FlowJob(int image_width, int image_height, float tau_D, int vector_skip, int id, int offset)
{
		 this->flowjob_id = id;
		 this->image_width = image_width;
		 this->image_height = image_height;
		 this->image_offset = offset;
		 this->flowimage_u_start = offset;
		 this->flowimage_u_end = image_width - offset;
		 this->flowimage_v_start = offset;
		 this->flowimage_v_end = image_height - offset;

		 this->x_centre = image_width/2;
		 this->y_centre = image_height/2;
		 this->vector_width = image_width - 2*offset;
		 this->vector_height = image_height - 2*offset;
		 this->vector_scale = 1;
		 this->vector_skip = vector_skip; 
		 
		 int half_width = (int) (vector_width / 2);
       int half_height = (int) (vector_height / 2);

       this->x_start = x_centre - (half_width-2);
       this->x_end  = x_centre + (half_width-2);
       this->y_start = y_centre - (half_height-2);
       this->y_end = y_centre + (half_height-2);
		
		 this->tau_D = tau_D; 
		 this->within_bounds = TRUE;

}


// define region to compute flow
FlowJob::FlowJob(int image_width, int image_height, int x_centre,
							   int y_centre, int vector_width, int vector_height,
								float tau_d, int vector_skip, int id, 
								float vector_scale, int offset)
{
		  this->flowjob_id = id;
		  this->image_width = image_width;
		  this->image_height = image_height;
		  this->flowimage_u_start = offset;
        this->flowimage_u_end = image_width - offset;
        this->flowimage_v_start = offset;
        this->flowimage_v_end = image_height - offset;
		  this->x_centre = x_centre;
		  this->y_centre = y_centre;
		  this->vector_width = vector_width;
		  this->vector_height = vector_height;
		  this->vector_scale = vector_scale;
		  this->vector_skip = vector_skip;

		  int half_width = (int) (vector_width / 2);
		  int half_height = (int) (vector_height / 2);

		  this->x_start = x_centre - half_width + offset;
		  this->x_end  = x_centre + half_width - offset;
		  this->y_start = y_centre - half_height + offset;
		  this->y_end = y_centre + half_height - offset;

		  this->tau_D = tau_D;

		  if(x_start < flowimage_u_start || x_end > flowimage_u_end || 
			  y_start < flowimage_v_end || y_end > flowimage_v_end)
					 this->within_bounds = FALSE;
		  else
					 this->within_bounds = TRUE;
		  
}

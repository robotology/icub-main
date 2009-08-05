#ifndef FLOW_H
#define FLOW_H

#include "ImageStack.h"
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include "Defn.h"
#include "FlowJob.h"

#define HORN	0
#define LUCAS 	1
#define URAS	2
#define NAGEL	3

#define ANANDAN	4

#define CAMUS	5

#define RAW_NORM	6


#define NO_FLOW	99


#define MAX_IMAGES	5	


#define NO_FILTER	0


// for Camus' method
#define SAD		0
#define	ZSAD	1
#define SSD		2
#define	ZSSD	3
#define	NCC		4
#define	ZNCC	5

#define ROOT2 1.4142136
#define ROOT8 2.8284271
#define ROOT5 2.2360679

// kernel labels 
#define SIMONCELLI	1
#define GAUSSIAN	2
#define RECURSIVE_TEMPORAL	3
#define HIEDELBERG	4
#define CENTRAL_DIV	5
#define CENTRAL_NODIV	6

#define SIMONCELLI_MAX	5	
#define FLOW_NO_VALUE	100.0
#define IMAGE_NO_VALUE	1000.0
#define URAS_NO_VALUE 1.0
#define N 2

#define TRUE	1
#define FALSE	0

#define MAXFLOW	20.0

#define KERNEL_SIZE	5
#define REGION_SIZE 8

#define TR1		1
#define TR2		0

using namespace yarp::sig;

class Flow
{
	private:
		int EI;
		int EJ;
		float FLOW_PI;
		int FLOW_HEAD;
		int MAX_I;
		int WHITE;
		int BLACK;
		int OK;
		int GREY;
		float SINGULAR_DETERMINANT;
		int NONE;
		float SMALL;
		float VERY_SMALL;
		float OF_HUGE;
		float BIG_THRESHOLD;
		float SIGNIFICANT;
		int IVALUE;
		int JVALUE;
		int HAGEN;
		int NR_END;
		float RAW_MAG;
		float MAX_RAW_MAG;

	public: Flow();  

	// loads images for optical flow.  Specify OF algorithm and filter.  max images
	public: void load_frames(ImageStack<ImageOf<PixelMono> > *is, int max_images, int pic_x, int pic_y);

	// apply simoncelli filtering (assumes images have been loaded via
	// init().  PREFILTER is TRUE or FALSE

	// FlowJob functions for simoncelli fulter
	public: void apply_simoncelli_prefilter(FlowJob *job);

	public: void apply_simoncelli_filter(FlowJob *job);

	private: void calc_lowpass_kernel(float  p_kernel[5], int filter);

	private: void calc_highpass_kernel(float  d_kernel[5], int filter);

 	private: void convolve_in_t(float  *out, unsigned char  *in[MAX_IMAGES], float  *kernel, int kernel_size, FlowJob *job);

   private: void convolve_in_t(float  *out, float *in[MAX_IMAGES], float  *kernel, int kernel_size, FlowJob *job);

	private: void convolve_in_x(float *out, float *in, float *kernel, int filter_size, FlowJob *job);

	private: void convolve_in_x(float *out, unsigned char *in, float *kernel, int filter_size, FlowJob *job);

	private: void convolve_in_y(float *out, float *in,float *kernel, int filter_size, FlowJob *job);


	/*****  Lucas and Kanade using FlowJob functions *****/
			// run this once before running first optical flow calculation
   public: void setup_lucas_optical_flow();

			// run this prior to running jobs over a frame buffer
   public: void init_lucas_optical_flow();

			// compute Lucas and Kanade optical flow (one call does all)
	public: void calc_lucas_optical_flow(ImageStack<ImageOf<PixelMono> > *is, 
												FlowJob *job, ImageOf<PixelFloat> *u_vels,
											  	ImageOf<PixelFloat> *v_vels);	

			// run after all jobs have been completed on frame buffer	
	private: void tidy_lucas_optical_flow(FlowJob *job, ImageOf<PixelFloat> *u_vels, ImageOf<PixelFloat> *v_vels);	
	/***** end FlowJob functions  *****/

	private: float dot(float  a[3],float b[3]);

	private: int eigen(float **mat,float **evt,float *evals,short n, int a, int b);

	private: float** a_fmatr(short dy,short dx);

	private: void f_fmatr(float **mat);

	private: float * vector(long nl, long nh);

	private: void vector(int loc[2],float vect[2]);

	private: void free_vector(float *v, long nl, long nh);

	private: void jacobi(float  aa[N][N],int  n,float d[N],float v[N][N],int *nrot);

	private: void j_jacobi(float **a, int n, float d[], float **v, int *nrot);

	private: float too_big(float  v[3]);

	private: float too_big3(float  v[3]);

	private: void cross_product(float  a[3],float b[3],float c[3]);

	private: int about(float  a,float b);

	private: int check_eigen_calc(float  mm[N][N],float d[N],float v[N][N],int  n,float diff1[N],float diff2[N],float *length1,float *length2,float *angle);

	private: void rotate(float  a[N][N],int  i,int j,int k,int l,float  *h,float *g,float s,float tau);


	//public: void calc_anandan_flow(int sm, int sp, int sf, int pyr,

	//								 int ssd, int f_type, float trsh, int iter);


//	public: int get_anandan_offset();

	public: void calc_raw_normals(int offset, float trsh);

	public: 

		// images for computing flow on
	public: float *floatpic[KERNEL_SIZE];
	public: unsigned char *intpic[MAX_IMAGES];
		
	public:
		int max_images;

	private:
		int pic_x, pic_y;
		int offset;

		int filter_method;
		
		// first order partial derivatives of image intensity function
	public: float *Ix;
	public: float *Iy;
	public: float *It;

	private:

	   // for lucas
		float weight[5][5];
	 	float coeff[5];
		float Imag[PIC_X][PIC_Y];
		float norm_vels1[PIC_X][PIC_Y][2];
		float norm_vels2[PIC_X][PIC_Y][2];
		float E[PIC_X][PIC_Y];
		int full_count;
		int norm_count1,norm_count2,no_count,eigen_count, no_swaps;

		float *smooth_in_x[SIMONCELLI_MAX];
		float *smooth_in_y[SIMONCELLI_MAX];
		float low_pass1[SIMONCELLI_MAX][PIC_X][PIC_Y];
		float low_pass2[SIMONCELLI_MAX][PIC_X][PIC_Y];
		float full_vels[PIC_X][PIC_Y][2];
									 

	public: int KERNEL_X, KERNEL_Y;	/* uras regularisation offsets*/
			int of_method;
		
	public:
		int iter_time;
		int aver_time;
};

#endif

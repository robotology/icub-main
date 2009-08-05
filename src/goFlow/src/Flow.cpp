#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include <Flow.h>

#include <math.h>

#define UNMATCHED 255

#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
                          a[k][l]=h+s*(g-h*tau);
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)

#define Q(T) (T/(T+2))
#define R(T) ((T-2)/(T+2))

#define xabs(A,x) ((x=(A))<0 ? -x:x) 

#define si short int

#define sad_diff(w0,w1,d,x) \

                 (xabs((si)w1[0][d]-*w0[0],x)+xabs((si)w1[1][d]-*w0[1],x)+\

                  xabs((si)w1[2][d]-*w0[2],x)+xabs((si)w1[3][d]-*w0[3],x)+\

                  xabs((si)w1[4][d]-*w0[4],x)+xabs((si)w1[5][d]-*w0[5],x)+\

                  xabs((si)w1[6][d]-*w0[6],x)+xabs((si)w1[7][d]-*w0[7],x)+\

                  xabs((si)w1[8][d]-*w0[8],x))



#define sqd(a,b) (((int)b-a))*((int)b-a)

#define ssd_diff(w0,w1,d) \

                 (sqd(*w0[0],w1[0][d])+sqd(*w0[1],w1[1][d])+\

                  sqd(*w0[2],w1[2][d])+sqd(*w0[3],w1[3][d])+\

                  sqd(*w0[4],w1[4][d])+sqd(*w0[5],w1[5][d])+\

                  sqd(*w0[6],w1[6][d])+sqd(*w0[7],w1[7][d])+\

                  sqd(*w0[8],w1[8][d])+sqd(*w0[9],w1[9][d])+\

				  sqd(*w0[10],w1[10][d])+\

                  sqd(*w0[11],w1[11][d])+sqd(*w0[12],w1[12][d])+\

                  sqd(*w0[13],w1[13][d])+sqd(*w0[14],w1[14][d])+\

                  sqd(*w0[15],w1[15][d])+sqd(*w0[16],w1[16][d])+\

                  sqd(*w0[17],w1[17][d])+sqd(*w0[18],w1[18][d])+\

				  sqd(*w0[19],w1[19][d])+\

                  sqd(*w0[20],w1[20][d])+sqd(*w0[21],w1[21][d])+\

                  sqd(*w0[22],w1[22][d])+sqd(*w0[23],w1[23][d])+\

                  sqd(*w0[24],w1[24][d]))



#define sop(w0,w1,d0,d1)\

                 (w0[0][d0]*w1[0][d1]+w0[1][d0]*w1[1][d1]+w0[1][d0]*w1[1][d1]+\

                  w0[2][d0]*w1[2][d1]+w0[3][d0]*w1[3][d1]+w0[4][d0]*w1[4][d1]+\

                  w0[5][d0]*w1[5][d1]+w0[6][d0]*w1[6][d1]+w0[7][d0]*w1[7][d1]+\

                  w0[8][d0]*w1[8][d1])

#define ncc_diff(w0,w1,d,L2) (-sop(w0,w1,0,d)/sqrt(L2*sop(w1,w1,d,d)))





// nagel macros

#define N_DUX(x,y,vels) (0.5 * vels[y][x-1][0] - vels[y][x+1][0]) 

#define N_DUY(x,y,vels) (0.5 * vels[y-1][x][0] - vels[y+1][x][0])

#define N_DVX(x,y,vels) (0.5 * vels[y][x-1][1] - vels[y][x+1][1])

#define N_DVY(x,y,vels) (0.5 * vels[y-1][x][1] - vels[y+1][x][1])
#define N_DUXY(x,y,vels) (0.5 * N_DUX(x,y-1,vels) - N_DUX(x,y+1,vels))

#define N_DVXY(x,y,vels) (0.5 * N_DVX(x,y-1,vels) - N_DVX(x,y+1, vels))

#define WLAPU(x,y,a,c,vels) (a*(vels[y][x-1][0] + vels[y][x+1][0]) + c*(vels[y-1][x][0] + vels[y+1][x][0]))

#define WLAPV(x,y,a,c,vels) (a*(vels[y][x-1][1] + vels[y][x+1][1]) + c*(vels[y-1][x][1] + vels[y+1][x][1]))

  
#define FREE_ARG char*
#define N 2
#define MOVERFLOW	222	

#define MAX_COND    100000.0
#define NRADIUS     4

Flow::Flow()
{
	EI = 120;
	EJ = 80;
	FLOW_PI = M_PI;
	FLOW_HEAD = 32;
	MAX_I = 25;
	WHITE = 255;
	BLACK = 0;
	OK = 255;
	GREY = 127;
	SINGULAR_DETERMINANT = 0.00001;
	NONE = 654321;
	SMALL = 0.000001;
	VERY_SMALL = 0.000000001;
	OF_HUGE = 1.0e9;
	BIG_THRESHOLD = 1000000000.0;
	SIGNIFICANT = 1.0;
	IVALUE = JVALUE = 64;
	HAGEN = FALSE;
	NR_END = 1;
	RAW_MAG = -(0.0000001);
	MAX_RAW_MAG = -HUGE;
	max_images = 0;
	KERNEL_X = 0;
	KERNEL_Y = 0;
	max_images = 5;
	
	Ix = new float[PIC_X*PIC_Y];
	Iy = new float[PIC_X*PIC_Y];
	It = new float[PIC_X*PIC_Y];

	for(int i = 0; i < max_images; i++){
		smooth_in_x[i] = new float[PIC_X*PIC_Y];
		smooth_in_y[i] = new float[PIC_X*PIC_Y];
		floatpic[i] = new float[PIC_X*PIC_Y];
	}	
	setup_lucas_optical_flow();
}



void Flow::load_frames(ImageStack<ImageOf<PixelMono> > *is, int max_images, int pic_x, int pic_y)
{
	unsigned char *temp_save;
	
	if(max_images < this->max_images) {
		fprintf(stderr, "not enough images for this filter\n");
		exit(EXIT_FAILURE);
	}
	while(is->isLocked())
			  Time::delay(0.1);
	is->Lock();
	this->pic_x = pic_x;
	this->pic_y = pic_y;

	for(int k=0;k<max_images;k++) {
			   ImageOf<PixelMono> *img = (is->getImage(k+1));
			/*
				if(img->getRawImage() == NULL){
					printf("raw image is NULL!\n");
				}
				for(int i = 0; i < img.width(); i++){
                    for(int j = 0; j < img.height(); j++){
                       printf("%d\n", img.pixel(i,j));
                     }
               }
			*/
		intpic[k] = new unsigned char[pic_x*pic_y];
		memcpy(intpic[k], img->getRawImage(), pic_x*pic_y*sizeof(unsigned char));
	}
	is->UnLock();
}


void Flow::apply_simoncelli_prefilter(FlowJob *job)
{  
   int k, i, j;
	
	for(k=0;k<SIMONCELLI_MAX;k++)
         for(i=0;i<pic_x;i++)
            for(j=0;j< pic_y;j++)
               low_pass1[k][i][j] = 0.25* *(intpic[k] + i + j*pic_x) +
                                    0.5* *(intpic[k+1] + i + j*pic_x) +
                                    0.25* *(intpic[k+2] + i + j*pic_x);

      for(k=0;k<SIMONCELLI_MAX;k++)
         for(i=1;i<pic_x;i++)
            for(j=1;j<pic_y;j++)
               low_pass2[k][i][j] = 0.25*low_pass1[k][i-1][j]+
                                    0.5*low_pass1[k][i][j]+
                                    0.25*low_pass1[k][i+1][j];

      for(k=0;k<SIMONCELLI_MAX;k++)
         for(i=1;i<pic_x;i++)
            for(j=1;j<pic_y;j++)
               *(floatpic[k] + i + j*pic_x) = 0.25*low_pass2[k][i][j-1]+
                                   0.5*low_pass2[k][i][j]+
                                   0.25*low_pass2[k][i][j+1];
}

void Flow::apply_simoncelli_filter(FlowJob *job)
{
  int i;

  float p_kernel[SIMONCELLI_MAX],ave1,ave2,ave3;
  float d_kernel[SIMONCELLI_MAX],threshold;
  float *pre_in_t = new float[job->image_width*job->image_height];
  float *pre_in_x = new float[job->image_width*job->image_height];
  float *pre_in_y = new float[job->image_width*job->image_height];


  calc_lowpass_kernel(p_kernel, SIMONCELLI);
  calc_highpass_kernel(d_kernel, SIMONCELLI);


	for(int j=0;j<pic_x;j++){
      for(int k=0;k<pic_y;k++){
         /*printf("[%d][%d]=%d, %d, %d, %d, %d\n", j, k, intpic[0][j+k*pic_x], intpic[1][j+k*pic_x], intpic[2][j+k*pic_x], intpic[3][j+k*pic_x], intpic[4][j+k*pic_x]);*/
         }
      }

  convolve_in_t(pre_in_t, intpic, p_kernel, 5, job);
  convolve_in_x(pre_in_x, pre_in_t, p_kernel, 5, job);
  convolve_in_y(pre_in_y, pre_in_t, p_kernel, 5, job);


  convolve_in_x(Ix, pre_in_y, d_kernel, 5, job);
  convolve_in_y(Iy, pre_in_x, d_kernel, 5, job);

  for(i=0;i<5;i++) {
   	convolve_in_x(smooth_in_x[i],intpic[i],p_kernel,5, job);
	   convolve_in_y(smooth_in_y[i],smooth_in_x[i],p_kernel, 5, job);
  }
  convolve_in_t(It, smooth_in_y, d_kernel, 5, job);

  delete pre_in_t;
  delete pre_in_x;
  delete pre_in_y;
}



void Flow::calc_highpass_kernel(float  d_kernel[5], int filter)
{

if(filter==CENTRAL_DIV) /* 4-point central difference */
{
	d_kernel[0] = -1.0/12.0;
	d_kernel[1] =  8.0/12.0;
	d_kernel[2] =  0.0;
	d_kernel[3] = -8.0/12.0;
	d_kernel[4] =  1.0/12.0;
}
else if(filter==SIMONCELLI) /* Simoncelli */
{
	d_kernel[0] = -0.108;
	d_kernel[1] = -0.282;
	d_kernel[2] =  0.0;
	d_kernel[3] =  0.282;
	d_kernel[4] =  0.108;
}
else if(filter==HIEDELBERG) /* Hiedelberg */
{
	d_kernel[0] = -21.38/256.0;
	d_kernel[1] = -85.24/256.0;
	d_kernel[2] =  0.0;
	d_kernel[3] =  85.24/256.0;
	d_kernel[4] =  21.38/256.0;
}
else if(filter==CENTRAL_NODIV) /* 4-point central diff without division */
{
	d_kernel[0] = -1.0;
	d_kernel[1] = 8.0;
	d_kernel[2] = 0.0;
	d_kernel[3] = -8.0;
	d_kernel[4] = 1.0;
}
else { fprintf(stderr,"Fatal error: filter type not 1, 2 or 3\n"); exit(1); }
}


void Flow::convolve_in_t(float  out[PIC_X * PIC_Y], unsigned char *in[KERNEL_SIZE], float  *kernel, int kernel_size, FlowJob *job)
{
	int i,j,a;
	float term;
	unsigned char undefined;
	int half = (int) (kernel_size/2);
	int pic_x = job->image_width;
	int pic_y = job->image_height;


	for(i=job->x_start-2; i<= job->x_end+2; i++)
	  for(j=job->y_start-2; j <= job->y_end+2; j++) {
		 term = 0.0;
			
		 for(a=(-half);a<=half;a++) {
			term = term + (*(in[a+2] + i + j*pic_x) * kernel[a+2]);
		 }
		 //printf("%d %d, %f\n", i, j, term); 
		 out[i + j*pic_x] = term;
	  }
}

void Flow::convolve_in_t(float  out[PIC_X * PIC_Y], float *in[KERNEL_SIZE], float  *kernel, int kernel_size, FlowJob *job) {
   int i,j,a;
   float term;
   unsigned char undefined;
   int half = (int) (kernel_size/2);
	int pic_x = job->image_width;
	int pic_y = job->image_height;

   for(i=job->x_start-2; i<= job->x_end+2; i++)
     for(j=job->y_start-2; j <= job->y_end+2; j++) {
       term = 0.0;

       for(a=(-half);a<=half;a++) {
         term = term + (in[a+2][i + j*pic_x] * kernel[a+2]);
       }
		// printf("term is %f\n", term);
       out[i + j*pic_x] = term;
     }
}



void Flow::convolve_in_x(float *out, float  *in,float  *kernel, int kernel_size, FlowJob *job)
{
  int i,j,a;
  float term;
  unsigned char undefined;
  int half = (int) (kernel_size/2);
  int pic_x = job->image_width;
  int pic_y = job->image_height;
  
  for(i=job->x_start-2;i<=job->x_end+2;i++)
	 for(j=job->y_start-2;j<=job->y_end+2;j++) {
		term = 0.0;
		undefined = FALSE;

		for(a=(-half);a<=half;a++) {
     		term = term + (in[a+i + j*pic_x] * kernel[a+2]);
      }
  		out[i +j*pic_x] = term;
  }
}


// another version for unsigned char image buffers
void Flow::convolve_in_x(float *out, unsigned char  *in, float  *kernel, int kernel_size, FlowJob *job)
{
  int i,j,a;
  float term;
  unsigned char undefined;
  int half = (int) (kernel_size/2);
  int pic_x = job->image_width;
  int pic_y = job->image_height;
  
  for(i=job->x_start-2;i<=job->x_end+2;i++)
    for(j=job->y_start-2;j<=job->y_end+2;j++) {
      term = 0.0;
      undefined = FALSE;
      for(a=(-half);a<=half;a++) {
         term = term + (in[a+i + j*pic_x] * kernel[a+2]);
      }
      out[i +j*pic_x] = term;
  }
}


void Flow::convolve_in_y(float *out, float  *in,float *kernel, int kernel_size, FlowJob *job)
{

  int i,j,a;
  float term;
  unsigned char undefined;
  int half = (int) (kernel_size/2);
  int pic_x = job->image_width;
  int pic_y = job->image_height;
  
  for(i=job->x_start-2;i<=job->x_end+2;i++)
    for(j=job->y_start-2;j<=job->y_end+2;j++) {
      term = 0.0;
      undefined = FALSE;

      for(a=(-half);a<=half;a++) {
         term = term + (in[i*pic_y + a+j] * kernel[a+2]);
      }
      out[i + j*pic_x] = term;
  }
}
   					


/*********************************************************************/
/* Assigns a 1d float array with the values                          */
/* used to smooth the images. The values have been taken from the    */
/* paper written by Eero Simoncelli titled "Design of multi-deimen-  */
/* sional derivative filters" in ICIP 94                             */
/*********************************************************************/

void Flow::calc_lowpass_kernel(float  p_kernel[5], int filter)
{

	if(filter==NO_FILTER) {
		p_kernel[0] = 0.0;
		p_kernel[1] = 0.0;
		p_kernel[2] = 1.0;
		p_kernel[3] = 0.0;
		p_kernel[4] = 0.0;
	} else if(filter==SIMONCELLI) {
		p_kernel[0] =  0.036;
		p_kernel[1] =  0.249;
		p_kernel[2] =  0.431;
		p_kernel[3] =  0.249;
		p_kernel[4] =  0.036;
	} else if(filter==HIEDELBERG) {
		p_kernel[0] = 5.91/256.0;
		p_kernel[1] = 61.81/256.0;
		p_kernel[2] = 120.46/256.0;
		p_kernel[3] = 61.81/256.0;
		p_kernel[4] = 5.91/256.0;
	} else { fprintf(stderr, "Fatal error: filter type not 1, 2 or 3\n");
	 	 exit(1); }
}


// run this once before running first optical flow calculation
void Flow::setup_lucas_optical_flow()
{
	 int i, j;
	  MAX_RAW_MAG = -HUGE;
	  
	  // Compute weights 
	  coeff[0] = coeff[4] = 0.0625;
	  coeff[1] = coeff[3] = 0.25;
	  coeff[2] = 0.375;

	  for(i=0;i<5;i++) 
		  for(j=0;j<5;j++)
        		weight[i][j] = coeff[i]*coeff[j];
}
	

// run this prior to running jobs over a frame buffer
void Flow::init_lucas_optical_flow()
{
	int i, j;	
   full_count = 0;
	norm_count1 = norm_count2 = 0;
	no_count = 0;
	eigen_count = 0;
	no_swaps = 0;

	for(i=0;i<PIC_X;i++)
		for(j=0;j<PIC_Y;j++) {
        	full_vels[i][j][0] = full_vels[i][j][1] = 0; // FLOW_NO_VALUE;
      	norm_vels1[i][j][0] = norm_vels1[i][j][1] = 0; //FLOW_NO_VALUE;
        	norm_vels2[i][j][0] = norm_vels2[i][j][1] = 0; //FLOW_NO_VALUE;
        	Imag[i][j] = E[i][j] = 0.0;
		}
}


// lucas and kanade optical flow.  Performs Simoncelli filtering prior to 
// flow computation - one call does all.
void Flow::calc_lucas_optical_flow(ImageStack<ImageOf<PixelMono> > *is, 
					 							FlowJob *job, ImageOf<PixelFloat> *u_vels,
											   ImageOf<PixelFloat> *v_vels)
{
	float mag,M[2][2],MI[2][2],B[2],denominator;
	float eigenvalues[2],eigenvectors[2][2],length1,length2;
	float angle,temp1,v1,v2;
	float sigma,sigma2,sigmap,temp,diff1[2],diff2[2];
	int i,j,k,l,nrot;
	float eigenvalues2[2],eigenvectors2[2][2],v[2];
	int mag_zero;
	float tau_D = 0;
	int x_start = job->x_start;
	int x_end = job->x_end;
	int y_start = job->y_start;
	int y_end = job->y_end;
	int vector_skip = job->vector_skip;
   int flag = TRUE;	
	pic_x = job->image_width;
	pic_y = job->image_height;
	max_images = 5;

	//printf("computing lucas optical flow\n");
   init_lucas_optical_flow();
	tau_D	= job->tau_D;

	u_vels->zero();
	v_vels->zero();

	load_frames(is, max_images, pic_x, pic_y);
	// apply spatio-temporal filtering to obtain image gradients Ix, Iy, and It
	apply_simoncelli_filter(job);

	mag_zero = 0;
	// values as specified in Simoncelli, Adelson and Heeger, page 313  */
	sigma = 0.08;
	sigma2 = 1.0;
	sigmap = 2.0;


	for(i=x_start;i<=x_end;i+=vector_skip)
		for(j=y_start;j<=y_end;j+=vector_skip) {
        M[0][0] = M[1][1] = M[0][1] = M[1][0] = 0.0;
        B[0] = B[1] = 0.0;
        mag = 1.0;
        /* Compute on 5*5 neighbourhood */
        for(k=(-2);k<=2;k++)
        for(l=(-2);l<=2;l++) {
				int ind = i+k+(j+l)*pic_x;
            if(flag==TRUE) mag = sigma*(Ix[ind]*Ix[ind]+
                                        Iy[ind]*Iy[ind])+sigma2;
            M[0][0] = M[0][0] + weight[k+2][l+2]*(Ix[ind]*Ix[ind])/mag;
            M[1][1] = M[1][1] + weight[k+2][l+2]*(Iy[ind]*Iy[ind])/mag;
            M[0][1] = M[0][1] + weight[k+2][l+2]*(Ix[ind]*Iy[ind])/mag;
            B[0] = B[0] + weight[k+2][l+2]*(Ix[ind]*It[ind])/mag;
            B[1] = B[1] + weight[k+2][l+2]*(Iy[ind]*It[ind])/mag;
			//	printf("B[0]=%f, B[1]=%f\n", B[0], B[1]);
        }
        M[1][0] = M[0][1]; /* The M array is symmetric */
        if(flag==TRUE) {
           	M[0][0] = M[0][0] + 1.0/sigmap;
            M[1][1] = M[1][1] + 1.0/sigmap;
        }
        /* Invert 2*2 matrix */
        denominator = M[0][0]*M[1][1]-M[1][0]*M[0][1]; /* The determinant of M */
        MI[0][0] = M[1][1]/denominator;
        MI[0][1] = -M[0][1]/denominator;
        MI[1][0] = -M[1][0]/denominator;
        MI[1][1] = M[0][0]/denominator;

		  /*printf("MI[0][0]=%f, MI[0][1]=%f, MI[1][0]=%f, MI[1][1]=%f\n", 
								MI[0][0], MI[0][1], MI[1][0], MI[1][1]);
		  printf("B[0]=%f, B[1]=%f\n", B[0], B[1]);*/

		  if(i==500 && j==250) {
          	eigenvalues2[0] = 0.5*((M[0][0]+M[1][1]) - sqrt(
												(M[0][0]-M[1][1])*(M[0][0]-M[1][1])
												+4.0*M[0][1]*M[1][0]));
            eigenvalues2[1] = 0.5*((M[0][0]+M[1][1]) + sqrt(
												(M[0][0]-M[1][1])*(M[0][0]-M[1][1])
												+4.0*M[0][1]*M[1][0]));
            angle = atan2(eigenvalues2[0]-M[0][0],M[0][1]);
            eigenvectors2[0][0] = -cos(angle);
            eigenvectors2[1][0] = -sin(angle);
            eigenvectors2[0][1] = -sin(angle);
            eigenvectors2[1][1] =  cos(angle);

        }

        jacobi(M,2,eigenvalues,eigenvectors,&nrot);
        if(check_eigen_calc(M,eigenvalues,eigenvectors,nrot,diff1,diff2,
                &length1,&length2,&angle)==FALSE) {
				eigen_count++;
		  } else {
        	/* Sort the eigenvalues and the corresponding eigenvectors */
        	/* Most likely, already ordered                            */
        		if(eigenvalues[0] > eigenvalues[1]){ /* Largest eigenvalue first */
                /* swap eigenvalues */
                temp = eigenvalues[0];
                eigenvalues[0] = eigenvalues[1];
                eigenvalues[1] = temp;
                /* swap eigenvector components */
                temp = eigenvectors[0][0];
                eigenvectors[0][0] = eigenvectors[0][1];
                eigenvectors[0][1] = temp;
                temp = eigenvectors[1][0];
                eigenvectors[1][0] = eigenvectors[1][1];
                eigenvectors[1][1] = temp;
                no_swaps++;
            }
        		/* Full velocity if spread of M is small */
        		if(eigenvalues[0] >= tau_D && eigenvalues[1] >= tau_D) {
        			 if(denominator > 0.000000001) {
                	full_vels[i][j][1] =   -(v1= -(MI[0][0]*B[0]+MI[0][1]*B[1]));
                	full_vels[i][j][0] =   (v2= -(MI[1][0]*B[0]+MI[1][1]*B[1]));
						/*printf("fullvels[%d][%d]: u: %f, v: %f\n", i, j, 
											 full_vels[i][j][0], full_vels[i][j][1]);*/
                	/* Compute the residual */
                	for(k=(-2);k<=2;k++)
                		for(l=(-2);l<=2;l++) {
								int ind = i+k+(j+l)*pic_x;
                        temp1 = weight[k+2][l+2]*
                              (Ix[ind]*v1+ Iy[ind]*v2+It[ind]);
                        E[i][j] += (temp1*temp1);
                  	}
                	full_count++;
                } else {
							full_vels[i][j][0] = full_vels[i][j][1] = FLOW_NO_VALUE; 
					 }
        }
			/* Normal velocity if spread of M is small in one direction only */
        else if(eigenvalues[1] >= tau_D && fabs(denominator) > 0.00000001){
        		/* Normal velocity if spread of MI is small in one direction only */
            /* Project v onto that direction */
            v[0] = -(MI[0][0]*B[0]+MI[0][1]*B[1]);
            v[1] = -(MI[1][0]*B[0]+MI[1][1]*B[1]);
            norm_vels1[i][j][1] = -(v[0]*eigenvectors[0][0]+
           		v[1]*eigenvectors[1][0])*eigenvectors[0][0];
            norm_vels1[i][j][0] = (v[0]*eigenvectors[0][0]+
               v[1]*eigenvectors[1][0])*eigenvectors[1][0];
            norm_count1++;
        } else { 
			   full_vels[i][j][0] = full_vels[i][j][1] = FLOW_NO_VALUE;
				no_count++;
        }
    	}

    	/* Compute type 2 (raw) normal velocity */
    	mag = (Ix[i+j*pic_x]*Ix[i+j*pic_x] + Iy[i+j*pic_x]*Iy[i+j*pic_x]);
    	Imag[i][j] = sqrt(mag);
    	if(Imag[i][j] > MAX_RAW_MAG) MAX_RAW_MAG = Imag[i][j];
    	if(Imag[i][j] > RAW_MAG) {
        norm_vels2[i][j][1] =   It[i+j*pic_x]*Ix[i+j*pic_x]/mag;
        norm_vels2[i][j][0] =  -It[i+j*pic_x]*Iy[i+j*pic_x]/mag;
        norm_count2++;
    	} else mag_zero++;
 	}

	tidy_lucas_optical_flow(job, u_vels, v_vels);
	//printf("Flow computed - have a nice day!\n");
}


// run after all jobs have been completed on frame buffer
void Flow::tidy_lucas_optical_flow(FlowJob *job, 
					 							ImageOf<PixelFloat> *u_vels,
											  	ImageOf<PixelFloat> *v_vels)
{
   int i, j;
	int pic_x = job->image_width, pic_y = job->image_height;
	for(i=0;i<pic_x;i++)
		for(j=0;j<pic_y;j++){
    		if(full_vels[i][j][0]!=FLOW_NO_VALUE && 
								 full_vels[i][j][1]!=FLOW_NO_VALUE) {
				/*
    			float temp = full_vels[i][j][0];
    			u_vels->pixel(i, j) = -full_vels[i][j][1];
    			v_vels->pixel(i, j) = -temp;
				*/
				u_vels->pixel(i,j) = full_vels[i][j][0];
				v_vels->pixel(i,j) = full_vels[i][j][1];
   		}
	}

	for(i=0;i<max_images;i++){
		delete intpic[i];
	}
}


/**********************************************************************/
/* Compute the dot product                                            */
/**********************************************************************/

float Flow::dot(float  a[3],float b[3])
{

return(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]);
}

/*********************************************************************/
/* Use Numerical Recipes routine to compute eigenvalues/eigenvectors */
/*********************************************************************/
int Flow::eigen(float **mat,float **evt,float *evals,short n, int a, int b)
{
    float *dd;
    float **aa;
    float **vv;
    int i,j;
    int nrot;
    short n1;

        /* adjust the pointer for numerical recipes */
        dd = evals-1;

        n1 = n+1;
        aa = a_fmatr(n1,n1);
		vv = a_fmatr(n1,n1);

    for(i=1;i<=n;i++)
    for(j=1;j<=n;j++)
           aa[i][j] = mat[i-1][j-1];

    /* Numerical recipes eigenvalue routine */
    /* dd contains the eigenvalues */
    /* vv contains the eigenvectors ev0 = vv[1][0],vv[2][0],vv[3][0],vv[4][0] */

	j_jacobi(aa,(int)n,dd,vv,&nrot);

    for(i=0;i<n;i++)
            {
            for(j=0;j<n;j++)
                evt[i][j] = vv[i+1][j+1];
            }
    for(i=0;i<n;i++)
            evals[i] = fabs(dd[i+1]);
    
    f_fmatr(vv);
    f_fmatr(aa);

    return TRUE;
}

/*****************************************************/
/* allocate a float matrix */
float** Flow::a_fmatr(short dy,short dx)
{
    short y;
    float **mat;

    mat = (float**)malloc(dy * sizeof(float*));
    if(mat == NULL) return NULL;

    mat[0] = (float*) calloc(dy*dx,sizeof(float));
    if(mat[0] == NULL) {
        free(mat);
        return NULL;
    }

    for(y=1;y<dy;y++) {
            mat[y] = mat[y-1] + dx;
    }

    return mat;
}
/******************************************************/
/* free float matrix */
void Flow::f_fmatr(float **mat)
{
    if(mat) {
        if(mat[0]) free(mat[0]);
        free(mat);
    }
}
/******************************************************/
/* allocate a float vector with subscript range v[nl..nh] */
float * Flow::vector(long nl, long nh)
{
    float *v;

    v=(float *)malloc((size_t) ((nh-nl+1+NR_END)*sizeof(float)));
    return v-nl+NR_END;
}


/* 
           NAME : vector(loc,f)
   PARAMETER(S) : loc : flow field location;
                    f : node pointer.
 
        PURPOSE : returns the vector located at loc in f.

         AUTHOR : Steven Beauchemin
             AT : University of Western Ontario
           DATE : November 7 1990
*/

void Flow::vector(int loc[2],float vect[2])

{ 
  
  if ((loc[0] != -1) && (loc[1] != -1)) {
    vect[0] = full_vels[loc[0]][loc[1]][0];
	vect[1] = full_vels[loc[0]][loc[1]][1];
  }
  else {
    vect[0] = -100.0 ;
    vect[1] = 100.0 ;
  }
 
}

/******************************************************/
/* free a float vector allocated with vector() */
void Flow::free_vector(float *v, long nl, long nh)
{
    free((FREE_ARG) (v+nl-NR_END));
}

/************************************************************************/
/* Compute all eigenvalues and eigenvectors of a real symmetric matrix  */
/* a[N][N]. On output elements of a above the disgonal are destroyed.   */
/* d[N] returns the eigenvalues of a. v[N][N] is a matrix whose columns */
/* contain, on output, the normalized eigenvectors of a. nrot returns   */
/* the number of Jacobi rotations that were required.                   */
/************************************************************************/
void Flow::jacobi(float  aa[N][N],int  n,float d[N],float v[N][N],int *nrot)
{

int j,iq,ip,i;
float thresh,theta,tau,t,sm,s,h,g,c;
float b[N],z[N],a[N][N];

if(n!=N) {fprintf(stderr, "\nFatal error: n not %d in jacobi\n",N); exit(1); }
for(ip=0;ip<n;ip++) /* Initialize to the identity matrix */
        {
        for(iq=0;iq<n;iq++) v[ip][iq] = 0.0;
        for(iq=0;iq<n;iq++) a[ip][iq] = aa[ip][iq]; /* Don't destroy aa */
        v[ip][ip] = 1.0;
        }
/* Initialize b and d to the diagonals of a */
for(ip=0;ip<n;ip++)
        {
        b[ip] = d[ip] = a[ip][ip];
        z[ip] = 0.0;
        }
*nrot = 0;
for(i=0;i<100;i++)
        {
        sm = 0.0;
        for(ip=0;ip<(n-1);ip++)
                {
                for(iq=ip+1;iq<n;iq++)
                        sm += fabs(a[ip][iq]);
                }

        /* Normal return, which relies on quadratic convergence to
           machine underflow */
        if(sm == 0.0) return;

        if(i<3) thresh=0.2*sm/(n*n); /* on the first three sweeps */
        else thresh = 0.0; /* the rest of the sweeps */

        for(ip=0;ip<(n-1);ip++)
                {
                for(iq=ip+1;iq<n;iq++)
                        {
                        g = 100.0*fabs(a[ip][iq]);
                        /* After 4 sweeps skip the rotation if the
                           off diagonal element is small */
                        if(i>3 && fabs(d[ip])+g == fabs(d[ip])
                               && fabs(d[iq])+g == fabs(d[iq])) a[ip][iq] = 0.0;
                        else if(fabs(a[ip][iq]) > thresh)
                                {
                                h = d[iq]-d[ip];
                                if(fabs(h)+g==fabs(h)) t=(a[ip][iq])/h;
                                else
                                  {
                                  theta = 0.5*h/(a[ip][iq]);
                                  t = 1.0/(fabs(theta)+sqrt(1.0+theta*theta));
                                  if(theta < 0.0) t = -t;
                                  }
                                c = 1.0/sqrt(1.0+t*t);
                                s = t*c;
                                tau = s/(1.0+c);
                                h = t*a[ip][iq];
                                z[ip] -= h;
                                z[iq] += h;
                                d[ip] -= h;
                                d[iq] += h;
                                a[ip][iq] = 0.0;
                                for(j=0;j<ip-1;j++)
                                        rotate(a,j,ip,j,iq,&h,&g,s,tau);
                                for(j=ip+1;j<iq-1;j++)
                                        rotate(a,ip,j,j,iq,&h,&g,s,tau);
                                for(j=iq+1;j<n;j++)
                                        rotate(a,ip,j,iq,j,&h,&g,s,tau);
                                for(j=0;j<n;j++)
                                        rotate(v,j,ip,j,iq,&h,&g,s,tau);
                                ++(*nrot);
                                }
                        }
                }
        for(ip=0;ip<n;ip++)
                {
                b[ip] += z[ip];
                d[ip] = b[ip];
                z[ip] = 0.0;
                }
        }
/* fprintf(stderr,"\nFatal error: too many iterations in jacobi\n"); */
}


/********************************************************************/
void Flow::j_jacobi(float **a, int n, float d[], float **v, int *nrot)
{
    int j,iq,ip,i;
    float tresh,theta,tau,t,sm,s,h,g,c,*b,*z;

    b=vector(1,n);
    z=vector(1,n);
    for (ip=1;ip<=n;ip++) {
        for (iq=1;iq<=n;iq++) v[ip][iq]=0.0;
        v[ip][ip]=1.0;
    }
    for (ip=1;ip<=n;ip++) {
        b[ip]=d[ip]=a[ip][ip];
        z[ip]=0.0;
    }
    *nrot=0;
    for (i=1;i<=50;i++) {
        sm=0.0;
        for (ip=1;ip<=n-1;ip++) {
            for (iq=ip+1;iq<=n;iq++)
                sm += fabs(a[ip][iq]);
        }
        if (sm == 0.0) {
            free_vector(z,1,n);
            free_vector(b,1,n);
            return;
        }
        if (i < 4)
            tresh=0.2*sm/(n*n);
        else
            tresh=0.0;
        for (ip=1;ip<=n-1;ip++) {
            for (iq=ip+1;iq<=n;iq++) {
                g=100.0*fabs(a[ip][iq]);
                if (i > 4 && (float)(fabs(d[ip])+g) == (float)fabs(d[ip])
                    && (float)(fabs(d[iq])+g) == (float)fabs(d[iq]))
                    a[ip][iq]=0.0;
                else if (fabs(a[ip][iq]) > tresh) {
                    h=d[iq]-d[ip];
                    if ((float)(fabs(h)+g) == (float)fabs(h))
                        t=(a[ip][iq])/h;
                    else {
                        theta=0.5*h/(a[ip][iq]);
                        t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
                        if (theta < 0.0) t = -t;
                    }
                    c=1.0/sqrt(1+t*t);
                    s=t*c;
                    tau=s/(1.0+c);
                    h=t*a[ip][iq];
                    z[ip] -= h;
                    z[iq] += h;
                    d[ip] -= h;
                    d[iq] += h;
                    a[ip][iq]=0.0;
                    for (j=1;j<=ip-1;j++) {
                        ROTATE(a,j,ip,j,iq)
                    }
                    for (j=ip+1;j<=iq-1;j++) {
                        ROTATE(a,ip,j,j,iq)
                    }
                    for (j=iq+1;j<=n;j++) {
                        ROTATE(a,ip,j,iq,j)
                    }
                    for (j=1;j<=n;j++) {
                        ROTATE(v,j,ip,j,iq)
                    }
                    ++(*nrot);
                }
            }
        }
        for (ip=1;ip<=n;ip++) {
            b[ip] += z[ip];
            d[ip]=b[ip];
            z[ip]=0.0;
        }
    }
    fprintf(stderr, "Too many iterations in routine jacobi");
}
#undef ROTATE

float Flow::too_big(float  v[3])
{

if(sqrt(v[0]*v[0]+v[1]*v[1]) > BIG_THRESHOLD) return(TRUE);
return(FALSE);
}

float Flow::too_big3(float  v[3])
{

if(sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]) > BIG_THRESHOLD) return(TRUE);
return(FALSE);
}

/************************************************************************/
/* Function to compute the cross product of two vectors.                */
/************************************************************************/

void Flow::cross_product(float  a[3],float b[3],float c[3])
{

c[0] = a[1]*b[2]-a[2]*b[1];
c[1] = a[2]*b[0]-a[0]*b[2];
c[2] = a[0]*b[1]-a[1]*b[0];
}

/*********************************************************/
/* Are floats a and b about the same?                    */
/*********************************************************/

int Flow::about(float  a,float b)
{

if(fabs(a-b) < SMALL) return(TRUE);
return(FALSE);
}

/*********************************************************************/
/* Check eigenvector and eigenvalue computation for 2*2 matrix       */
/*********************************************************************/

int Flow::check_eigen_calc(float  mm[N][N],float d[N],float v[N][N],int  n,float diff1[N],float diff2[N],float *length1,float *length2,float *angle)
{

	int status;

	status = TRUE;
	/* Compute angle between two eigenvectors - should be orthogonal */
	(*angle)=acos((v[0][0]*v[0][1]+v[1][0]*v[1][1])/
         (sqrt(v[0][0]*v[0][0]+v[1][0]*v[1][0])*
          sqrt(v[0][1]*v[0][1]+v[1][1]*v[1][1])))*180.0/3.1415926535;
	if((*angle) < 89.5 && (*angle) > 90.5) {
        status = FALSE;
   }

	/* Eigenvector test */
	diff1[0] = mm[0][0]*v[0][0]+mm[0][1]*v[1][0];
	diff1[1] = mm[1][0]*v[0][0]+mm[1][1]*v[1][0];
	diff1[0] = diff1[0] - d[0]*v[0][0];
	diff1[1] = diff1[1] - d[0]*v[1][0];
	if(((*length1)=sqrt(diff1[0]*diff1[0]+diff1[1]*diff1[1])) > 0.1) {
        status = FALSE;
   }
	diff2[0] = mm[0][0]*v[0][1]+mm[0][1]*v[1][1];
	diff2[1] = mm[1][0]*v[0][1]+mm[1][1]*v[1][1];
	diff2[0] = diff2[0] - d[1]*v[0][1];
	diff2[1] = diff2[1] - d[1]*v[1][1];

	if(((*length2)=sqrt(diff2[0]*diff2[0]+diff2[1]*diff2[1])) > 0.1) {
        status = FALSE;
   }

	if(n > 50) {
        status = FALSE;
   }

	return(status);
}

/*********************************************************************/
/* Do rotations required by Jacobi Transformation                    */
/*********************************************************************/

void Flow::rotate(float  a[N][N],int  i,int j,int k,int l,float  *h,float *g,float s,float tau)
{

(*g) = a[i][j];
(*h) = a[k][l];
a[i][j] = (*g)-s*((*h)+(*g)*tau);
a[k][l] = (*h)+s*((*g)-(*h)*tau);
}




void Flow::calc_raw_normals(int offset, float trsh)
{
	float mag;
	int i, j;

	for(i=0;i<PIC_X;i++)
		for(j=0;j<PIC_Y;j++)
			full_vels[i][j][0] = full_vels[i][j][1] = FLOW_NO_VALUE;
			

	for(i=offset; i<pic_x-offset; i++)
		for(j=offset;j<pic_y-offset; j++) {
			mag = Ix[i +j*pic_x] * Ix[i+j*pic_x] + Iy[i+j*pic_x] * Iy[i+j*pic_x];
			if(sqrt(mag) >= trsh)
			{
				full_vels[i][j][0] = (Ix[i+j*pic_x] * -It[i+j*pic_x])/mag;
				full_vels[i][j][1] = (Iy[i+j*pic_x] * -It[i+j*pic_x])/mag;
			}
		}
}


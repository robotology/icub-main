#include "public.h"
#include "basicOpenCV.h"


IplImage *BasicOpenCV::Rotate(IplImage *src, float angle)
{
	IplImage* dst = cvCloneImage( src );

	cvNamedWindow( "src", 1 );
	cvShowImage( "src", src );

	float m[6];
	CvMat M = cvMat( 2, 3, CV_32F, m );
	int w = src->width;
	int h = src->height;

	float factor = 1.f;//resizing factor
	m[0] = (float)(factor*cosf(-angle*2*(float)CV_PI/180.f));
	m[1] = (float)(factor*sinf(-angle*2*(float)CV_PI/180.f));
	m[2] = w*0.5f;
	m[3] = -m[1];
	m[4] = m[0];
	m[5] = h*0.5f;

	cvGetQuadrangleSubPix( src, dst, &M);

	return dst;
}

void BasicOpenCV::integralImage(const IplImage *image, IplImage **intimage)
{
	u32 W = image->width, H = image->height;

	IMKILL((*intimage));
	if((*intimage)!=0){
		cvReleaseImage(&(*intimage));
		(*intimage)=0;
	}

	// Create resulting images
	(*intimage) = cvCreateImage(cvGetSize(image),IPL_DEPTH_32S,image->nChannels);
	(*intimage)->origin = image->origin;

	u8 *i_m = (u8 *)(image->imageData);
	u32 *ii_m = (u32 *)((*intimage)->imageData);

	ii_m[0] = i_m[0];

	// Create the first row of the integral image
	for (u32 x = 1; x < W; x++)
	{
		ii_m[x] = ii_m[x-1] + i_m[x];
	}

	// Compute each other row/column
	for (u32 y = 1, Y = W, YY=0; y < H; y++, Y+=W, YY+=W)
	{
		// Keep track of the row sum
		u32 r = 0, rs = 0;

		for (u32 x = 0; x < W; x++)
		{
			r += i_m[Y + x];
			ii_m[Y + x] = ii_m[YY + x] + r;
		}
	}
}


u32 BasicOpenCV::GetSum(IplImage *integral, CvRect rect){
	u32 *data = (u32 *)integral->imageData;
	u32 w = integral->width;
	register u32 a = rect.y*w + rect.x;
	register u32 b = rect.y*w + rect.x + rect.width;
	register u32 c = (rect.y+rect.height)*w + rect.x;
	register u32 d = (rect.y+rect.height)*w + rect.x + rect.width;
	return data[d] - data[b] - data[c] + data[a];
}

void BasicOpenCV::cvCopyFlipped(IplImage *src, IplImage *dst)
{
	assert(src);
	assert(dst);
	assert(src->width == dst->width && src->height == dst->height);
	if(src->origin != IPL_ORIGIN_TL)
		cvFlip(src, dst);
	else
		cvCopy(src,dst);
	dst->origin = IPL_ORIGIN_TL;
}

void BasicOpenCV::DisplayHueSatHist(IplImage* src)
{
	if(!src) return;
	IplImage* h_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(src), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };
	IplImage* hsv = cvCreateImage( cvGetSize(src), 8, 3 );
	u32 h_bins = 30, s_bins = 32;
	s32 hist_size[] = {h_bins, s_bins};
	f32 h_ranges[] = { 0, 180 }; /* hue varies from 0 (~0red) to 180 (~360red again) */
	f32 s_ranges[] = { 0, 255 }; /* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
	f32* ranges[] = { h_ranges, s_ranges };
	s32 scale = 10;
	IplImage* hist_img = cvCreateImage( cvSize(h_bins*scale,s_bins*scale), 8, 3 );
	CvHistogram* hist;
	f32 max_value = 0;

	cvCvtColor( src, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
	hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	cvCalcHist( planes, hist, 0, 0 );
	cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
	cvZero( hist_img );

	FOR(h, h_bins){
		FOR(s, s_bins){
			f32 bin_val = cvQueryHistValue_2D( hist, h, s );
			s32 intensity = cvRound(bin_val*255/max_value);
			cvRectangle( hist_img, cvPoint( h*scale, s*scale ),
				cvPoint( (h+1)*scale - 1, (s+1)*scale - 1),
				CV_RGB(intensity,intensity,intensity), /* draw a grayscale histogram.
													   if you have idea how to do it
													   nicer let us know */
													   CV_FILLED );
		}
	}
	cvNamedWindow( "H-S Histogram", 1 );
	cvShowImage( "H-S Histogram", hist_img );
}

void BasicOpenCV::RGB2NCC(IplImage *image, IplImage *red, IplImage *green)
{
	u32 w = image->width;
	u32 h = image->height;

	u32 intensity, rM, gM;
	u8 *pix = (u8 *)image->imageData;
	FOR(i, w*h){
		intensity = pix[i*3] + pix[i*3+1] + pix[i*3+2];
		if (intensity > 0){
			if (intensity < 60){
				if (pix[i*3] + pix[i*3+1] < 10 || pix[i*3] + pix[i*3+2] < 10){
					rM = gM = 0;
				}
				else{
					rM = pix[i*3+2] *255 / intensity;
					gM = pix[i*3+1] *255 / intensity;
				}
			}
			else{
				rM = pix[i*3+2] *255 / intensity;
				gM = pix[i*3+1] *255 / intensity;
			}
		}
		else{
			rM = gM = 0;
		}
		red->imageData[i] = (u8)(rM);
		green->imageData[i] = (u8)(gM);
	}
}

void BasicOpenCV::BinaryMedian(IplImage *src, IplImage *dst)
{
	u32 w = src->width;
	u32 h = src->height;

	FOR(j, h){
		dst->imageData[j*w] = 0;
		dst->imageData[j*w + w-1] = 0;
	}

	FOR(j, w){
		dst->imageData[j] = 0;
		dst->imageData[(h-1)*w + j] = 0;
	}

	for (u32 j=1; j < h-1; j++)
	{
		for (u32 i=1; i < w-1; i++)
		{
			u8 c = 0;
			c =(((u8)src->imageData[(j-1)*w + i] >0) +
				((u8)src->imageData[j*w + i-1]   >0) +
				((u8)src->imageData[j*w + i]     >0) +
				((u8)src->imageData[j*w + i+1]   >0) +
				((u8)src->imageData[(j+1)*w + i] >0) );
			dst->imageData[j*w + i] = (c > 2)*255;
		}
	}
}

void BasicOpenCV::RemoveNoise(IplImage * src)
{
	//get the size of input_image (src)
	CvSize sz = cvSize( src->width & -2, src->height & -2 ); 

	//create  temp-image
	IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), src->depth, src->nChannels ); 

	cvPyrDown( src, pyr, 7);	//pyr DOWN
	cvPyrUp( pyr, src, 7);		//and UP
	cvReleaseImage(&pyr);		//release temp
	pyr = NULL;
}

IplImage *BasicOpenCV::Deinterlace(IplImage *image)
{
	if(!image) return NULL;
	IplImage *fields = cvCreateImage(cvGetSize(image), 8, 3);
	fields->origin = image->origin;
	u32 height = image->height;
	u32 width = image->width;
	u32 step = image->widthStep;
	for(u32 i = 0; i < height; i += 2){
		FOR(j, width){
			// first field
			rgb(fields,(i>>1)*step + j*3  ) = rgb(image,i*step + j*3  );
			rgb(fields,(i>>1)*step + j*3+1) = rgb(image,i*step + j*3+1);
			rgb(fields,(i>>1)*step + j*3+2) = rgb(image,i*step + j*3+2);

			// second field
			rgb(fields,(((i>>1)+(height>>1))*step + j*3  )) = rgb(image,(i+1)*step + j*3  );
			rgb(fields,(((i>>1)+(height>>1))*step + j*3+1)) = rgb(image,(i+1)*step + j*3+1);
			rgb(fields,(((i>>1)+(height>>1))*step + j*3+2)) = rgb(image,(i+1)*step + j*3+2);
		}
	}
	return fields;
}

IplImage *BasicOpenCV::GetField(IplImage *image, u32 field)
{
	if(!image) return NULL;
	u32 height = image->height;
	u32 width = image->width;
	u32 step = image->widthStep;
	IplImage *fields = cvCreateImage(cvSize(width, height>>1), 8, 3);
	fields->origin = image->origin;
	for(u32 i = 0; i < height; i += 2){
		i += field ? 1 : 0;
		for(u32 j=0; j<width; j++){
			rgb(fields,(i>>1)*step + j*3  ) = rgb(image,i*step + j*3  );
			rgb(fields,(i>>1)*step + j*3+1) = rgb(image,i*step + j*3+1);
			rgb(fields,(i>>1)*step + j*3+2) = rgb(image,i*step + j*3+2);
		}
	}
	return fields;
}

IplImage *BasicOpenCV::Half2Full(IplImage *image)
{
	IplImage *newImage = cvCreateImage(cvSize(image->width, image->height*2),image->depth, image->nChannels);
	newImage->origin = image->origin;
	u32 step = newImage->widthStep;
	u32 chan = newImage->nChannels;
	FOR(i, (u32)newImage->height){
		FOR(j, (u32)newImage->width){
			rgb(newImage,i*step +j*chan  ) = rgb(image, (i>>1)*step + j*chan  );
			rgb(newImage,i*step +j*chan+1) = rgb(image, (i>>1)*step + j*chan+1);
			rgb(newImage,i*step +j*chan+2) = rgb(image, (i>>1)*step + j*chan+2);
		}
	}
	return newImage;
}

void BasicOpenCV::Half2Full(IplImage *src, IplImage *dst)
{
	dst->origin = src->origin;
	u32 step = dst->widthStep;
	u32 chan = dst->nChannels;
	FOR(i, (u32)dst->height){
		FOR(j, (u32)dst->width){
			rgb(dst,i*step +j*chan  ) = rgb(src, (i>>1)*step + j*chan  );
			rgb(dst,i*step +j*chan+1) = rgb(src, (i>>1)*step + j*chan+1);
			rgb(dst,i*step +j*chan+2) = rgb(src, (i>>1)*step + j*chan+2);
		}
	}
}

IplImage *BasicOpenCV::Half2Demi(IplImage *image)
{
	IplImage *newImage = cvCreateImage(cvSize(image->width/2, image->height),image->depth, image->nChannels);
	newImage->origin = image->origin;
	u32 step = newImage->widthStep;
	u32 stepSrc = image->widthStep;
	u32 chan = newImage->nChannels;
	FOR(i, (u32)newImage->height){
		FOR(j, (u32)newImage->width){
			rgb(newImage,i*step +j*chan  ) = rgb(image, i*stepSrc + (j<<1)*chan  );
			rgb(newImage,i*step +j*chan+1) = rgb(image, i*stepSrc + (j<<1)*chan+1);
			rgb(newImage,i*step +j*chan+2) = rgb(image, i*stepSrc + (j<<1)*chan+2);
		}
	}
	return newImage;
}

void BasicOpenCV::Half2Demi(IplImage *src, IplImage *dst)
{
	dst->origin = src->origin;
	u32 step = dst->widthStep;
	u32 stepSrc = src->widthStep;
	u32 chan = dst->nChannels;
	FOR(i, (u32)dst->height){
		FOR(j, (u32)dst->width){
			rgb(dst,i*step +j*chan  ) = rgb(src, i*stepSrc + (j<<1)*chan  );
			rgb(dst,i*step +j*chan+1) = rgb(src, i*stepSrc + (j<<1)*chan+1);
			rgb(dst,i*step +j*chan+2) = rgb(src, i*stepSrc + (j<<1)*chan+2);
		}
	}
}

IplImage *BasicOpenCV::Half(IplImage *src)
{
	CvSize size = cvGetSize(src);
	size.width /= 2;
	size.height /= 2;
	IplImage *dst = cvCreateImage(size, src->depth, src->nChannels);
	dst->origin = src->origin;
	cvResize(src, dst, CV_INTER_CUBIC);
	return dst;
}

void BasicOpenCV::Half(IplImage **src)
{
	IplImage *img = (*src);
	CvSize size = cvGetSize(img);
	size.width /= 2;
	size.height /= 2;
	IplImage *dst = cvCreateImage(size, img->depth, img->nChannels);
	dst->origin = img->origin;
	cvResize(img, dst, CV_INTER_CUBIC);
	IMKILL(img);
	(*src) = dst;
}

IplImage *BasicOpenCV::Resize(IplImage *src, CvSize size)
{
	IplImage *dst = cvCreateImage(size, src->depth, src->nChannels);
	dst->origin = src->origin;
	cvResize(src, dst, CV_INTER_CUBIC);
	return dst;
}

void BasicOpenCV::Resize(IplImage **src,CvSize size)
{
	IplImage *img = (*src);
	if(!img){
		(*src) = cvCreateImage(size, 8, 1);
		return;
	}
	if(size.width == img->width && size.height == img->height) return;
	IplImage *dst = cvCreateImage(size,img->depth,img->nChannels);
	dst->origin = img->origin;
	cvResize(img,dst,CV_INTER_CUBIC);
	IMKILL(img);
	(*src) = dst;
}

// 1 = red, 2 = blue, 
void BasicOpenCV::ChannelSubtraction(IplImage *src , IplImage *dst, u32 first, u32 second)
{
	bool bYellow = first == 4 || second == 4;
	IplImage *channels[5];
	FOR(i,5)
	{
		channels[i] = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F, 1);
	}
	cvSplit(src, channels[0], channels[1], channels[2], channels[3]);
	if(bYellow)
	{
		cvAddWeighted(channels[0],0.5,channels[1],0.5,0,channels[4]);
	}
	cvSub(channels[first], channels[second], dst);
	FOR(i,5)
	{
		cvReleaseImage(&channels[i]); channels[i] = NULL;
	}
}

void BasicOpenCV::Divide(IplImage *img1, IplImage *img2 )
{
	if(!img1 || !img2) return;
	cvDiv(img1,img2,img1);
}

IplImage *BasicOpenCV::Crop(IplImage *image,CvRect selection)
{
	if(!image) return NULL;
	IplImage *dest = cvCreateImage(cvSize(selection.width, selection.height),image->depth,image->nChannels);
	dest->origin = image->origin;
	ROI(image, selection);
	cvCopy(image, dest);
	unROI(image);
	return dest;
}

int BasicOpenCV::otsuThreshold(CvMat* prob, CvHistogram* hist)
{
	cvCalcHist((IplImage**)&prob, hist, 0, NULL); 

	/* 
	Instead of calling cvNormalizeHist(hist, 1.0), we can :
	in the first loop, compute the sum of pi and 
	in the second loop, normalize the pi before doing calculation.
	This save us some times.
	*/

	float w0 = 0;
	float w1 = 1;
	float mu0 = 0;
	float mu1 = 0;

	CvMat mat;
	cvGetMat(hist->bins, &mat, 0, 1);

	float sum = 0; /* used to normalized the histogram */

	float* ptrHist = mat.data.fl;
	int i;
	for (i=1; i<=256; i++) {
		mu1 = mu1 + i* *ptrHist;
		sum += *ptrHist;
		ptrHist++;
	}
	mu1 /= sum;

	int threshold = 0;
	float sigma;
	float sigmaMax = 0;
	ptrHist = mat.data.fl;
	float pi;
	for (i=1; i<=256; i++) {
		pi = *ptrHist;
		pi /= sum; /* for normalization */

		mu0 = mu0*w0;
		mu1 = mu1*w1;
		w0  = w0 + pi;
		w1  = w1 - pi;
		mu0 = (mu0 + (i* pi)) /w0;
		mu1 = (mu1 - (i* pi)) /w1;

		sigma = (w0*w1*(mu1-mu0)*(mu1-mu0));

		if (sigma > sigmaMax){
			threshold = i;
			sigmaMax = sigma;
		}
		ptrHist++;
	}

	return threshold;
}

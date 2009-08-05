/*!
 * \file basicOpenCV.h
 * \author Basilio Noris
 * \date 25-11-07
 */
#ifndef _BASICOPENCV_H_
#define _BASICOPENCV_H_

#define ROI(a,b) cvSetImageROI(a,b)
#define unROI(a) cvResetImageROI(a)

/* computes the point corresponding to a certain angle on an input image */
#define calc_point(img, angle)                                      \
    cvPoint( cvRound(img->width/2 + img->width/3*cos(angle)),		\
                cvRound(img->height/2 - img->width/3*sin(angle)))

/* plot points */
#define draw_cross(img, center, color, d )										\
    cvLine( img, cvPoint( center.x - d, center.y - d ),							\
                    cvPoint( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
    cvLine( img, cvPoint( center.x + d, center.y - d ),							\
                    cvPoint( center.x - d, center.y + d ), color, 1, CV_AA, 0 )
/* draws a plus */
#define draw_plus(img, center, color, d )					\
	cvLine( img, cvPoint( center.x - d, center.y),			\
		cvPoint( center.x + d, center.y), color, 1, 0 );	\
	cvLine( img, cvPoint( center.x, center.y - d ),			\
		cvPoint( center.x, center.y + d ), color, 1, 0 )

class BasicOpenCV
{
public:
	static IplImage *Rotate(IplImage *src, float angle);
	static void integralImage(const IplImage *image, IplImage **intimage);
	static u32 GetSum(IplImage *integral, CvRect rect);
	static void cvCopyFlipped(IplImage *src, IplImage *dst);
	static void DisplayHueSatHist(IplImage* src);
	static void RGB2NCC(IplImage *image, IplImage *red, IplImage *green);
	static void BinaryMedian(IplImage *src, IplImage *dst);
	static void RemoveNoise(IplImage * src);
	static IplImage *Deinterlace(IplImage *image);
	static IplImage *GetField(IplImage *image, u32 field);
	static IplImage *Half2Full(IplImage *image);
	static void Half2Full(IplImage *src, IplImage *dst);
	static IplImage *Half2Demi(IplImage *image);
	static void Half2Demi(IplImage *src, IplImage *dst);
	static IplImage *Half(IplImage *src);
	static void Half(IplImage **src);
	static IplImage *Resize(IplImage *src, CvSize size);
	static void Resize(IplImage **src,CvSize size);
	static void ChannelSubtraction(IplImage *src , IplImage *dst, u32 first, u32 second);
	static void Divide(IplImage *img1, IplImage *img2 );
	static IplImage *Crop(IplImage *image,CvRect selection);
	static int otsuThreshold(CvMat* prob, CvHistogram* hist);
};

typedef BasicOpenCV CV;

namespace BasicML
{
	//returns the index corresponding to the minimal value 
	static int smallest(s32 values[], u32 length){
		s32 minValue = values[0];
		u32 minIndex = 0;
		FOR(i, length){
			if (values[i] < minValue){
				minIndex = i;
				minValue = values[i];
			}
		}
		return minIndex;
	}

	// returns the mean for the specified cluster
	static CvPoint mean(CvPoint3D32f points[], int length, int cluster){
		CvPoint mean = cvPoint(0,0);
		int nbPointInCluster = 0;
		for (int i=0; i<length; i++){
			if((int)points[i].z == cluster){
				mean.x += (int)points[i].x;
				mean.y += (int)points[i].y;
				nbPointInCluster++;
			}
		}

		// if a cluster has one or more point(s), change the cluster's mean
		if (nbPointInCluster){
			mean.x /= nbPointInCluster;
			mean.y /= nbPointInCluster;
		}
		return mean;
	}

	/**
	  * performs the K-mean clustering algorithm
	  *
	  * @param points[] : each element of this array is a 3-uple (x,y,c), where
	  *                   x,y are the coordinate of the point, and c the cluster
	  *                   the point belongs to.
	  *                   the c component can be initialized randomly and will be changed by the algo
	  * @param numberOfPoints : number of point to be clustered (i.e number of points
	  *                         corresponding to skin color (in our case)
	  * @param nbCluster :      number of clusters. Min 1, Max 3
	  * @param xmax :           max value for the x coordinate (i.e image's width)
	  * @param ymax :           max value for the y coordinate (i.e image's height)
	  *
	  */
	#define squareNorm(p1, p2) ((int)p1.x-p2.x)*((int)p1.x-p2.x) + ((int)p1.y-p2.y)*((int)p1.y-p2.y)

	static CvPoint *KmeansClustering(CvPoint3D32f points[],u32 length, u32 nbCluster, s32 xmax, s32 ymax){
		const s32 maxInt = 2^32-1;

		// contains the means for each cluster
		CvPoint *means = new CvPoint[nbCluster];

		// mean of the current cluster
		CvPoint currentMean;

		// has one point moved from one cluster to another ?
		bool atLeastOneChange=true;

		// contains the "distance" (i.e d*d) from the current point to each cluster
		s32 *currentSquareDistance = new s32[nbCluster];


		// Random number generation for initial means of clusters
		srand((u32)(cvGetTickCount()/cvGetTickFrequency()/10000000.0));

		//initialize the means randomly
		
		FOR(i, nbCluster){
			currentMean.x=rand()%xmax;
			currentMean.y=rand()%ymax;
			means[i]=(currentMean);
		}

		FOR(k, nbCluster){
			currentSquareDistance[k]=maxInt;
		}

		while(atLeastOneChange){
			atLeastOneChange=false;
			//classify the points into clusters
			FOR(i, length){
				FOR(j,nbCluster){
					currentSquareDistance[j]=squareNorm(points[i], means[j]);
				}
				if (points[i].z != (f32)smallest(currentSquareDistance,nbCluster)){
					atLeastOneChange=true;
					points[i].z = (f32)smallest(currentSquareDistance,nbCluster);
				}
			}
			//calculate the mean of each cluster
			FOR(k,nbCluster){
				means[k]=mean(points, length,k);
			}
		}
		return means;
	}

	static CvPoint *KMeans(IplImage *image, u32 clusterCount)
	{
		if(!image ||image->nChannels != 1) return NULL;

		cvMorphologyEx(image, image, 0, 0, CV_MOP_CLOSE,2);
		u32 width = image->width;
		u32 height = image->height;
		CvPoint3D32f *points = new CvPoint3D32f[width*height];
		u32 pointsCount = 0;

		FOR(i,height){
			FOR(j,width){
				if (rgb(image,i*width+j)){
					points[pointsCount++] = cvPoint3D32f(j,i,0);
				}
			}
		}
		CvPoint *clusters = KmeansClustering(points, pointsCount, clusterCount, width, height);
		//FOR(i,pointsCount)
			//rgb(image, u32(points[i].y*width + points[i].x)) = ((u32)points[i].z + 1)*255/(clusterCount+1);
		delete points;
		return clusters;
	}

	#ifndef EIGEN_STRUCT
	#define EIGEN_STRUCT
	typedef struct{
		 CvPoint2D32f e1;
		 CvPoint2D32f e2;
		 CvPoint2D32f lambda;
		 CvPoint2D32f mean;
		 //f32 l1;
		 //f32 l2;
		 //f32 mean.x;
		 //f32 mean.y;
	} Eigen;
	#endif // EIGEN_STRUCT

	/**
	  *   PCA over the CbCr space using calibration data.
	  *
	  *	  @return :          eigenvectors (e1 & e2), cooresponding eigenvalues (l1 & l2) 
	  *                      and the center of mass of distribution (mean.x & mean.y)
	  */

	static Eigen PCA(CvPoint2D32f points[], u32 nbPoints){
		CvPoint2D32f mean = cvPoint2D32f(0,0);

		//the covariance matrix
		f32 c0[2][2];	

		FOR(i, nbPoints){
			mean.y+=points[i].x;
			mean.x+=points[i].y;
		}
		mean.y/=(f32)nbPoints;
		mean.x/=(f32)nbPoints;

		//initialisation
		FOR(i,2)
			FOR(j,2)
				c0[i][j]=0.0;

		//CbCb (variance of Cb)
		FOR(i,nbPoints)
			c0[0][0]+=(points[i].x-mean.x)*(points[i].x-mean.x);
		c0[0][0]/=(f32)nbPoints;

		//CbCr (covariance of Cb-Cr = covariance Cr-Cb)
		c0[0][1] = 0.0;
		FOR(i, nbPoints)
			c0[0][1]+=(points[i].x-mean.x)*(points[i].y-mean.y);
		c0[0][1]/=(f32)nbPoints;

		c0[1][0] = c0[0][1];

		//CrCr (variance of Cr)
		FOR(i, nbPoints)
			c0[1][1]+=(points[i].y-mean.y)*(points[i].y-mean.y);
		c0[1][1]/=(f32)nbPoints;

		f32 determinant = (c0[1][1]+c0[0][0])*(c0[1][1]+c0[0][0])-4.0f*(c0[0][0]*c0[1][1]-c0[1][0]*c0[0][1]);
		CvPoint2D32f eig = cvPoint2D32f(0,0);

		if (determinant > 0){
			eig.x= (c0[1][1]+c0[0][0]+sqrtf(determinant))/2.0f;
			eig.y= (c0[1][1]+c0[0][0]-sqrtf(determinant))/2.0f;
		}else{
			//printf("determinant is not positive during calculation of eigenvalues !!");
		}

		CvPoint2D32f e1;
		e1.x=-c0[0][1]/(c0[0][0]-eig.x);
		e1.y=1.0;

		//f32 norm = sqrtf(e1.x*e1.x+e1.y*e1.y);

		CvPoint2D32f e2;
		e2.x=-c0[0][1]/(c0[0][0]-eig.y);
		e2.y=1.0;

		Eigen result;

		result.e1=e1;
		result.e2=e2;
		result.lambda = eig;
		result.mean = mean;

		return result;

	}

	static Eigen PCA(IplImage *x, IplImage *y){
		Eigen eig;
		eig.e1 = cvPoint2D32f(0,0);
		if(!x || !y) return eig;
		u32 length = x->widthStep*x->height;
		if(length != (u32)(y->widthStep*y->height)) return eig;
		CvPoint2D32f *points = new CvPoint2D32f[length];
		FOR(i, length){
			points[i].x = rgb(x, i)/255.f;
			points[i].y = rgb(y, i)/255.f;
		}
		eig = PCA(points, length);
		delete[] points;
		return eig;
	}

	/**
	  *   determines if a point is inside a ellipse given by the eigenvalues (radius), eigenvectors (direction) and center.
	  *
	  *   @param point        : is the CbCr **point** into the ellipse  
	  *	  @param eigenValVect : eigenvalues, corresp. eigenvectors, center of mass

	  *	  @return : true if the *point* is inside the ellipse (i.e this point corresponds to skin color)
	  */

	static bool isInsideEllipse(CvPoint2D32f point, Eigen eigenValVect, f32 proportionFactor){

		//translate the coordinate system to the center of mass of points
		CvPoint2D32f newPoint;
		newPoint.x = point.x - eigenValVect.mean.x;
		newPoint.y = point.y - eigenValVect.mean.y;

		f32 theta;
		
		if (eigenValVect.e1.x >= 0)
			theta = atanf(eigenValVect.e1.y/eigenValVect.e1.x);
		else if(eigenValVect.e2.x >= 0)
			theta = atanf(eigenValVect.e2.y/eigenValVect.e2.x);
		else{
			//printf("\n*** ERROR - NON ORTHOGONAL VECTORS ***\n");
		}

		// rotation
		newPoint.x=newPoint.x*cosf(theta)-point.y*sinf(theta);
		newPoint.y=newPoint.x*sinf(theta)+point.y*cosf(theta);

		return ((((newPoint.x*newPoint.x)/(eigenValVect.lambda.x*proportionFactor*eigenValVect.lambda.x*proportionFactor))
			+((newPoint.y*newPoint.y)/(eigenValVect.lambda.y*proportionFactor*eigenValVect.lambda.y*proportionFactor))) < 1.0f);
	}

	static void SelectPCA(IplImage *x, IplImage *y, IplImage *dst, Eigen eig, f32 ratio)
	{
		if(!x || !y) return;
		if(!dst) dst = cvCreateImage(cvGetSize(x),8,1);
		else if (dst->width != x->width && dst->height != x->height){
			cvReleaseImage(&dst);
			dst = NULL;
			cvCreateImage(cvGetSize(x),8,1);
		}
		u32 width = x->width;
		u32 height = x->height;
		FOR(i,height){
			FOR(j, width){
				rgb(dst,i*width + j) = isInsideEllipse(cvPoint2D32f(rgb(x,i*width+j)/255.f, rgb(y,i*width+j)/255.f), eig, ratio) ? 255 : 0;
			}
		}
	}
}
#endif //_BASICOPENCV_H_

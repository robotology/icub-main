#ifndef __ICUB_BLOB_DESC_SUPPORT_H__
#define __ICUB_BLOB_DESC_SUPPORT_H__

/* helper classes and functions - most are taken from Ivana Cingovska's 2008 work */

/* OpenCV */
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

/* iCub */
#include <iCub/BlobDescriptorModule.h>

/* system */
//#include <fstream>
//#include <iostream>
//#include <sstream>
#include <math.h>
#include <string>
#include <stdio.h>
//#include <vector>
using namespace std;

/* objectDescriptor.h */

//Should create a class with proper constructors and destructors
struct ObjectDescriptor
{
    int no; // number of objects in the image ??? It looks like it is more the index of the object in the array
    int label;
    int area;
    CvPoint center;
    CvHistogram *objHist; // colour descriptor? CHECK
    int h_bins, s_bins;

    IplImage* mask_image; // object mask
    unsigned char *mask_data;

    CvScalar color; // for display? CHECK

	//New fields added for contour processing - Alex 13/12/2009
	CvMemStorage *storage; // = cvCreateMemStorage(0);
	CvSeq *contours; // = 0;
	CvSeq *convexhull; // = 0;

};

/* calcdistances.h */
class Distance
{
private:
	float bhattacharyya(CvHistogram *hist1, CvHistogram *hist2, int h_bins, int s_bins);
    double euclidian(CvPoint c1, CvPoint c2);
    double weightB, weightE, weightA; // weight coefficients of the different distances that contribute to the overall distance

 public:
	Distance(double wB = 1, double wE = 1, double wA = 1);
	double overallDistance(ObjectDescriptor *d1, ObjectDescriptor *d2);
};

/* selectObjects.h */
int selectObjects(IplImage *labeledImage, IplImage *out, int numLabels, int areaThres);
void whiteBalance(IplImage *maskImage, IplImage *originalImage, IplImage *whiteBalancedImg);
int indexOfL(int label, ObjectDescriptor *odTable, int length);
void extractObj(IplImage *labeledImage, int numObjects, ObjectDescriptor *objDescTable);
void int32ToInt8Image(IplImage *labeledIntImage, IplImage *labeledImage);
bool existsInList(int x, int a[], int numEl);
void printImgLabels(IplImage *img, int numObjects);

#endif // __ICUB_BLOB_DESC_SUPPORT_H__

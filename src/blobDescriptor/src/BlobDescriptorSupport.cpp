// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* helper classes and functions, taken from Ivana Cingovska's 2008 work */

/* iCub */
#include <iCub/BlobDescriptorModule.h>
#include <iCub/BlobDescriptorSupport.h>

/* system */
//#include <math.h>
//#include <stdio.h>
using namespace std;

/*
 * helper classes and functions, taken from Ivana Cingovska's 2008 work
 **********************************************************************
 */

/* calcdistances.cpp */
Distance::Distance(double wB, double wE, double wA)
{
    weightB = wB;
    weightE = wE;
    weightA = wA;
}

float Distance::bhattacharyya(CvHistogram *hist1, CvHistogram *hist2, int h_bins, int s_bins)
{
    float dist, coef = 0;
    for (int i = 0; i < h_bins; i++)
        for (int j = 0; j < s_bins; j++)
            {
                float val1 = cvQueryHistValue_2D( hist1, i, j );
                float val2 = cvQueryHistValue_2D( hist2, i, j );
                coef += sqrtf(val1 * val2);
            }
    dist = - log10(coef);
    return dist;
}

double Distance::euclidian(CvPoint c1, CvPoint c2)
{
    double dist;
    dist = ((c1.x - c2.x)*(c1.x - c2.x)) + ((c1.y - c2.y) * (c1.y - c2.y));
    dist = sqrt(dist);
    return dist;
}

double Distance::overallDistance(ObjectDescriptor *d1, ObjectDescriptor *d2)
{
    double dist = 0;
    dist += weightB * cvCompareHist(d1->objHist, d2->objHist, CV_COMP_BHATTACHARYYA);
    //dist += weightB * bhattacharyya(d1->objHist, d2->objHist, d1->h_bins, d2->s_bins);
    dist += weightE * euclidian(d1->center, d2->center);
    dist += weightA * abs((d1->area - d2->area));
    return dist;
}

/* selectObjects.cpp */

/* Function:  selectObjects
Description:  In an image segmentation process, in an integer labeled image, returns an image
              having only the objects that have an area greater than some threshold
Dependencies: OpenCV
Arguments:    labeledImage - integer labeled image
              out          - greyscale image containing the labels (used for visualization only)
              numLabels    - number of labels that the input image contains
              areaThres    - threshold
*/
int selectObjects(IplImage *labeledImage, IplImage *out, int numLabels, int areaThres)
{
    int *area;
    area = new int[numLabels];
    memset(area, 0, sizeof(int)*numLabels);

    int *labeledData;
    //unsigned char *output;
    labeledData = (int*)labeledImage->imageData;
    //output = (unsigned char*)out->imageData;

    int width, height, stride;
    width = labeledImage->width;
    height = labeledImage->height;
    stride = labeledImage->widthStep/sizeof(int);

    int numObjects = 0;

    //calculating the area
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            {
                int k = labeledData[i * width + j];
                if (k != 0)
                    area[k - 1] += 1;
            }
    //annulling the areas smaller than threshold
    for (int i = 0; i < numLabels; i++)
        if (area[i] < areaThres)
            area[i] = 0;
        else numObjects++;


    //annulling the objects with small area
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            {
                int k = area[labeledData[i * width + j] - 1];
                if (k == 0)
                    labeledData[i * width + j] = 0;
            }
    //convert from label image (integer) to output image (unsigned char)
    int32ToInt8Image(labeledImage, out);

	delete [] area;

    return numObjects;
}

/* Function:  whiteBalance
Description:  white balances an image according to the surface that is believed to be white (the table)
Dependencies: OpenCV
Arguments:    maskImage        - binary image that distinguishes the background (black) from the objects (white)
              originalImage    - original RGB image that has to be processed
              whitebalancedImg - result
*/
void whiteBalance(IplImage *maskImage, IplImage *originalImage, IplImage *whiteBalancedImg)
{
    unsigned char *maskData; //int
    unsigned char *orgDataR, *orgDataG, *orgDataB;

    maskData = (unsigned char*)maskImage->imageData;

    IplImage *r_plane, *g_plane, *b_plane;
    r_plane = cvCreateImage(cvGetSize(originalImage),8,1);
    g_plane = cvCreateImage(cvGetSize(originalImage),8,1);
    b_plane = cvCreateImage(cvGetSize(originalImage),8,1);
    cvCvtPixToPlane(originalImage, r_plane, g_plane, b_plane, 0);

    orgDataR = (unsigned char*)r_plane->imageData;
    orgDataG = (unsigned char*)g_plane->imageData;
    orgDataB = (unsigned char*)b_plane->imageData;
    int width, height, stride;
    width = maskImage->width;
    height = maskImage->height;
    stride = maskImage->widthStep/sizeof(unsigned char);

    int areaWhite = 0;
    double rMean = 0, gMean = 0, bMean = 0;
    for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
                {
                    if (maskData[i * stride + j] == 0)
                        {
                            areaWhite++;
                            rMean += orgDataR[i * stride + j];
                            gMean += orgDataG[i * stride + j];
                            bMean += orgDataB[i * stride + j];
                        }
                }
        }
    rMean /= areaWhite; gMean /= areaWhite; bMean /= areaWhite;
    double min = rMean;
    if (gMean < min) min = gMean;
    if (bMean < min) min = bMean;
    double normR, normG, normB;
    normR = min / rMean; normG = min / gMean; normB = min / bMean;
    for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
                {
                    orgDataR[i * stride + j] *= normR;
                    orgDataG[i * stride + j] *= normG;
                    orgDataB[i * stride + j] *= normB;
                }
        }
    //merge the R, G and B planes
    cvMerge(r_plane, g_plane, b_plane, NULL, whiteBalancedImg);
    cvReleaseImage(&r_plane); cvReleaseImage(&g_plane); cvReleaseImage(&b_plane);
}

/* Function:  extractObj
Description:  extracts the characteristics of the objects present in the labeled image as an array of objectDescriptors
Dependencies: OpenCV
Arguments:    labeledImage - integer labeled image
              numLabels    - number of objects (i.e. labels) in the image
              objDescTable - array of objectDescriptors that is to be filled in
*/

/* help function that finds the index of a label in the table of object descriptors */
int indexOfL(int label, ObjectDescriptor *odTable, int length)
{
    int ind = 0;
    while (odTable[ind].label != -1 && odTable[ind].label != label) ind++;
    return ind;
}

void extractObj(IplImage *labeledImage, int numObjects, ObjectDescriptor *objDescTable)
{
    int *labeledData;
    labeledData = (int*)labeledImage->imageData;

    int width, height, stride;
    width = labeledImage->width;
    height = labeledImage->height;
    stride = labeledImage->widthStep/sizeof(int);

    //initialization of the area and labels of the objectDescriptors
    for (int i = 0; i < numObjects; i++)
    {
		objDescTable[i].no = i;
        objDescTable[i].area = 0;
        //objDescTable[i].mask_image = cvCreateImage(cvSize(width, height),8,1);
        //objDescTable[i].mask_data = (unsigned char*)objDescTable[i].mask_image->imageData;
        for( int k = 0; k < height; k++ )
			for( int j = 0; j < width; j++ )
				objDescTable[i].mask_data[k*stride+j]  = 0; //set all the pixels black
        objDescTable[i].center = cvPoint(0,0); //default center of the object
        objDescTable[i].label = -1;
    }
    //calculating the area and the mask images for the histogram for each object
    for (int i = 0; i < height; i++)
    {			
		for (int j = 0; j < width; j++)
        {
			int label = labeledData[i*stride+j];
            if (label != 0)
			{
				int ind = indexOfL(label, objDescTable, numObjects);
                int a = objDescTable[ind].area;
                objDescTable[ind].mask_data[i*stride+j] = 255;
                objDescTable[ind].area++;
                objDescTable[ind].label = label;
                objDescTable[ind].center.x += j;
                objDescTable[ind].center.y += i;
            }
        }
    }

    for (int i = 0; i < numObjects; i++)
    {
        objDescTable[i].center.x /= objDescTable[i].area;
        objDescTable[i].center.y /= objDescTable[i].area;
    }
}

/* Function:  int32ToInt8Image
Description:  converts an IPL image with 32-bit depth to an image with 8-bit depth, i.e.,
              from a labeled image (integer) to an output image (unsigned char)
Dependencies: OpenCV
Arguments:    labeledIntImage - pointer to the 32-bit image
              labeledImage    - pointer to the 8-bit image
*/
void int32ToInt8Image(IplImage *labeledIntImage, IplImage *labeledImage)
{
    int *labeledIntData;
    unsigned char *labeledData;
    labeledIntData = (int*)labeledIntImage->imageData;
    labeledData = (unsigned char*)labeledImage->imageData;

    int width, height, stride;
    width = labeledIntImage->width;
    height = labeledIntImage->height;
    stride = labeledIntImage->widthStep/sizeof(int);

    for(int i = 0; i < height; i++ )
        for(int j = 0; j < width; j++ )
            labeledData[i*stride+j]  = (unsigned char)labeledIntData[i*stride+j];
}

/* Function:  printImgLabels
Description:  prints all the labels that are present on an image
Dependencies: OpenCV
Arguments:    img        - input image
              numObjects - number of objects detected in the image
*/
bool existsInList(int x, int a[], int numEl)
{
    for (int i = 0; i < numEl; i++)
        if (a[i] == x) return true;
    return false;
}
void printImgLabels(IplImage *img, int numObjects)
{
    int *imgData;
    imgData = (int*)img->imageData;

    int width, height, stride;
    width = img->width;
    height = img->height;
    stride = img->widthStep/sizeof(int);

    int label, numLabels = 0;
    int *labelsList = new int[numObjects + 1];
    printf("Existing labels in the image: ");
    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
            {
                label = imgData[i * stride + j];
                if(!existsInList(label, labelsList, numLabels))
                    {
                        printf("%d, ", label);
                        labelsList[numLabels++] = label;
                    }

            }
    printf("\n");
	delete [] labelsList;
}

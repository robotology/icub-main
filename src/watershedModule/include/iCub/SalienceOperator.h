// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _SALIENCEOPERATOR_H_
#define _SALIENCEOPERATOR_H_

#include <yarp/sig/Image.h>//#include <yarp/YARPImage.h>
#include <yarp/math/Math.h>//#include <yarp/YARPMath.h>

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
//IPP include
#include <ipp.h>

//within Project Include
//#include <iCub/ImageProcessor.h>

#include <iCub/ColorVQ.h>
#include <iCub/YARPBox.h>
#include <iCub/YARPIntegralImage.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

#define Giotto1 0
#define Giotto2 1
#define CUST 20
#define FITIN   99
#define FITOUT 101

// Logpolar trans -- Notational conventions:
// x = ro*cos(eta/q)
// y = ro*sin(eta/q)
// ro = k1 + k2*lambda^(csi) csi > F
// r0 = csi					 csi <= F
// J(csi, eta) = lambda^csi * ln(lambda)/q (k1+k2*lambda^csi) outside the fovea
// J(csi, eta) = J(F, eta) within the fovea
// Jan 2004 -- by nat

namespace _logpolarParams
{
	const int _xsize = 256;
	const int _ysize = 256;
	const int _srho = 152;
	const int _stheta = 252;
	const int _sfovea = 42;
	
	const int _xsizefovea = 128;
	const int _ysizefovea = 128;
	// this is the ratio between the full size cartesian image and the actual one
	const double _ratio = 0.25;		// 1/4
	
	// parameter of the transformation
	const double _q = _stheta/(2*PI);
	const double _lambda = 1.02314422608633;
	const double _logLambda = log(_lambda);
	const double _k1 = (_sfovea - 0.5)+(_lambda)/(1-_lambda);
	const double _k2 = _lambda/(pow(_lambda,_sfovea)*(_lambda-1));
};

struct Image_Data
{
	// Logarithm Index
	double Log_Index;
	bool Valid_Log_Index;

	// Zoom Level of the Remapped Image
	double Zoom_Level;

	// Ratio between the diameter of the image and the size of the smallest pixel
	int Resolution;
	double dres;

//	int Fovea_Display_Mode; //0 Sawtooth (Raw); 1 Triangular; 2 Complete

	// Log Polar Metrics
	int Size_Rho;
	int Size_Theta;
	int Size_Fovea;
	int Size_LP;
	int Fovea_Type; //0->3 Giotto 2.0; 4->7 Giotto 2.1 //0;4 Sawtooth (Raw); 1;5 Triangular; 2;6 Complete
	int Pix_Numb;
	int Fovea_Display_Mode;

	// Remapped Cartesian Metrics
	int Size_X_Remap;
	int Size_Y_Remap;
	int Size_Img_Remap;

	// Original Cartesian Metrics
	int Size_X_Orig;
	int Size_Y_Orig;
	int Size_Img_Orig;

	// Color Depth of the Images
	int Orig_Planes;
	int Remap_Planes;
	int LP_Planes;

	// Orientation of the Cartesian Image
	bool Orig_LandScape;
	bool Remap_LandScape;

	int padding;

	float Ratio;  //Used just for naming purpose
};


/**
* Operator that manage the blobs and calculates the saliency of every blob
* @author Francesco Rea
*/

class SalienceOperator {
private:
	/**
	* pointer to the first blob
	*/
	YARPBox *m_boxes;
	/**
	* size of the image
	*/
	int imageSize;
	/**
	* pointer to the checkCutted
	*/
	bool *_checkCutted;
	/**
	* pointer to index cutted
	*/
	PixelInt *_indexCutted;
	/**
	* height of the images
	*/
	int height;
	/**
	* width of the images
	*/
	int width;
public:
	//---methods
	/**
	* default constructor
	*/
	SalienceOperator(){};
	/**
	* constructor
	* @param width1 dimension width
	* @param height1 dimension height
	*/
	SalienceOperator(const int width1,const int height1);
	/**
	* resizes to the new value of width and height
	* @param width1 new width
	* @param height1 new height
	*/
	void resize(const int width1, const int height1);
	/**
	* gets the tagged image of pixels,the R+G-, G*R-,B+Y-, the Red, blue and green Plans and creates a catalog of blobs
	* @param rg R+G- opponency colour image
	* @param rg R+G- opponency colour image
	* @param rg R+G- opponency colour image
	* @param r1 Red plan of the colour image
	* @param g1 Green plan of the opponency colour image
	* @param b1 Blue plan of the opponency colour image
	*/
	void blobCatalog(ImageOf<PixelInt>& tagged,
							   ImageOf<PixelMonoSigned> &rg,
							   ImageOf<PixelMonoSigned> &gr,
							   ImageOf<PixelMonoSigned> &by,
							   ImageOf<PixelMono> &r1,
							   ImageOf<PixelMono> &g1,
							   ImageOf<PixelMono> &b1,
							   int last_tag);  //
	/**
	* conversion from log polar to cartesian of a point
	* @param irho rho ray of the polar point
	* @param itheta itheta angle of the polar point
	* @param ox x position of the point
	* @param oy y position of the point
	*/
	void logPolar2Cartesian(int irho, int itheta, int& ox, int& oy); 
	/**
	* returns the XY center of a data image
	*/
	int Get_XY_Center(double *xx, double *yy, int rho, int theta, Image_Data *par, double *Ang_Shift);
	/**
	* set parameters for Image_data
	*/
	Image_Data Set_Param(int SXO,
					 int SYO,
					 int SXR,
					 int SYR,
					 int rho,
					 int theta,
					 int fovea,
					 int resolution,
					 int LPMode, 
					 double ZoomLevel);
	/**
	* compute Index
	*/
	double Compute_Index(double Resolution, int Fovea, int SizeRho);
	/**
	* draws visual quantized Colour Imade
	*/
	void DrawVQColor(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged);
	/**
	* gets x and y center
	*/
	void get_XY_Center(int* xx, int *yy,int irho,int itheta,ImageOf<PixelRgb> *_img);
	/**
	* returns in box the blob with the maximum saliency value
	* @param tagged the image of the tags
	* @param max_tag maximum value of the tag
	* @param box box of the fovea
	*/
	void maxSalienceBlob(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box); //
	/**
	* draw the blobs present in the fovea area
	* @param id image of the ids
	* @param tagged the image of the tags
	*/
	void drawFoveaBlob(ImageOf<PixelMono>& id, ImageOf<PixelInt>& tagged); //
	/**
	* fuses all blobs in fovea proximity
	*/
	void fuseFoveaBlob3(ImageOf<PixelInt>& tagged, char *blobList, PixelBgr var, int max_tag); //
	/**
	* function that calculates the centre of mass of the blob
	*/
	void centerOfMassAndMass(ImageOf<PixelInt> &in, PixelInt tag, int *x, int *y, double *mass); //
	/**
	* returns the mean colour of the selected blob
	*/
	PixelBgr varBlob(ImageOf<PixelInt>& tagged, ImageOf<PixelMono> &rg, ImageOf<PixelMono> &gr, ImageOf<PixelMono> &by, int tag); //
	/**
	* computes the saliency value for all the blobs 
	*/
	void ComputeSalienceAll(int num_blob, int last_tag); 
	/**
	* count the number of the small blobs
	*/
	int countSmallBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int min_size);
	/**
	* merge blobs that are tagged with the same value
	* @param tagged the image of the tags
	* @param blobList list of the blobs
	* @param max_tag tag with the maximum value
	* @param numBlob number of blobs
	*/
	void mergeBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int numBlob); //
	/**
	* calculates the MeanColours for every blob
	* @param last_tag tag of the last
	*/
	void ComputeMeanColors(int last_tag); //
	/**
	* draws the blobs in the scene painting the blobs with their mean colour
	*/
	void DrawMeanColorsLP(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged); //
	/**
	* draws the most salient blobs in the scene
	*/
	void DrawMaxSaliencyBlob(ImageOf<PixelMono>& id,int max_tag,ImageOf<PixelInt>& tagged);
	/**
	* remove the blobs that are out of range
	*/
	void RemoveNonValidNoRange(int last_tag, const int max_size, const int min_size);
	/**
	*  draw the contrast LP and calculates saliency
	*/
	int DrawContrastLP(ImageOf<PixelMonoSigned>& rg, ImageOf<PixelMonoSigned>& gr,
		ImageOf<PixelMonoSigned>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
		int numBlob, float pBU, float pTD,
		PixelMonoSigned prg, PixelMonoSigned pgr, PixelMonoSigned pby);

	/**
	* draws the contrast LP2 and calculates saliency
	*/
	int DrawContrastLP2(ImageOf<PixelMonoSigned>& rg, ImageOf<PixelMonoSigned>& gr,
								  ImageOf<PixelMonoSigned>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
								  int numBlob, float pBU, float pTD,
								  PixelMonoSigned prg, PixelMonoSigned pgr, PixelMonoSigned pby, PixelMono maxDest);

	//inline methods
	inline YARPBox & getBlobNum(int num){return m_boxes[num];}
	/**
	* removes the fovea blob
	*/
	inline void removeFoveaBlob(ImageOf<PixelInt>& tagged) {m_boxes[tagged(0, 0)].valid=false;}
	/**
	* return the total area of the box passed as parameter
	* @param box reference to the box whose area will be returned
	*/
	inline int TotalArea(YARPBox& box) { return (box.xmax - box.xmin + 1) * (box.ymax - box.ymin + 1); }
	//---attributes
	/**
	* color quantizator
	*/
	ColorVQ *colorVQ;
	/** 
	* integral image of R+G-
	*/
	YARPIntegralImage *integralRG;
	/** 
	* integral image of G+R-
	*/
	YARPIntegralImage *integralGR;
	/** 
	* integral image of B+Y-
	*/
	YARPIntegralImage *integralBY;
	/**
	* one channel image representing the fovea blob
	*/
	ImageOf<PixelMono>* foveaBlob;
	/**
	* image of the most salient blob
	*/
	ImageOf<PixelMono>* maxSalienceBlob_img;
	/**
	* colour quantization image
	*/
	ImageOf<PixelBgr>* colorVQ_img;

};

#endif //_SALIENCEOPERATOR_H_
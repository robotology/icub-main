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

/**
* Operator that manage the blobs and calculates the saliency of every blob
* \author Francesco Rea
*/

class SalienceOperator {
private:
	YARPBox *m_boxes;
	int imageSize;
	bool *_checkCutted;
	PixelInt *_indexCutted;
	int height, width;
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
	* draws the contrast LP2
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
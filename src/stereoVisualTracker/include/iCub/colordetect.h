/*!
 * \file faceDetection.h
 * \author Basilio Noris
 * \date 24-11-06
 */
#ifndef _COLORDETECT_H
#define _COLORDETECT_H

/*!
 * Color Detection, Looks for a specific color and creates a corresponding mask
 * the system uses either one or more histograms. If one histogram only is used
 * the detector will compute the response of that histogram and use it as a mask
 * if more than one histogram are used, the responses from those histograms will
 * be combined to accept only the regions that contain ALL the histograms at the
 * same time.
 */
class ColorDetect : public Filter
{
protected:
	const static bool bVerbose = false; // used to display stuff
	int mode;
	int singleMode;
	int currentHist;
	int nbHist;
	int prevNbHist;
	int editing;
	bool bActive;
	float weights[9];
	double threshold;
	f32 density;
	f32 rDensity;
	f32 sigma;
	CvScalar ycbcr_values;
	CvScalar hsv_values;
	CvPoint *points;
	IplImage *mask, *resp, *fullResp;
	IplImage **singleResp;
	u32 bDrawAdd;
	CvHistogram *histCbcr, *histHsv, *addHistCbcr, *addHistHsv, *hist1, *hist2, *hist3, *hist4;
	CvHistogram **multiHist;
	u32 addHistCnt;

public:

	/*! The Standard Constructor */
	ColorDetect();

	/*! The Deconstructor */
	~ColorDetect();

	/*!
		Extracts the color information from the image portion selected.
		\param image the input image
		\param selection the input sample region
		\param msk optional mask for sample pixels
	*/
	void SetColor(IplImage *image, CvRect selection, IplImage *msk=NULL);

	/*!
		Get the color mask
		\return returns the color mask
	*/
	IplImage *GetMask(){return mask;};
	
	/*!
		Get the histogram filtered response for a histogram combination
		\return returns the histogram filtered response
	*/
	IplImage *GetResp(){return fullResp;};

	/*!
		Get the histogram filtered response for a single histogram
		\return returns the histogram filtered response
	*/
	IplImage *GetSingleResp(int a){return singleResp[a];};

	/*!
		Get the normalized density of the mask
		\return returns the mask density
	*/
	f32 GetDensity(){return density;};

	/*!
		Get the normalized density of the combined histograms filtered response
		\return returns the histograms filtered response density
	*/
	f32 GetRDensity(){return f32(cvSum(fullResp).val[0])/(fullResp->width*fullResp->height*(1<<fullResp->depth));};

	/*!
		Get the normalized density of a single histogram filtered response
		\return returns the histogram filtered response density
	*/
	f32 GetSRDensity(int a){return f32(cvSum(singleResp[a]).val[0])/(singleResp[a]->width*singleResp[a]->height*(1<<singleResp[a]->depth));};

	/*!
		Get the current histogram mode (multi histogram combination or single histograms in parallel)
		\return returns the multi/single mode
	*/
	int GetMSMode(){return singleMode;};

	/*!
		Use this to know if the histograms are enabled, otherwise you'll get an error when accessing the first time
		\return returns the activity
	*/
	bool IsActive(){return bActive;};

	/*!
		Get the number of histograms currently used (i.e. the number of different color to search for)
		\return returns the histogram count
	*/
	int GetNbHist(){return nbHist;};

	/*!
		Get the color model sigma
		\return returns sigma
	*/
	f32 GetSigma(){return sigma;};

	/*!
		Set the color model sigma
		\param s sigma
	*/
	void SetSigma(f32 s){sigma = s;};

	/*!
		Saves the current color model to file
		\param filename the destination file
	*/
	void Save(const char *filename);

	/*!
		Loads the current color model from file
		\param filename the source file
		\return 1 if file could be opened, 0 otherwise
	*/
	int Load(const char *filename);

	/*!
		Swaps the color mode between Normalized Color Components and YCbCr
	*/
	void Mode(){mode ^= 1;/*printf("%s\n",mode?"ncc":"ycbcr");*/};

	/*!
		The Processing function, finds all occurrences of the color model in the input image
		\param image the input image
	*/
	void Apply(IplImage *image);

	/*!
		The Processing function, finds all occurrences of the color model in the input image
		\param image the input image
		\return returns the mask image
	*/
	IplImage *Process(IplImage *image);

	/*!
		Draws the mask on the input image, either masking or highlighting the selected pixels
		\param image the input image, will be modified
	*/
	void DrawMask(IplImage *image);

	/*!
		Changes the drawing mode from masking to highlighting
		\param drawmode the mode: 0-masking 1-highlight
	*/
	void DrawMode(u32 drawmode = 1){bDrawAdd = drawmode;};

	/*!
		The Configuration function, extracts the color information from the selected portion of the image
		\param image, the input image
		\param selection the input sample region
	*/
	void Config(IplImage *image, CvRect selection, IplImage *mask=0){SetColor(image, selection, mask);}

	/*!
		Computes the 2D histogram from two image channels in a given region with an optional mask
		\param cbImg the first channel
		\param crImg the second channel
		\param rect the region to limit the computation to
		\param msk the mask (optional) only white pixels will be computed
		\return returns the corresponding 2d histogram
	*/
	CvHistogram *getHist(IplImage *cbImg, IplImage *crImg, CvRect rect, IplImage *msk=NULL);

	/*!
		Computes the 3D histogram from three image channels in a given region with an optional mask
		\param yImg the first channel
		\param cbImg the second channel
		\param crImg the third channel
		\param rect the region to limit the computation to
		\param msk the mask (optional) only white pixels will be computed
		\return returns the corresponding rd histogram
	*/
	CvHistogram *getHist(IplImage* yImg, IplImage* cbImg, IplImage* crImg, CvRect rect, IplImage *msk=NULL);

	/*!
		Computes the sum of two histograms
		\param a the first histogram, the result will be stored in this histogram
		\param b the second histogram
	*/
	void SumHist(CvHistogram *a, CvHistogram *b);

	/*!
		Display an image representation of the given 2d histogram
		\param hist the input histogram
	*/
	void DrawHist(CvHistogram *hist);

	/*!
		Draws all histograms
	*/
	void DrawHists();

	/*!
		Resets the histogram count to 0 effectively clearing all other histograms
	*/
	void ResetHist(){addHistCnt = 0;};

	/*!
		Select a given histogram, selecting the histogram will clear it
		\param index the desired histogram index
	*/
	void SelectHist(int index=0);

	/*!
		Add or replaces an histogram to the current histograms stack
		\param hist the histogram to be added
		\param pos the position in the stack of the histogram
	*/
	void AddHist(CvHistogram *hist, int pos);
};


#endif // _COLORDETECT_H

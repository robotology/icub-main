/*!
 * \file GmmTracker.h
 * \author Basilio Noris
 * \date 25-11-06
 */
#ifndef _GMMFILTER_H_
#define _GMMFILTER_H_

#include "gmm.h"

/*!
 * GmmTracker, applies a mixture of gaussians to track blobs in a grayscale image
 */

#include "public.h"
class GmmTracker : public Filter
{
private:
	Mixture *mix;
	u32 kernels;
	f64 *ws;
	u32 emLimit;
	Database *DB;
	u32 age;

public:
	/*!
		The Standard Constructor, taking the size and number of kernels
		\param size size of the image/input array
		\param kernelCnt the number of kernels
	*/
	GmmTracker(CvSize size, u32 kernelCnt);

	/*!
		Get the mixture
		\return returns the mixture itself
	*/
	Mixture *Get();

	/*!
		Randomizes the mixture starting points
	*/
	void Randomize();

	/*!
		Get the coordinates for a given kernel
		\param index index of the kernel
		\return returns the 2d coordinates of the kernel
	*/
	CvPoint GetCoords(u32 index){return mix->GetCoords(index);};

	/*!
		Get the current age of the tracker
		\return the number of frames since the last reinitialization
	*/
	u32 GetAge(){return age;};

	/*!
		Gets the current likelihood
	*/
	f32 GetLikelihood();

	/*!
		The Processing function, fits the gaussian kernels to the input image
		\param image the input image
	*/
	void Apply(IplImage *image);

	/*!
		Overlays the Gaussian Mixture over the input image
		\param image the input image
	*/
	void DrawGmm(IplImage *image);
};

#endif // _GMMFILTER_H_

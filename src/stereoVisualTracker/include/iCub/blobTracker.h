/*!
 * \file blobTracker.h
 * \author Basilio Noris
 * \date 25-11-06
 */
#ifndef _BLOBTRACKER_H_
#define _BLOBTRACKER_H_

#include "basicMath.h"

/*!
 * Blob Tracker, Tracks blobs in a mask image using the Continuously Adaptive Mean Shift algorithm
 */
class BlobTracker : public Filter
{
private:
	u32 maxBlobs;
	u32 current;
	CvBox2D *track_boxes;
	CvRect *track_windows;
	CvConnectedComp track_comp;
	u32 *track_age;
	f64 *track_density;
	CvSize *track_max_size;
	bool *bTrack;
	bool bTracking;

public:
	/*!
		The Standard Constructor, takes the maximum number of blobs to track
		\param cnt maximum blob count (default=1)
	*/
	BlobTracker(u32 cnt=1);
	/*!
		The Deconstructor
	*/
	~BlobTracker();

	/*!
		Clears all the tracking points 
	*/
	void Clear();

	/*!
		Adds a new blob position to the tracker, blobs selected after the max_blobs will replace the first ones
		\param selection the blob starting position
	*/
	void AddBlob(CvRect selection);

	/*!
		Get blob coordinates
		\param index the index of the tracked blob
		\return returns the 2d coordinates of the blob centroid
	*/
	CvPoint2D32f GetCoords(u32 index);
	
	/*!
	  Indicated if the blob is found
	  \param index the index of the tracked blob
	  \return return 1 if the blob is found 0 otherwise
	*/
	int Found(u32 index=0);
	/*!
		Get the blob density
		\param index the index of the tracked blob
		\return the density of the blob
	*/
	f64 GetDensity(u32 index);

	/*!
		Get Angle from the center of the image of the tracked blob
		\param index index of the blob
		\param imageSize size of the image
		\param distance distance of the object from the camera (default=500)
		return returns vector containing the two angles (vertical and horizontal) of the blob
	*/
	CvPoint2D32f GetAngle(u32 index, CvSize imageSize, u32 distance=500);

	/*!
		Get position of the search window for a given blob
		\param index index of the blob
		\return the 2d position of the search window
	*/
	Vec2 GetWindowPosition(u32 index);

	/*!
		Set position of the search window for a given blob
		\param index index of the blob
		\param position the position of the search window
		\param size the size of the search window
	*/
	void SetWindowPosition(u32 index, Vec2 position, Vec2 size);

	/*!
		Is the tracking active
		\return returns whether the tracker is active or not
	*/
	u32 IsTracking(){return bTracking;};

	/*!
		Is the tracking active for a given blob
		\param index index of the blob
		\return returns whether the tracker of a given blob is active or not
	*/
	u32 IsTracking(u32 index){return index < maxBlobs ? bTrack[index] : 0;};

	/*!
		The Processing function, tracks all the blobs on the input mask image
		\param image the input image (grayscale or binary)
		\param index the index of the blob, -1 means all the blobs
	*/
	void Apply(IplImage *image, int index=-1);
	
	/*!
		Overlays the position of the tracked blobs over the input image
		\param image the input image
		\param bLine if set, draws a line from the center of the image to the center of the blobs
	*/
	void DrawBlobs(IplImage *image, u32 bLine=0);

	/*!
		The Configuration function, adds a new blob to the tracker
		\param image the input image (useless here but kept for compatibility with the Filter interface)
		\param selection the initial position and size of the search window
	*/
	void Config(IplImage *image, CvRect selection){AddBlob(selection);}
};

#endif // _BLOBTRACKER_H_

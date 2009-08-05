/*!
 * \file imgGrabber.h
 * \author Basilio Noris
 * \date 27-11-06
 */
#ifndef _IMGGRABBER_H_
#define _IMGGRABBER_H_

/*!
 *	Image Sequence Grabber, grabs images from a file sequence (*001.*,*002.*,...)
 */
class ImgGrabber : public FrameGrabber
{
private:
	u32 currentFrame;	// the current frame number
	bool bLoop;
	u32 width;
	u32 height;
	u32 length;
	f32 framerate;
	char *filename;		// current filename
	u32 fnNumFormat;	// number of digits in the filename
	char *fnBase;		// filename base (without digits)
	char *fnExtension;	// extension of the images

public:
	/*!
		The Standard Constructor
		\param filename the name of the first image in the sequence
		\param framerate the desired framerate (default=30)
	*/
	ImgGrabber (char *firstname, f32 frate=30);

	/*!
		Sets the current position of the sequence to a set percentage
		\param percentage the percentage of the sequence to skip to (default=0)
	*/
	void SetPosition(f32 percentage=0);

	/*!
		Sets the current frame of the sequence to a set frame
		\param frame the frame to skip to (default=0)
	*/
	void SetFrame(u32 frame=0);

	/*!
		Sets the current looping instruction
		\param loop the looping instruction
	*/
	void SetLoop(bool loop);

	/*!
		Skips a given amount of frames
		\param frames the amount of frames to skip
	*/
	void Skip(u32 frames);

	/*!
		Gets the resolution of the sequence
		\return returns the resolution of the sequence
	*/
	CvSize GetSize();

	/*!
		Gets the sequence framerate
		\return returns the sequence framerate
	*/
	f32 GetFramerate();

	/*!
		Gets the current percentage of the sequence
		\return returns the percentage of the sequence the current frame is at
	*/
	f32 GetPercentage();

	/*!
		Gets the length in frames of the sequence
		\return returns the length of the sequence
	*/
	u32 GetLength();

	/*!
		Gets the current frame index
		\return returns the current frame index
	*/
	u32 GetCurrentFrame();

	/*!
		Gets whether the grabber is looping
		\return returns the looping instruction
	*/
	bool GetLoop();

	/*!
		Is the sequence done
		\return returns whether the current frame is the end of the sequence
	*/
	bool IsDone();

	/*!
		The Grabbing function
		\param frame the destination frame pointer
		\index not used
	*/
	void GrabFrame(IplImage **frame, u32 index=0);

	/*!
		The Kill function, frees up the buffers allocated by the grabber
	*/
	void Kill();
};

#endif //_IMGGRABBER_H_

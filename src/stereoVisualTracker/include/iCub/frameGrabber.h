/*!
 * \file frameGrabber.h
 * \author Basilio Noris
 * \date 25-11-06
 */
#ifndef _FRAMEGRABBER_H_
#define _FRAMEGRABBER_H_

/*!
 * The Frame Grabber virtual class
 */
class FrameGrabber
{
public:
	/*!
		Grabs the current frame
		\param frame the pointer to the frame that will be filled by the grabber
		\param index can be the index to a particular frame or to one of many streams (in case of stereo grabbers)
	*/
	virtual void GrabFrame(IplImage **frame, u32 index=0){};

	/*!
		Kills the grabber, freeing and releasing up the memory allocated by the grabber
	*/
	virtual void Kill(){};

	virtual CvSize* GetSize(){return NULL;};
	
	virtual double GetTime(){return 0.0;};
};

#endif // _FRAMEGRABBER_H_

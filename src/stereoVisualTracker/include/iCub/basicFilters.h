/*!
 * \file basicFilters.h
 * \author Basilio Noris
 * \date 27-11-06
 */
#ifndef _BASICFILTERS_H_
#define _BASICFILTERS_H_

/*!
 * Frame Counter class, filter for keeping a global count of the processed frames
 */
class FrameCounter : public Filter
{
private:
	/*!
		The static frame counter
	*/
	static u32 frame;
public:
	/*!
		The Standard Constructor
	*/
	FrameCounter();

	/*!
		The Processing function, simply increments the frame counter
		\param image not used
	*/
	void Apply(IplImage *image){frame++;};

	/*!
		Gets the current frame count
		\return returns the frame counter value
	*/
	static u32 GetFrame(){return frame;};

	/*!
		Sets or resets the current frame counter
		\param fr the new frame count (default=0)
	*/
	static void SetFrame(u32 fr=0){frame = fr;};
};

/*!
 * Text class, filter for writing text over images
 */
class Text : public Filter
{
protected:
	/*! Color text will be written in*/
	CvScalar color;
	/*! Font of the text */
	CvFont font;
	/*! Coordinates of the starting point for the text */
	CvPoint position;
	/*! text buffer */
	char *text;
public:
	/*!
		The Standard Constructor
	*/
	Text();

	/*!
		Constructor taking as input the text and the position
		\param txt the text string
		\param pos the position for the text
	*/
	Text(char *txt, CvPoint pos);

	/*!
		The Processing function, applies the buffered text to
		the image at the preset position
		\param image the input image, will be modified by the filter
	*/
	void Apply(IplImage *image);

	/*!
		Sets the text buffer
		\param txt the new text string
	*/
	void SetText(char *txt);

	/*!
		Sets the position for the text
		\param pos the new text position
	*/
	void SetPosition(CvPoint pos);

	/*!
		Sets the text color
		\param c the new text color
	*/
	void SetColor(CvScalar c);
};

/*!
 *	Show FPS class, shows the current frames per second rate
 *	requires a FrameCounter filter to work
 */
class ShowFPS : public Text
{
private:
	/*! The starting time for framerate computation*/
	u32 startTime;
	/*! the expected framerate */
	f32 framerate;
public:
	/*!
		The Standard Constructor
	*/
	ShowFPS();

	/*!
		Constructor taking expected framerate and text position
		\param rate the expected framerate
		\param pos the fps text position
	*/
	ShowFPS(f32 rate, CvPoint pos);

	/*!
		The Processing function, writes the fps rate on the image
		\param image the input image, will be modified by the filter
	*/
	void Apply(IplImage *image);
};

#endif //_BASICFILTERS_H_
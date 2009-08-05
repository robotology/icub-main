#include <stdio.h>
#include "public.h"
#include "imgGrabber.h"

#pragma warning(disable: 4996)

ImgGrabber::ImgGrabber(char *firstname, f32 frate)
{
	if(!firstname) return;
	u32 len = (u32)strlen(firstname);
	if (len < 5) return;
	fnExtension = new char[5];
	filename = new char[len+1];
	sprintf(fnExtension, "%s",&firstname[len-4]);
	strcpy(filename, firstname);

	fnNumFormat = 0;
	FOR(i,len-4){
		char c = filename[len-5 - i];
		if(c < '0' || c > '9') break;
		fnNumFormat++;
	}

	FILE *f;
	f = fopen(filename, "rb");
	u32 filecount = 0;
	length = 0;
	while(f){
		fclose(f);
		filecount++;
		length++;
		if(filecount == 1000){
			char *newFilename = new char[255];
			filename[length-7] = '\0';
			len++;
			sprintf(newFilename,"%s1000",filename);
			filename = newFilename;
			filecount = 0;
		}
		filename[len-7] = '\0';
		sprintf(filename, "%s%.3d%s",filename, filecount, fnExtension);
		f = fopen(filename, "r");
	}
	strcpy(filename, firstname);
	IplImage *im = cvLoadImage(filename);
	width = im->width;
	height = im->height;
	fnBase = new char[len-4-fnNumFormat+1];
	memcpy(fnBase, filename, len-4-fnNumFormat);
	fnBase[len-4-fnNumFormat] = '\0';
	framerate = frate;
	currentFrame = 0;
	cvReleaseImage(&im); im = NULL;
	bool bLoop = true;
}

void ImgGrabber::SetPosition(f32 percentage)
{
	trim(percentage, 0, 1.f);
	currentFrame = (u32)(length * percentage);
}

void ImgGrabber::Skip(u32 frames)
{
	currentFrame += frames;
	if (currentFrame > length) currentFrame = length-1;
}

void ImgGrabber::SetFrame(u32 frame)
{
	currentFrame = frame;
}

void ImgGrabber::GrabFrame(IplImage **frame, u32 index)
{
	if(currentFrame >= length) return;
	char format[255];
	sprintf(format, "%%s%%.%dd%%s",fnNumFormat);
	sprintf(filename, format, fnBase, currentFrame, fnExtension);
	IplImage *f = (*frame);
	// hack to avoid undefined pointers or freed pointers 
	if(f){
		cvReleaseImage(&f); f = NULL;
	}
	f = cvLoadImage(filename);
	(*frame) = f;
	currentFrame++;
	if(bLoop && currentFrame >= length) currentFrame = 0;
}

void ImgGrabber::Kill()
{
	delete filename;
	delete fnExtension;
	delete fnBase;
}

f32 ImgGrabber::GetFramerate()
{
	return framerate;
}

CvSize ImgGrabber::GetSize()
{
	return cvSize(width, height);
}

f32 ImgGrabber::GetPercentage()
{
	return f32(currentFrame) / length;
}

u32 ImgGrabber::GetLength()
{
	return length;
}

u32 ImgGrabber::GetCurrentFrame()
{
	return currentFrame;
}

void ImgGrabber::SetLoop(bool loop)
{
	bLoop = loop;
}

bool ImgGrabber::GetLoop()
{
	return bLoop;
}


bool ImgGrabber::IsDone()
{
	return currentFrame >= length;
}

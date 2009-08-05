// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 Basilio Noris
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <stdio.h>
#include "public.h"
#include "basicMath.h"
#include "blobTracker.h"

#define TRACK_MIN_HEIGHT 5
#define TRACK_MIN_WIDTH 5
#define ADAPTIVE_TIMESPAN 100
#define ADAPTIVE_ALPHA 0.9

BlobTracker::BlobTracker(u32 cnt)
{
	maxBlobs = cnt;
	current = 0;
	bTracking = false;

	track_windows = new CvRect [maxBlobs];
	track_boxes = new CvBox2D [maxBlobs];
	bTrack = new bool[maxBlobs];
	track_max_size = new CvSize[maxBlobs];
	track_density = new f64[maxBlobs];
	track_age = new u32[maxBlobs];

	FOR(i, maxBlobs){
		bTrack[i] = false;
	}
}

BlobTracker::~BlobTracker()
{
	delete [] track_windows;
	delete [] track_boxes;
	delete [] bTrack;
	delete [] track_max_size;
	delete [] track_density;
	delete [] track_age;
}

void BlobTracker::AddBlob(CvRect selection)
{
    track_windows[current] = selection;
	bTrack[current] = true;
	bTracking = true;
	track_max_size[current] = cvSize((u32)(selection.width*1.5f), (u32)(selection.height*1.5f));
	track_age[current] = 0;

	current++;
	current %= maxBlobs;
}

void BlobTracker::Clear()
{
    FOR(i, maxBlobs){
		bTrack[i] = false;
	}
	bTracking = false;
	current = 0;
}
int meanId = 0;

void BlobTracker::Apply(IplImage *image, int index)
{
  int l_range, u_range;
  int i;

  if(!image || !bTracking) return;

  if(index>=0 && index<maxBlobs){ //	FOR(i,maxBlobs){
    l_range =index;
    u_range =index+1;
  }
  else{
    l_range=0;
    u_range=maxBlobs;
  }
  for(i=l_range;i<u_range;i++){
    if (bTrack[i]){
      CvBox2D track_box;
      track_boxes[i].size.height *= 0.3f;
      track_boxes[i].size.width *= 0.3f;
	    
      Truncate(track_windows[i].height, track_max_size[i].height);
      Truncate(track_windows[i].width, track_max_size[i].width);
	    
      cvCamShift( image, track_windows[i],
		  cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 1 ),
		  &track_comp, &track_box);
	    
      f64 density = (track_comp.rect.height != 0 && track_comp.rect.width != 0) ?
	track_comp.area / track_comp.rect.width / track_comp.rect.height : 0;
	    
      track_boxes[i] = track_box;
      track_density[i] = density/256;
	    
      // updates the track window
      track_windows[i] = track_comp.rect;
      track_age[i]++;
	    
      if (track_boxes[i].size.height < TRACK_MIN_HEIGHT && 
	  track_boxes[i].size.width < TRACK_MIN_WIDTH){
	  track_boxes[i].size.width = 0;
	  track_boxes[i].size.height = 0;
	  track_density[i] = 0;
	}
	    
      if(!image->origin ) track_boxes[i].angle = -track_boxes[i].angle;
    }
  }
}



void BlobTracker::DrawBlobs(IplImage *image, u32 bLine)
{
	FOR(i,maxBlobs){
		if(track_boxes[i].size.width > 1 && track_boxes[i].size.height > 1)
			cvEllipseBox(image, track_boxes[i], CV_RGB(255,255,255), 1, CV_AA, 0);
		if(bLine) cvLine(image, cvPoint((u32)track_boxes[i].center.x, (u32)track_boxes[i].center.y),
			cvPoint((u32)track_boxes[i].center.x, (u32)track_boxes[i].center.y),
			CV_RGB(255,0,0), 2, CV_AA, 0);
	}
}

CvPoint2D32f BlobTracker::GetCoords(u32 index)
{
	if (index < maxBlobs && bTrack[index] && track_boxes[index].size.height > 1) return track_boxes[index].center;
	else return cvPoint2D32f(-1, -1);
}

int BlobTracker::Found(u32 index){
  CvPoint2D32f p = GetCoords(index);
  return (p.x!=-1 || p.y !=-1);
}
				   


f64 BlobTracker::GetDensity(u32 index)
{
	return (index < maxBlobs && bTrack[index]) ? track_density[index] : 0;
}

CvPoint2D32f BlobTracker::GetAngle(u32 index, CvSize imageSize, u32 distance)
{
	u32 width = imageSize.width;
	u32 height = imageSize.height;
	CvPoint2D32f coords = GetCoords(index);
	if(coords.x == -1) coords = cvPoint2D32f(width/2, height/2);
	f32 x = coords.x - (f32)(width>>1);
	f32 y = coords.y - (f32)(height>>1);
	return cvPoint2D32f(atan(x/distance), atan(y/distance));
}

Vec2 BlobTracker::GetWindowPosition(u32 index)
{
	return vec2(track_windows[index].x, track_windows[index].y);
}

void BlobTracker::SetWindowPosition(u32 index, Vec2 position, Vec2 size)
{
	u32 w = track_max_size[index].width;
	u32 h = track_max_size[index].height;
	u32 x = (u32)position.x;
	u32 y = (u32)position.y;

	track_windows[index].x = x;
	track_windows[index].y = y;
	track_windows[index].width = (u32)size.x;
	track_windows[index].height = (u32)size.y;
}

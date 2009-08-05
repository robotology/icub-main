// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Chris McCarthy 
 *
 */


#include <ImpactMapModule.h>
#include <ImpactData.h>

ImpactMapModule::ImpactMapModule()
{
	this->_last = 0;
	this->_buf = NULL;
	this->_alive = 0;
	this->_divPosCount = 1;
   this->_prevTTC = 0;
   this->_prevDiffx = 0;
   this->_prevDiffy = 0;
   this->_maxDiffxAcc = 0;
   this->_maxDiffyAcc = 0;
   this->_prevMinDiffx = 0;
   this->_prevMinDiffy = 0;
   this->_minDiffxAcc = 0;
   this->_minDiffyAcc = 0;
   this->_prev;
   this->_prevTTCDiff = 1;
   this->_accTTCDiff = 0;
	this->_prevMaxDivPoint = cvPoint(0, 0);
   this->_prevMinDivPoint = cvPoint(0, 0);
   this->_flowUpdate = 0;
   this->_accTime = 0;
   this->_divCount = 0;
	this->_ratioAcc = 0;
  	this->_prevRatio = 0;
	this->_interestPoint = IP_MAX_DIV;
	this->_firstDivPosUpdate = 1;

}

ImpactMapModule::~ImpactMapModule()
{}



void ImpactMapModule::createTranslationTemplates(IplImage *tmpMatchImg[MAX_MATCH_WINS][2], int width, int height)
{
   float u_vals[MAX_MATCH_WINS] = {0, 0, 1, 0, -1, 1/ROOT2, -1/ROOT2, 1/ROOT2, -1/ROOT2};
   float v_vals[MAX_MATCH_WINS] = {0, 1, 0, -1, 0, 1/ROOT2, -1/ROOT2, -1/ROOT2, 1/ROOT2};

   for(int n = 1; n < MAX_MATCH_WINS; n++){
      tmpMatchImg[n][0] = cvCreateImage(cvSize(width, height),
                                                            IPL_DEPTH_32F, 1);
      tmpMatchImg[n][1] = cvCreateImage(cvSize(width, height),
                                                            IPL_DEPTH_32F, 1);
      BwImageFloat local_u_vels(tmpMatchImg[n][0]);
      BwImageFloat local_v_vels(tmpMatchImg[n][1]);

      for(int i = 0; i < width; i++){
         for(int j = 0; j < height; j++){
            local_u_vels[j][i] = u_vals[n];
            local_v_vels[j][i] = v_vals[n];
         }
      }
   }


}


// merge two equal sized images together
void ImpactMapModule::mergeImages(IplImage *src1, IplImage *src2, IplImage *out)
{
   CvMat *row1, *row2i;
   uchar *buf1, *buf2, *imagebuf;
   int step1, step2, step3;

   cvGetRawData( src1, &buf1, &step1);
   cvGetRawData( src2, &buf2, &step2);
   cvGetRawData( out, &imagebuf, &step3);

   for(int i = 0; i < 240; i++){
      memcpy(i*step3 + imagebuf, i*step1+buf1, step1);
      memcpy(i*step3 + step1 + imagebuf, i*step2+buf2, step2);
   }
   cvSetData(out, imagebuf, step3);
}


void ImpactMapModule::makePyrVectorImage(IplImage *image, 
					 CvPoint2D32f* points[2], int count, char *status, 
					 IplImage *vels_mask, float tangentAngle, CvPoint center,
					 CvRect boundingBox)
{
   BwImage local_mask(vels_mask);
   float magnitude = 10;

   cvRectangle(image, cvPoint(boundingBox.x*POINT_DIST,
                  boundingBox.y*POINT_DIST),
                  cvPoint((boundingBox.x+boundingBox.width)*POINT_DIST,
                  (boundingBox.y+boundingBox.height)*POINT_DIST),
                  CV_RGB(255, 0, 0));

   center = cvPoint( ((boundingBox.x + boundingBox.width/2)*POINT_DIST),
                      ((boundingBox.y + boundingBox.height/2)*POINT_DIST) );

   CvScalar angleColour = CV_RGB(255, 255, 255);
   cvCircle( image, center, cvRound(magnitude*1.2),
                   CV_RGB(255, 255, 255), 3, CV_AA, 0 );

   cvLine( image, center,
                   cvPoint( cvRound( center.x + magnitude*cos(tangentAngle)),
                   cvRound( center.y - magnitude*sin(tangentAngle))),
                   CV_RGB(255, 255, 255), 3, CV_AA, 0 );

   for( int i = 0; i < count; i++ ) {
      if(!local_mask[(int)(points[0][i].y/POINT_DIST)]
                    [(int)(points[0][i].x/POINT_DIST)])
         continue;

      cvCircle( image, cvPointFrom32f(points[0][i]),
                      1, CV_RGB(0,255,0), -1, 8,0);
      cvLine(image, cvPointFrom32f(points[0][i]),
                      cvPointFrom32f(points[1][i]), CV_RGB(0, 255, 0));

   }
}


//draws impact location map
//outout to
void ImpactMapModule::makeImpactLocationImage(IplImage *impactImage, float ttc,
                                                CvPoint2D32f impactLocation)
{
   int width = impactImage->width;
   int height = impactImage->height;
   int half_width = width/2;
   int half_height = height/2;
   CvPoint half_box = cvPoint(10,10);
   int ttcPix = 255 - cvRound(ttc/TTC_SCALE * 256);
   CvPoint offset;
   CvPoint impactImageLocation;

   impactImageLocation =
                  cvPoint(cvRound(half_width + half_width*impactLocation.x),
                  cvRound(half_height + half_width*impactLocation.y));

   cvRectangle(impactImage, cvPoint(impactImageLocation.x-half_box.x,
            impactImageLocation.y-half_box.y),
            cvPoint(impactImageLocation.x+half_box.x,
            impactImageLocation.y+half_box.y), CV_RGB(ttcPix, ttcPix, ttcPix),
            CV_FILLED);

   cvRectangle(impactImage, cvPoint(impactImageLocation.x-half_box.x,
                                     impactImageLocation.y-half_box.y),
                                     cvPoint(impactImageLocation.x+half_box.x,
                                     impactImageLocation.y+half_box.y),
                                     CV_RGB(255, 255, 255));

   /*
      CvFont font;
      cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
      char ttcString[10];
      sprintf(ttcString, "%.1f", ttc*10000);
      //, double shear=0, int thickness=1, int line_type=8 );
      //cvPutText(dst2, ttcString, cvPoint(5, 15), &font, CV_RGB(255,255,255) );
   */
}



// computes location of impact on plane about camera.  Approach angle
// is used to estimate radial distance from the camera.  Angle of
// tangential motion is used to estimate direction.
// Note: currently this does not account for interest points that are not
// about the image centre.
// returns 0 if ttc is < 0
int ImpactMapModule::computeImpactLocation(CvPoint2D32f *impactLocation,
					 				float ttc, float approachAngle, float tangentAngle,
                           CvPoint imagePos)
{
   float radDist = tan(CV_PI/2.0 - approachAngle);

   if(ttc >= 0 ){
      *impactLocation = cvPoint2D32f(radDist*cos(tangentAngle),
                                       -radDist*sin(tangentAngle));

      if(_divPosCount < TTC_N){
           _locHist[_divPosCount%(TTC_ARRAY_SIZE)] = *impactLocation;
      }else{
         float accLocX = 0;
         float accLocY = 0;

         _locHist[_divPosCount%(TTC_ARRAY_SIZE)] = *impactLocation;
         for(int i=0; i < TTC_ARRAY_SIZE; i++){
             accLocX += _locHist[i].x;
             accLocY += _locHist[i].y;
         }
         impactLocation->x = accLocX/TTC_ARRAY_SIZE;
         impactLocation->y = accLocY/TTC_ARRAY_SIZE;
      }
      return 1;
   }
   return 0;
}


// compute time-to-contact from divergence(div).  Assuming spherical
// projection (or an interest region centred on the optical axis), this is
// computed as 2/div (though it should still work ok for a small FOV)
float ImpactMapModule::calcTimeToContact(float div, float kp, float ki, float kd)
{
   float acc = 0;
   float thisTTC = 2.0/div;
   float new_ttc;

   if(_divPosCount == 0){
      _prevTTC = thisTTC;
   }else{
      float ttcDiffErr = (thisTTC - _prevTTC);
      _accTTCDiff += ttcDiffErr;
      float newTTCDiff =( kp*ttcDiffErr + kd*(ttcDiffErr-_prevTTCDiff) +
                                             ki*(_accTTCDiff/_divPosCount));
      _prevTTCDiff = newTTCDiff;
      _prevTTC += newTTCDiff;
   }
   return _prevTTC;
}



// use angle of approach (w.r.t camera optical centre) from div/tangential
// ratio
float ImpactMapModule::computeApproachAngle(float newRatio)
{
   float ratio;

   _ratioCount++;
   ratio = rKP*(newRatio) - rKD*(newRatio - _prevRatio) +
           rKI*(_ratioAcc/_ratioCount);
   _prevRatio = ratio;
   _ratioAcc+=ratio;

   return(atan(ratio));
}


// calculate tangental motion direction
float ImpactMapModule::calcTangentAngle(IplImage *u_vels, IplImage *v_vels,
                       IplImage *u_res, IplImage *v_res, IplImage *vels_mask,
                       IplImage *tmpX[2], IplImage *tmpY[2], CvRect tmpRect,
                       float *mag)
{
   cvSetImageROI(tmpX[0], tmpRect);
   cvSetImageROI(tmpX[1], tmpRect);
   cvSetImageROI(tmpY[0], tmpRect);
   cvSetImageROI(tmpY[1], tmpRect);
   cvMul(u_vels, tmpX[0], u_res);
   cvMul(v_vels, tmpX[1], v_res);
   cvAdd(u_res, v_res, v_res, vels_mask);
   CvScalar sumX = cvSum(v_res);
   cvMul(u_vels, tmpY[0], u_res);
   cvMul(v_vels, tmpY[1], v_res);
   cvAdd(u_res, v_res, v_res, vels_mask);
   CvScalar sumY = cvSum(v_res);

   cvResetImageROI(tmpX[0]);
   cvResetImageROI(tmpX[1]);
   cvResetImageROI(tmpY[0]);
   cvResetImageROI(tmpY[1]);

   *mag = (sumY.val[0] + sumX.val[0])/2.0;

   return atan(sumY.val[0]/sumX.val[0]);

}

int ImpactMapModule::classifyRegion(IplImage *u_vels, IplImage *v_vels,
					 		IplImage *mag_map, 
							IplImage *tmpMatchImg[MAX_MATCH_WINS][2],
                     IplImage *vels_mask2, CvRect boundingBox, float *ratio,
						  	float *tangentAngle, float div)
{
   int max_region_index;
   float max_region_sum = -10000;
   CvRect tmpRect = boundingBox;
   CvScalar regionSum, divSum;
   float regionSumAcc = 0;
   CvRect oldROI = cvGetImageROI(u_vels);

   IplImage *u_res = cvCreateImage(cvSize(tmpRect.width, tmpRect.height),
                                     IPL_DEPTH_32F, 1);
   IplImage *v_res = cvCreateImage(cvSize(tmpRect.width, tmpRect.height),
                                     IPL_DEPTH_32F, 1);
   cvSetImageROI(u_vels, tmpRect);
   cvSetImageROI(v_vels, tmpRect);
   cvSetImageROI(vels_mask2, tmpRect);

   CvScalar size = cvSum(vels_mask2);

   cvSetImageROI(tmpMatchImg[DIV][0], tmpRect);
   cvSetImageROI(tmpMatchImg[DIV][1], tmpRect);

   cvMul(u_vels, tmpMatchImg[DIV][0], u_res);
   cvMul(v_vels, tmpMatchImg[DIV][1], v_res);

   cvAdd(u_res, v_res, v_res, vels_mask2);
   divSum = cvSum(v_res);

   cvResetImageROI(tmpMatchImg[DIV][0]);
   cvResetImageROI(tmpMatchImg[DIV][1]);


for(int n = 1; n < MAX_MATCH_WINS; n++){
      cvSetImageROI(tmpMatchImg[n][0], tmpRect);
      cvSetImageROI(tmpMatchImg[n][1], tmpRect);
      cvMul(u_vels, tmpMatchImg[n][0], u_res);
      cvMul(v_vels, tmpMatchImg[n][1], v_res);
      cvAdd(u_res, v_res, v_res, vels_mask2);
      regionSum = cvSum(v_res);
      regionSumAcc += regionSum.val[0];
      if(regionSum.val[0] > max_region_sum){
         max_region_sum = regionSum.val[0];
         max_region_index = n;
      }
      cvResetImageROI(tmpMatchImg[n][0]);
      cvResetImageROI(tmpMatchImg[n][1]);
   }

   float thisRatio;
   switch(max_region_index){
           case LEFT:
         *tangentAngle = 3*CV_PI/4.0 +
                     calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                           tmpMatchImg[UP_LEFT], tmpMatchImg[DOWN_LEFT],
                           tmpRect, &thisRatio);
        break;

           case RIGHT:
         *tangentAngle = 7*CV_PI/4.0 +
                     calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                               tmpMatchImg[DOWN_RIGHT], tmpMatchImg[UP_RIGHT],
                               tmpRect, &thisRatio);
         break;

      case UP:
         *tangentAngle = CV_PI/4.0 +
                 calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                                 tmpMatchImg[UP_RIGHT], tmpMatchImg[UP_LEFT],
                                 tmpRect, &thisRatio);
         break;

      case DOWN:
         *tangentAngle = 5*CV_PI/4.0 +
                 calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                               tmpMatchImg[DOWN_LEFT], tmpMatchImg[DOWN_RIGHT],
                               tmpRect, &thisRatio);
         break;
		
		case UP_LEFT:
         *tangentAngle = CV_PI/2.0 +
                 calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                                 tmpMatchImg[UP], tmpMatchImg[LEFT],
                                 tmpRect, &thisRatio);
         break;

      case UP_RIGHT:
         *tangentAngle = calcTangentAngle(u_vels, v_vels, u_res, v_res,
                         vels_mask2, tmpMatchImg[RIGHT], tmpMatchImg[UP],
                         tmpRect, &thisRatio);

         break;

      case DOWN_LEFT:
         *tangentAngle = CV_PI +
                 calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                                 tmpMatchImg[LEFT], tmpMatchImg[DOWN],
                                 tmpRect, &thisRatio);
         break;

      case DOWN_RIGHT:
         *tangentAngle = 3*CV_PI/2.0 +
                 calcTangentAngle(u_vels, v_vels, u_res, v_res, vels_mask2,
                              tmpMatchImg[DOWN], tmpMatchImg[RIGHT],
                              tmpRect, &thisRatio);

         break;
   }

   double min_val, max_div;

   *ratio = div/(((max_region_sum)/size.val[0]));

   cvSetImageROI(u_vels, oldROI);
   cvSetImageROI(v_vels, oldROI);
   cvSetImageROI(vels_mask2, oldROI);
   return max_region_index;
}


void ImpactMapModule::findMaxDivergence(IplImage *u_vels, IplImage *v_vels,
					 			IplImage *div_map, IplImage *u_tmp, IplImage *v_tmp,
								double *max_div, double *av_div, IplImage *vels_mask)
{
   //CvRect tmpRect;
   double min_val;
   float div;
   CvRect thisROI = cvGetImageROI(u_vels);
   IplImage *div_vels_mask = cvCreateImage(cvSize(thisROI.width,
                                             thisROI.height), IPL_DEPTH_8U, 1);
/*
   tmpRect.x = tmpPoints[0].x;
   tmpRect.y = tmpPoints[0].y;
   tmpRect.width = TEMPLATE_WIN_SIZE;
   tmpRect.height = TEMPLATE_WIN_SIZE;
*/
   IplImage *u_res = cvCreateImage(cvSize(thisROI.width, thisROI.height),
                                                            IPL_DEPTH_32F, 1);
   IplImage *v_res = cvCreateImage(cvSize(thisROI.width, thisROI.height),
                                                            IPL_DEPTH_32F, 1);

   cvSobel(u_vels, u_res, 1, 0, SOBEL_SIZE);
   cvSobel(v_vels, v_res, 0, 1, SOBEL_SIZE);
   cvAdd(u_res, v_res, div_map, vels_mask);
   CvScalar divSum = cvSum(div_map);

   cvCmpS(div_map, 0, div_vels_mask, CV_CMP_GT);
   cvSet(div_vels_mask, cvScalar(1));
   cvAnd(div_vels_mask, vels_mask, div_vels_mask);
   CvScalar av = cvAvg(div_map, div_vels_mask);
   cvMinMaxLoc(div_map, &min_val, max_div, &_minDivPoint, &_maxDivPoint,
                                                            div_vels_mask);
   *max_div = (*max_div/(SOBEL_SIZE*POINT_DIST));

   float diffx = _maxDivPoint.x - _prevMaxDivPoint.x;
   float diffy = _maxDivPoint.y - _prevMaxDivPoint.y;
   float minDiffx = _minDivPoint.x - _prevMinDivPoint.x;
   float minDiffy = _minDivPoint.y - _prevMinDivPoint.y;

   float localMaxDiffxAcc = (DIVPOS_KI*_maxDiffxAcc)/_divPosCount;
   float localMaxDiffyAcc = (DIVPOS_KI*_maxDiffyAcc)/_divPosCount;
   float localMinDiffxAcc = (DIVPOS_KI*_minDiffxAcc)/_divPosCount;
   float localMinDiffyAcc = (DIVPOS_KI*_minDiffyAcc)/_divPosCount;

	float newDiffx = DIVPOS_KP*(diffx) + DIVPOS_KD*(diffx - _prevDiffx) +
                                                             localMaxDiffxAcc;
   float newMinDiffx = DIVPOS_KP*(minDiffx) + DIVPOS_KD*(minDiffx -
                                              _prevMinDiffx) + localMinDiffxAcc;
   float newDiffy = DIVPOS_KP*(diffy) + DIVPOS_KD*(diffy - _prevDiffy) +
                                                               localMaxDiffyAcc;
   float newMinDiffy = DIVPOS_KP*(minDiffy) + DIVPOS_KD*(minDiffy -
                                              _prevMinDiffy) + localMinDiffyAcc;

   _prevDiffx = newDiffx;
   _prevMinDiffx = newMinDiffx;
   _prevDiffy = newDiffy;
   _prevMinDiffy = newMinDiffy;

   if(!_firstDivPosUpdate){
      _maxDivPoint.x = cvRound(_prevMaxDivPoint.x + newDiffx);
      _maxDivPoint.y = cvRound(_prevMaxDivPoint.y + newDiffy);
      _minDivPoint.x = cvRound(_prevMinDivPoint.x + newMinDiffx);
      _minDivPoint.y = cvRound(_prevMinDivPoint.y + newMinDiffy);
   }
   else{
      _divPosCount = 0;
      _maxDiffxAcc = 0;
      _maxDiffyAcc = 0;
      _minDiffxAcc = 0;
      _minDiffyAcc = 0;
      _firstDivPosUpdate = 0;
   }

   _divPosCount++;
   _maxDiffxAcc += diffx;
   _maxDiffyAcc += diffy;
   _minDiffxAcc += minDiffx;
   _minDiffyAcc += minDiffy;

   _prevMaxDivPoint = _maxDivPoint;
   _prevMinDivPoint = _minDivPoint;

   *av_div = av.val[0]/(SOBEL_SIZE*POINT_DIST);

   cvResetImageROI(u_tmp);
   cvResetImageROI(v_tmp);
   cvReleaseImage(&u_res);
   cvReleaseImage(&v_res);
   cvReleaseImage(&div_vels_mask);
}


CvRect ImpactMapModule::supportWindowFilter(IplImage *img, IplImage *vels_mask,
                         IplImage *new_vels_mask, CvSize supportWindowSize,
                         int *divCount)
{
   CvRect supportRect;
   CvScalar thisSum;
   BwImage local_mask(new_vels_mask);
   CvRect boundingBox = cvGetImageROI(vels_mask);
   float supportSum = supportWindowSize.width*supportWindowSize.height;
   int set = 0;
   int half_width = supportWindowSize.width/2;
   int half_height = supportWindowSize.height/2;
   int max_x = 0 , min_x = 320 , min_y = 240, max_y = 0;

   *divCount = 0;
   supportRect.width = supportWindowSize.width;
   supportRect.height = supportWindowSize.height;

   for(int i = boundingBox.x + half_width;
                      i < boundingBox.x + boundingBox.width-half_width; i++){
      for(int j = boundingBox.y + half_height;
                      j < boundingBox.y + boundingBox.height-half_height; j++){

         supportRect.x = i - half_width;
         supportRect.y = j - half_height;
         cvSetImageROI(vels_mask, supportRect);
         thisSum = cvSum(vels_mask);
         if(thisSum.val[0] < (supportSum)){
            local_mask[j][i] = (unsigned char) 0;
         }else{

            set = 1;
            if(i > max_x){
               max_x = i;
            }

            if(i < min_x){
               min_x = i;
            }

            if(j > max_y){
               max_y = j;
            }

            if(j < min_y){
               min_y = j;
				}

				local_mask[j][i] = (unsigned char) 1;
            (*divCount)++;
         }
         cvResetImageROI(vels_mask);
      }
   }

   int w = max_x - min_x;
   int h = max_y - min_y;
   int x = min_x;
   int y =min_y;

   if(set && max_x > min_x && max_y > min_y){
      boundingBox.x = x;
      boundingBox.y = y;
      boundingBox.width = w;
      boundingBox.height = h;
   }

   return boundingBox;
}


void ImpactMapModule::computeVels(CvPoint2D32f *points[2], IplImage *u_vels,
					 IplImage *v_vels, IplImage *mags, char *status0, char *status,
					 IplImage *vels_mask, int count)
{
   BwImageFloat local_u_vels(u_vels);
   BwImageFloat local_v_vels(v_vels);
   BwImage local_mask(vels_mask);
   BwImageFloat local_mags(mags);
   CvRect rect;
   int i, j;

   /*for(int i = 0; i < u_vels->width; i++){
      for(int j = 0; j < v_vels->height; j++){*/
    for(int k = 0; k < count; k++){
         i = (int) points[0][k].x/POINT_DIST;
         j = (int) points[0][k].y/POINT_DIST;

         float dx = (points[1][k].x - points[0][k].x)/POINT_DIST;
         float dy = (points[1][k].y - points[0][k].y)/POINT_DIST;
         float m = cvSqrt(dx*dx + dy*dy);
         if(status[k] && status0[k] && m > MAG_LOW_LIMIT && m < MAG_HIGH_LIMIT){
            local_mags[j][i] = m;
            local_u_vels[j][i] = dx;
            local_v_vels[j][i] = dy;
            local_mask[j][i] = 1;
         }else{
            local_u_vels[j][i] = 0;
            local_v_vels[j][i] = 0;
            local_mags[j][i] = 0;
            status[k] = 0;
            local_mask[j][i] = 0;
         }
   }

}


void ImpactMapModule::createDivergenceTemplate(IplImage *tmpMatchImg[2],
																		 int width, int height)
{
   tmpMatchImg[0] = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
   tmpMatchImg[1] = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
   BwImageFloat local_u_vels(tmpMatchImg[0]);
   BwImageFloat local_v_vels(tmpMatchImg[1]);
   float large_mag = cvSqrt(width/2*width/2 + height/2*height/2);
   int half_width = width/2;
   int half_height = height/2;

   for(int i = 0; i < width; i++){
      for(int j = 0; j < height; j++){
              float x_pos = i-width/2;
              float y_pos = j-height/2;
              float mag = cvSqrt((float) (x_pos*x_pos + y_pos*y_pos));
              if(mag == 0){
                  local_u_vels[j][i] = 0;
                  local_v_vels[j][i] = 0;
               }else{
                  if(x_pos == 0) local_u_vels[j][i] = 0;
                  else
                     local_u_vels[j][i]=(((x_pos))/mag)*(1.0/(2*fabs(x_pos)));
                  if(y_pos == 0) local_v_vels[j][i] = 0;
                  else local_v_vels[j][i]=((y_pos)/mag)*(1.0/(2*fabs(y_pos)));
               }
      }
   }
}


bool ImpactMapModule::open(Searchable& config)
{
	
	_videoIn.open(getName("imageIn"));
	_videoOut.open(getName("imageOut"));
	_dataOut.open(getName("dataOut"));

	start();
	return true;
}

bool ImpactMapModule::close()
{
	_videoIn.close();
   _videoOut.close();
	_dataOut.close();
	stop();
	return true;
}

bool ImpactMapModule::interruptModule()
{
	_videoIn.interrupt();
	_videoOut.interrupt();
	_dataOut.interrupt();
	return true;
}

bool ImpactMapModule::updateBuffer(IplImage *img)
{
    int i;
    IplImage* img2, *temp;
    ImageOf<PixelMono> *ypImg;
    CvRect comp_rect;
    double count;
    double magnitude;
    CvScalar color;


    // allocate images at the beginning
    if( _buf == NULL ) {
       _buf = (IplImage**)malloc(BUFF_SIZE*sizeof(_buf[0]));
       memset( _buf, 0, BUFF_SIZE*sizeof(_buf[0]));

      for( i = 0; i < BUFF_SIZE; i++ ) {
        cvReleaseImage( &_buf[i] );
        _buf[i] = cvCreateImage( _size, IPL_DEPTH_8U, 1 );
      }
   }
   cvCvtColor( img, _buf[_last], CV_BGR2GRAY ); // convert frame to grayscale

   _idx1 = _last;
   _idx2 = (_last + 1) % BUFF_SIZE; // index of (last - (N-1))th frame
   _last = _idx2;

   if(_idx1 > 2) _alive = 1;

}

bool ImpactMapModule::threadInit()
{

}


void ImpactMapModule::run()
{
	int winSize = WIN_SIZE;
   int flags = 0;
   double timestamp = (double)clock()/CLOCKS_PER_SEC;
   IplImage *uVels, *vVels, *uNorms, *vNorms;
   IplImage *swapTemp;
   IplImage *divMap, *ttcMap, *magMap;
   IplImage *dst, *dst1, *dst2, *dst3, *dst4;
   IplImage *regionMask, *magMask, *fullRegionMask, *divRegionMask;
   IplImage *pyramid, *prevPyramid;
   IplImage *tmpMatchImg[MAX_MATCH_WINS][2];
   IplImage *divCalcTmp[2];
   IplImage *velsMask1;
   IplImage *velsMask2;
   CvRect compRect;
   CvSize thisSize;
   CvSize levelSize[2];
   CvSize flowFieldSize;
   CvScalar lowMag = cvScalar(MAG_LOW_LIMIT);
   CvScalar highMag = cvScalar(MAG_HIGH_LIMIT);
   int divLevel = DIV_LEVEL;
   CvRect maxRect[4];
   CvPoint2D32f* points[2] = {0,0}, *swapPoints;
   char* status = 0, *status0 = 0;
   int count = 0;
   CvRect levelMaxRect;
   CvScalar *mag;
   CvRect boundingBox;
   float approachAngle;
   float minTTC, maxTTC, avgTTC;
   double maxDiv, div;
   float tangentAngle;
   CvSize windowSize;
   CvSize blockSize;
   CvSize shiftSize;
   CvSize maxRange;
   Bottle b;
   ImpactData dataToSend;
   CvPoint2D32f impactPoint;

   _ratioCount = 0;


   thisSize.width = 320;
   thisSize.height = 240;

   // size of flow field to compute
   flowFieldSize.width = thisSize.width/POINT_DIST;
   flowFieldSize.height = thisSize.height/POINT_DIST;

   createDivergenceTemplate(divCalcTmp, TEMPLATE_WIN_SIZE, TEMPLATE_WIN_SIZE);

   createDivergenceTemplate(tmpMatchImg[0], 320/POINT_DIST,240/POINT_DIST);
   createTranslationTemplates(tmpMatchImg, 320/POINT_DIST, 240/POINT_DIST);

   levelSize[0].width = thisSize.width;
   levelSize[1].width = thisSize.width/2;
   levelSize[0].height = thisSize.height;
   levelSize[1].height = thisSize.height/2;

   uVels = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   vVels = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   uNorms = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   vNorms = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   divMap = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   ttcMap = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   magMap = cvCreateImage(flowFieldSize, IPL_DEPTH_32F, 1);
   regionMask = cvCreateImage(levelSize[0], 8, 1);
   fullRegionMask = cvCreateImage(levelSize[0], 8, 1);
   magMask = cvCreateImage(levelSize[0], 8, 1);
   divRegionMask = cvCreateImage(levelSize[divLevel], IPL_DEPTH_8U, 1);

   pyramid = cvCreateImage( levelSize[0], 8, 1 );
   prevPyramid = cvCreateImage( levelSize[0], 8, 1 );
   points[0] = (CvPoint2D32f*)cvAlloc(MAX_POINTS*sizeof(points[0][0]));
   points[1] = (CvPoint2D32f*)cvAlloc(MAX_POINTS*sizeof(points[0][0]));
   mag = (CvScalar*)cvAlloc(MAX_POINTS*sizeof(CvScalar));
   status0 = (char*)cvAlloc(MAX_POINTS);
   status = (char*)cvAlloc(MAX_POINTS);


	dst = cvCreateImage( levelSize[divLevel], 8, 3 );
   dst2 = cvCreateImage( cvSize(320, 240), 8, 3);
   dst3 = cvCreateImage( cvSize(2*320, 240), 8, 3);
   dst4 = cvCreateImage( cvSize(320, 240), 8, 3);

   velsMask1 = cvCreateImage(flowFieldSize, IPL_DEPTH_8U, 1);
   velsMask2 = cvCreateImage(flowFieldSize, IPL_DEPTH_8U, 1);

   while(!_alive)
      yarp::os::Time::delay(0.1);

   while(true){
      dataToSend.updated = 0;
      //timestamp = (double)clock()/CLOCKS_PER_SEC;
      int thisIdx1 = _idx1-1;
      int thisIdx2 = _idx2-1;

      if(thisIdx1 == -1) thisIdx1 = BUFF_SIZE-1;
      if(thisIdx2 == -1) thisIdx2 = BUFF_SIZE-1;

      _maxSize = -1;
      int thisSize;
      _maxIndex = -1;

      if(_maxIndex == -1)
         _maxIndex = 0;
      else
         _lastMaxIndex = _maxIndex;


      blockSize.width = 5;
      blockSize.height = 5;

      shiftSize.width = 1;
      shiftSize.height = 1;

      maxRange.width = 25;
      maxRange.height = 25;

      windowSize.width = WIN_SIZE;
      windowSize.height = WIN_SIZE;

      cvZero(uVels);
      cvZero(vVels);

		_maxCompRect.x = 0; // - (POINT_DIST)/2;
      _maxCompRect.y = 0; // - (POINT_DIST)/2;
      _maxCompRect.width = (320); //320/3;// + (POINT_DIST);
      _maxCompRect.height = (240); // 240/3;// + (POINT_DIST);

      maxRect[0]   = _maxCompRect;
      maxRect[1].x = _maxCompRect.x/POINT_DIST;
      maxRect[1].y = _maxCompRect.y/POINT_DIST;
      maxRect[1].width = _maxCompRect.width/POINT_DIST;
      maxRect[1].height = _maxCompRect.height/POINT_DIST;

      if(_maxCompRect.width > windowSize.width &&
          _maxCompRect.height > windowSize.height &&
          _maxCompRect.width > TEMPLATE_WIN_SIZE &&
          _maxCompRect.height > TEMPLATE_WIN_SIZE){
          cvZero(regionMask);
          cvZero(velsMask1);
          cvZero(velsMask2);
          cvSet(regionMask, cvScalar(1));
          BwImage regionMaskLU(regionMask);

            count = 0;
            for(int i = maxRect[0].x; i < maxRect[0].x + maxRect[0].width;
                                                               i+=POINT_DIST){
               for(int j = maxRect[0].y; j < maxRect[0].y + maxRect[0].height;
                                                               j += POINT_DIST){
                  /*
                  if(!region_mask_lu[j][i])
                          status0[count]=0;
                  else*/
                  status0[count]=1;
                  points[0][count].x = i;
                  points[0][count].y = j;
                  count++;
               }
            }

            cvSetImageROI(ttcMap, maxRect[1]);
            cvSetImageROI(uVels, maxRect[1]);
            cvSetImageROI(vVels, maxRect[1]);
            cvSetImageROI(uNorms, maxRect[1]);
            cvSetImageROI(vNorms, maxRect[1]);
            cvSetImageROI(velsMask1, maxRect[1]);
            cvSetImageROI(velsMask2, maxRect[1]);
            cvSetImageROI(regionMask, _maxCompRect);
				cvSetImageROI(divMap, maxRect[1]);


            cvCalcOpticalFlowPyrLK(_buf[thisIdx1], _buf[thisIdx2],
                      prevPyramid, pyramid, points[0], points[1],
                      count, cvSize(winSize,winSize), 3, status, 0,
                     cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,100,0.0),
                     flags );

   //       flags |= CV_LKFLOW_PYR_A_READY; // | CV_LKFLOW_INITIAL_GUESSES;

            cvSetImageCOI(dst, 1);
            cvCopy(_buf[thisIdx2], dst);
            cvSetImageCOI(dst, 2);
            cvCopy(_buf[thisIdx2], dst);
            cvSetImageCOI(dst, 3);
            cvCopy(_buf[thisIdx2], dst);
            cvSetImageCOI(dst, 0);
            //cvZero(div_map);

            // compute vectors from points arra u_norm, v_norm,y
            computeVels(points, uVels, vVels, magMap, status0, status,
                            velsMask1, count);

            _divCount = 0;
            boundingBox = supportWindowFilter(dst, velsMask1, velsMask2,
                                    cvSize(SUPPORT_WIN_SIZE,SUPPORT_WIN_SIZE),
                                    &_divCount);
            //boundingBox = maxRect[1];
            //boundingBox.width = 5; //maxRect[1].width; //TEMPLATE_WIN_SIZE;
            //boundingBox.height = 5; //maxRect[1].height; //TEMPLATE_WIN_SIZE;

            _minDivPoint.x = _maxDivPoint.x = 0;
            _minDivPoint.y = _maxDivPoint.y = 0;


				if(_divCount > DIV_COUNT_THRESHOLD){
               CvPoint central = cvPoint(boundingBox.x+boundingBox.width/2,
                                         boundingBox.y+boundingBox.height/2);

               cvZero(divMap);
               findMaxDivergence(uVels, vVels, divMap,
                                 tmpMatchImg[DIV][0], tmpMatchImg[DIV][1],
                                  &maxDiv, &div, velsMask2);

               if(_interestPoint == IP_CENTER){
                  boundingBox.x = central.x - 2;
                  boundingBox.y = central.y - 2;
               }else if(_interestPoint == IP_MAX_DIV){
                  boundingBox.x = _maxDivPoint.x - 2;
                  boundingBox.y = _maxDivPoint.y - 2;
               }else if(_interestPoint == IP_MAXMIN_DIV){
                  boundingBox.x = (_minDivPoint.x + _maxDivPoint.x)/2 - 2;
                  boundingBox.y = (_minDivPoint.y + _maxDivPoint.y)/2 - 2;
               }
               else if(_interestPoint == IP_FULL_IMAGE){
                  boundingBox = maxRect[1];
               }


               boundingBox.width = 5;
               boundingBox.height = 5;

               int regionLabel = 0;
               float newMagSupp;
               float newRatio;
               regionLabel = classifyRegion(uVels, vVels, magMap,
                                             tmpMatchImg, velsMask2,
                                             boundingBox, &newRatio,
                                             &tangentAngle, maxDiv);

               approachAngle = computeApproachAngle(newRatio);


               float ttcC, ttcP;

               ttcC = calcTimeToContact(maxDiv, TTC_KP, TTC_KI, TTC_KD);
               ttcP = ttcC*cos(CV_PI/2.0 - approachAngle);

               CvPoint center = cvPoint(160,120);

					_maxDivPoint.x = (boundingBox.x + _maxDivPoint.x);
               _maxDivPoint.y = (boundingBox.y + _maxDivPoint.y);
               _minDivPoint.x = (boundingBox.x + _minDivPoint.x);
               _minDivPoint.y = (boundingBox.y + _minDivPoint.y);

               if(approachAngle > APPROACH_ANGLE_THRESHOLD &&
                     computeImpactLocation(&impactPoint, ttcP,
                                                approachAngle, tangentAngle,
                                                _maxDivPoint) != 0){

                     makeImpactLocationImage(dst2, ttcP, impactPoint);

                     _maxDivPoint.x*=POINT_DIST;
                     _maxDivPoint.y*=POINT_DIST;
                     _minDivPoint.x*=POINT_DIST;
                     _minDivPoint.y*=POINT_DIST;

                     cvRectangle(dst, cvPoint(boundingBox.x*POINT_DIST,
                           boundingBox.y*POINT_DIST),
                           cvPoint((boundingBox.x+boundingBox.width)*POINT_DIST,
                                 (boundingBox.y+boundingBox.height)*POINT_DIST),
                           CV_RGB(255, 0, 0));

                        center = cvPoint( ((boundingBox.x +
                                   boundingBox.width/2)*POINT_DIST),
                                   ((boundingBox.y +
                                     boundingBox.height/2)*POINT_DIST) );

                        makePyrVectorImage(dst, points, count, status,
                                           velsMask2, tangentAngle,
                                           center, boundingBox);


                        dataToSend.updated = 1;
                        dataToSend.imageWidth = dst->width;
                        dataToSend.imageHeight = dst->height;
                        dataToSend.interestPointX = _maxDivPoint.x;
                        dataToSend.interestPointY = _maxDivPoint.y;
                        dataToSend.ttc = ttcP;
                        dataToSend.approachAngle = approachAngle;
                        dataToSend.impactLocationX = impactPoint.x;
                        dataToSend.impactLocationY = impactPoint.y;
               }

            }else{
					cvZero(dst2);
               CvPoint v1 = cvPoint(dst2->width/2, 0);
               CvPoint v2 = cvPoint(dst2->width/2, dst2->height);
               CvPoint h1 = cvPoint(0, dst2->height/2);
               CvPoint h2 = cvPoint(dst2->width, dst2->height/2);
               cvLine(dst2, v1, v2, CV_RGB(0,255,255));
               cvLine(dst2, h1, h2, CV_RGB(0,255,255));
               _prevTTC = 0;
               _prevTTCDiff = 1;
               _accTTCDiff = 0;
               _prevRatio = 0;
               _ratioAcc = 0;
               _ratioCount = 0;
               _firstDivPosUpdate = 1;
               _prevMaxDivPoint = cvPoint(0,0);
            }

            mergeImages(dst, dst2, dst3);
            ImageOf<PixelBgr> ypImg;
            ypImg.wrapIplImage((void *) dst3);
            _videoOut.write(ypImg);

            /**      ImageOf<PixelBgr> ypImg2;
                     ypImg2.wrapIplImage((void *) dst2);
                     p2.write(ypImg2);
            **/
            Bottle b;
            packMessage(dataToSend, &b);
            _dataOut.write(b);


            for(int i = 0; i < count; i++){
               status[i] = 0;
            }

            cvZero(dst3);

            cvResetImageROI(uVels);
            cvResetImageROI(vVels);
            cvResetImageROI(velsMask1);
            cvResetImageROI(velsMask2);
            cvResetImageROI(uNorms);
            cvResetImageROI(vNorms);
            cvResetImageROI(magMask);
            cvResetImageROI(regionMask);
            cvResetImageROI(dst2);
				cvResetImageROI(dst3);
            cvResetImageROI(ttcMap);
            CV_SWAP( prevPyramid, pyramid, swapTemp );
            CV_SWAP( points[1], points[0], swapPoints);
         }
         _flowUpdate++;
   }

   cvReleaseImage(&divMap);
   cvReleaseImage(&magMap);
}



void ImpactMapModule::threadRelease()
{
}


bool ImpactMapModule::updateModule()
{
	yarp::sig::ImageOf<PixelRgb> ypImg;
	IplImage *image0, *image1;

	_videoIn.read(ypImg);
	image0 = (IplImage *) ypImg.getIplImage();
   image1 = (IplImage *) cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 3);
   cvResize(image0, image1);
   _size = cvSize(image1->width,image1->height);
   updateBuffer(image1);
	cvReleaseImage(&image1);
   //this_time =  yarp::os::Time::now() - this_time;

	return true;
}

#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include "public.h"
#include "basicMath.h"
#include "opticalflow.h"


inline static double square(int a)
{
	return a * a;
}

OpticalFlow::OpticalFlow(u32 features)
{
	pre = NULL;
	maxFeatures = nbFeatures = features;
	pre_features = new Vec2[nbFeatures];
	post_features = new Vec2[nbFeatures];
	//	vectors = new Vec2[nbFeatures];
	of_feature_found = new char[nbFeatures];
	of_feature_error = new f32[nbFeatures];
	startpositionx = 0;
	startpositiony = 0;
	motionx = 0;
	motiony = 0;
	countframe = 0;
	dist = new Vec2[nbFeatures];
	ori = new Vec2[nbFeatures];
	arrival = new Vec2[nbFeatures];
	externalFeatures = false;
	index = new u32[nbFeatures];
}


OpticalFlow::~OpticalFlow()
{
	delete [] dist;
	delete [] ori;
	delete [] pre_features;
	delete [] post_features;
	//	delete [] vectors;
	delete [] of_feature_found;
	delete [] of_feature_error;
	delete [] arrival;
	delete [] index;
	if(pre)cvReleaseImage(&pre);pre = NULL;
	if(dst)cvReleaseImage(&dst);dst = NULL;
	if(eig)cvReleaseImage(&eig);eig = NULL;
	if(tmp)cvReleaseImage(&tmp);tmp = NULL;
	if(pyr1)cvReleaseImage(&pyr1);pyr1 = NULL;
	if(pyr2)cvReleaseImage(&pyr2);pyr2 = NULL;
}

void OpticalFlow::Apply(IplImage *image)
{
	count = 0;
	CvSize res = cvGetSize(image);
	if(!pre){
		pre = cvCreateImage(res,IPL_DEPTH_8U,1);
		dst  = cvCreateImage(res,IPL_DEPTH_8U,1);	
		cvConvertImage(image, pre, CV_CVTIMG_FLIP);
		cvConvertImage(image, dst);//added by micha
		eig  = cvCreateImage(res,IPL_DEPTH_32F,1);
		tmp  = cvCreateImage(res,IPL_DEPTH_32F,1);
		pyr1 = cvCreateImage(res,IPL_DEPTH_8U,1);
		pyr2 = cvCreateImage(res,IPL_DEPTH_8U,1);
		pre->origin = dst->origin = eig->origin = tmp->origin = pyr1->origin = pyr2->origin = image->origin;
		return;
	}
	if(pre->width != res.width || pre->height != res.height){
		if(pre)cvReleaseImage(&pre);pre = NULL;
		if(dst)cvReleaseImage(&dst);dst = NULL;
		if(eig)cvReleaseImage(&eig);eig = NULL;
		if(tmp)cvReleaseImage(&tmp);tmp = NULL;
		if(pyr1)cvReleaseImage(&pyr1);pyr1 = NULL;
		if(pyr2)cvReleaseImage(&pyr2);pyr2 = NULL;
		return;
	}

	FOR(i,(u32)maxFeatures){
	  //  pre_features[i] = Vec2(0,0);
	  post_features[i] = Vec2(-1,-1);
	  of_feature_found[i] = 0;
	}
	if(!externalFeatures){
	  FOR(i,(u32)maxFeatures){
	    pre_features[i] = Vec2(-1,-1);
	  }
	}

	cvCopy(dst, pre);
	cvConvertImage(image, dst);
	if(!externalFeatures){
	
	  cvGoodFeaturesToTrack(pre, eig, tmp, pre_features, &nbFeatures, .01, 5, NULL);//was 0.1 insteaa
	}
	CvSize optical_flow_window = cvSize(3,3);
	CvTermCriteria criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
// 	TICTOC;
// 	printf("of ");
// 	tic;
	cvCalcOpticalFlowPyrLK(pre, dst, pyr1, pyr2, pre_features, post_features, nbFeatures, optical_flow_window, 5, of_feature_found, of_feature_error, criteria, 0 );

	//	toc;
// 	FOR(i,(u32)nbFeatures){
// 		vectors[i] = post_features[i] - pre_features[i];
// 	}

	countframe++;

	if (countframe < 10) {
		startpositionx = 0;
		startpositiony = 0;
	}
	if (countframe > 1000) countframe = 10;

	ComputeMotion();
	PruneOutliers2(0.05,0.05);
}

void OpticalFlow::ComputeMotion()
{
	FOR(i,(u32)nbFeatures){

		/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
		if ( of_feature_found[i] != 0 )
		{
		  //	if ( (post_features[i].x-pre_features[i].x)!=0.0 || (post_features[i].y-pre_features[i].y)!=0.0 ) {
			
		  if(post_features[i].x >= 0.0 && post_features[i].x >= 0.0 && pre_features[i].x >= 0.0 && pre_features[i].y >= 0.0){
		  // The arrows will be a bit too short for a nice visualization because of the high framerate
			// (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
				dist[count].x =post_features[i].x - pre_features[i].x;
				dist[count].y = post_features[i].y - pre_features[i].y;
				ori[count].x = pre_features[i].x;
				ori[count].y = pre_features[i].y;
				arrival[count].x = post_features[i].x;
				arrival[count].y = post_features[i].y;
				if(!externalFeatures){
				  index[count] = i;
				}
				else{
				  index[count] = index[i];
				}
				count++;
			}
		}
	}
	//	printf("cm %d of %d\n",count, nbFeatures);
}


void OpticalFlow::PruneOutliers()
{
	if(!count) return;
	//// elimination of extreme values
	/// sort by dist.x and erase 20% extreme values
	// bubble_sort
	bool has_swapped = 0;
	float temp;
	while (true) {
        has_swapped = false; //reset flag
		for(u32 i=0;i<count-1;i++) {
			if (dist[i].x > dist[i+1].x) { //if they are in the wrong order
				temp = dist[i].x; dist[i].x = dist[i+1].x; dist[i+1].x = temp;
				temp = dist[i].y; dist[i].y = dist[i+1].y; dist[i+1].y = temp;
				temp = ori[i].x; ori[i].x = ori[i+1].x; ori[i+1].x = temp; 
				temp = ori[i].y; ori[i].y = ori[i+1].y; ori[i+1].y = temp; 
                has_swapped = true; //we have swapped at least once, list may not be sorted yet
			}
		}
        //if no swaps were made during this pass, the list has been sorted
        if (has_swapped == 0)
			break;
	}

	//draw graph
	IplImage *graph = cvCreateImage(cvSize(200,200),IPL_DEPTH_8U,3); 
	cvSet( graph, cvScalar(1)); //cvSetZero(graph);
	FOR(i,(u32)count) {
		cvDrawLine(graph,cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),CV_RGB(0,255,0));
	}

	// erase 40% values
	FOR(i,(u32)count-count/5) {
		dist[i].x = dist[i+int(count/10)].x;
		dist[i].y = dist[i+int(count/10)].y;
		ori[i].x = ori[i+int(count/10)].x;
		ori[i].y = ori[i+int(count/10)].y;
	}
	int oldcount = count;
	count -= count/5;

	/// sort by dist.y and erase 20% extreme values
	// bubble_sort
	has_swapped = 0;
	temp;
	while (true) {
        has_swapped = false; //reset flag
		for(u32 i=0;i<count-1;i++) {
			if (dist[i].y > dist[i+1].y) { //if they are in the wrong order
				temp = dist[i].x; dist[i].x = dist[i+1].x; dist[i+1].x = temp;
				temp = dist[i].y; dist[i].y = dist[i+1].y; dist[i+1].y = temp;
				temp = ori[i].x; ori[i].x = ori[i+1].x; ori[i+1].x = temp; 
				temp = ori[i].y; ori[i].y = ori[i+1].y; ori[i+1].y = temp; 
                has_swapped = true; //we have swapped at least once, list may not be sorted yet
			}
		}
        //if no swaps were made during this pass, the list has been sorted
        if (has_swapped == 0)
			break;
	}
	
	// erase 40% values
	FOR(i,(u32)count-oldcount/5) {
		dist[i].x = dist[i+int(oldcount/10)].x;
		dist[i].y = dist[i+int(oldcount/10)].y;
		ori[i].x = ori[i+int(oldcount/10)].x;
		ori[i].y = ori[i+int(oldcount/10)].y;
	}
	count -= count/5;

	FOR(i,(u32)count) {		
		cvDrawLine(graph,cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),CV_RGB(255,0,0));
	}

	cvShowImage("graph",graph);
	cvReleaseImage(&graph);

}

void OpticalFlow::Draw(IplImage *image)
{
	/* For fun (and debugging :)), let's draw the flow field. */

	// evaluate mean and variance of angle and hypotenuse and draw arrows
	double meanangle=0, stdangle=0;
	double meanhypotenuse=0, stdhypotenuse=0;
	double meandistx=0, stddistx=0;
	double meandisty=0, stddisty=0;

	CvPoint p,q;

	//evaluate mean


	FOR(i,(u32)count){
		// Let's make the flow field look nice with arrows.
	  cvDrawLine(image,ori[i].to2d(),(ori[i]+dist[i]).to2d(),CV_RGB(255,0,0));
	  //		sprintf(text,"%d",index[i]);
	  //	cvPutText(image,text,ori[i].to2d(),&font,CV_RGB(255,0,0));
	  //int line_thickness = 1;
	  //CvScalar line_color = CV_RGB(255,0,0);
	  
		// The arrows will be a bit too short for a nice visualization because of the high framerate
		// (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		p.x = (int) ori[i].x;
		p.y = (int) ori[i].y;
		q.x = (int) (p.x + dist[i].x);
		q.y = (int) (p.y + dist[i].y);

		double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
		double hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );
		
		meanangle += angle;
		meanhypotenuse += hypotenuse;
		meandistx += dist[i].x;
		meandisty += dist[i].y;
	}

	meanangle /= (double)count;
	meanhypotenuse /= (double)count;
	meandistx /= (double)count;
	meandisty /= (double)count;
	
	//evaluate STD
	FOR(i,(u32)count){

		p.x = (int) ori[i].x;
		p.y = (int) ori[i].y;
		q.x = (int) (p.x + dist[i].x);
		q.y = (int) (p.y + dist[i].y);

		double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
		double hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

	stdangle += (angle - meanangle)*(angle - meanangle);
	stdhypotenuse += (hypotenuse - meanhypotenuse)*(hypotenuse - meanhypotenuse);
	stddistx += (dist[i].x - meandistx)*(dist[i].x - meandistx);
	stddisty += (dist[i].y - meandisty)*(dist[i].y - meandisty);
	}

	stdangle /= (double)count;
	stdhypotenuse /= (double)count;
	stddistx /= (double)count;
	stddisty /= (double)count;
	
	p.x = image->width/2; 
	p.y = image->height/2;

	q.x = (int) (p.x + 3 * meanhypotenuse * cos(meanangle));
	q.y = (int) (p.y + 3 * meanhypotenuse * sin(meanangle));

	// we discard the vectors that are too long (i.e. fast movements in the video)
	// the horizontal being a larger dimension, we multiply it by a factor

	motionx = meandistx;
	motiony = meandisty;

	int line_thickness = 3;
	
	startpositionx = startpositionx + meandistx;
	startpositiony = startpositiony + meandisty;
}
CvPoint2D32f OpticalFlow::GetMotion()
{
	return cvPoint2D32f(motionx, motiony);
}


CvPoint2D32f OpticalFlow::GetEstimatedPos()
{
	return cvPoint2D32f(startpositionx, startpositiony);
}

void OpticalFlow::Drawold(IplImage *image)
{
	/* For fun (and debugging :)), let's draw the flow field. */
	FOR(i,(u32)nbFeatures){
		/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
		if ( of_feature_found[i] == 0 )	continue;

		int line_thickness = 1;
		/* CV_RGB(red, green, blue) is the red, green, and blue components
		 * of the color you want, each out of 255.
		 */	
		CvScalar line_color = CV_RGB(255,0,0);

		/* Let's make the flow field look nice with arrows. */

		/* The arrows will be a bit too short for a nice visualization because of the high framerate
		 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		 */
		CvPoint p,q;
		p.x = (int) pre_features[i].x;
		p.y = (int) pre_features[i].y;
		q.x = (int) post_features[i].x;
		q.y = (int) post_features[i].y;

		double angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
		double hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

		/* Here we lengthen the arrow by a factor of three. */
		q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
		q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

		/* Now we draw the main line of the arrow. */
		/* "image" is the frame to draw on.
		 * "p" is the point where the line begins.
		 * "q" is the point where the line stops.
		 * "CV_AA" means antialiased drawing.
		 * "0" means no fractional bits in the center cooridinate or radius.
		 */
		cvLine( image, p, q, line_color, line_thickness, CV_AA, 0 );
		/* Now draw the tips of the arrow.  I do some scaling so that the
		 * tips look proportional to the main line of the arrow.
		 */			
		p.x = (int) (q.x + 9 * cos(angle + PIf / 4));
		p.y = (int) (q.y + 9 * sin(angle + PIf / 4));
		cvLine( image, p, q, line_color, line_thickness, CV_AA, 0 );
		p.x = (int) (q.x + 9 * cos(angle - PIf / 4));
		p.y = (int) (q.y + 9 * sin(angle - PIf / 4));
		cvLine( image, p, q, line_color, line_thickness, CV_AA, 0 );
	}
}
int OpticalFlow::SetFeaturesFromNeighbours(IplImage *img, Vec2 *features, s32 nbFeatures, u32 *ext_index){
  // TICTOC;
  if(!pre || !img)return 0;
  if(nbFeatures>maxFeatures){
    nbFeatures=maxFeatures;
  }
  CvSize optical_flow_window = cvSize(3,3);
  CvTermCriteria criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );
  //  printf("cross ");
  //  tic; 
 cvCalcOpticalFlowPyrLK(img, pre, pyr1, pyr2, features, pre_features, nbFeatures,
			 optical_flow_window, 5, of_feature_found, of_feature_error, criteria, 0 );
 //  toc; 
 this->nbFeatures = CleanFeatures(pre_features,of_feature_found,nbFeatures,ext_index);
 // printf("pre_left %d\n",this->nbFeatures);
  this->externalFeatures = true;
 return this->nbFeatures;
 
}


int OpticalFlow::CleanFeatures(Vec2 *features, char *valid, s32 size, u32 *ext_index){
  int top=0;
  FOR(i,size){
    if (valid[i]){// && i!=top) could be added if there are only a few misses 
      features[top].x=features[i].x;
      features[top].y=features[i].y;
      index[top] = ext_index[i];
      top++;
    }
  }
  return top;
}

void OpticalFlow::ComputeEnds(){
  FOR(i,count){
    arrival[i].x = ori[i].x+dist[i].x;
    arrival[i].y = ori[i].y+dist[i].y;
  }
  
}

int OpticalFlow::RetrieveEnd(Vec2 feat, u32 ind){
  int dist;
  for(int i=0;i<count;i++){
    if(index[i]==ind){
   //    dist = (feat-arrival[i]).lengthSquared();
//       if(dist<=100){
	return i;
	//     }
    }
  }
  return -1;
}


void OpticalFlow::PruneOutliers2(float ratio_small, float ratio_big)
{
	if(!count) return;
	//// elimination of extreme values
	/// sort by manhattan distance and erase ratio % extreme values
	// bubble_sort
	bool has_swapped = 0;
	float temp;
	int tempi;
	while (true) {
        has_swapped = false; //reset flag
	for(u32 i=0;i<count-1;i++) {
	  if (abs(dist[i].x)+abs(dist[i].y) > abs(dist[i+1].x)+abs(dist[i].y)) { //if they are in the wrong order
	    temp = dist[i].x; dist[i].x = dist[i+1].x; dist[i+1].x = temp;
	    temp = dist[i].y; dist[i].y = dist[i+1].y; dist[i+1].y = temp;
	    temp = ori[i].x; ori[i].x = ori[i+1].x; ori[i+1].x = temp; 
	    temp = ori[i].y; ori[i].y = ori[i+1].y; ori[i+1].y = temp;
	    // 				temp = arrival[i].x; arrival[i].x = arrival[i+1].x; arrival[i+1].x = temp; 
	    // 				temp = arrival[i].y; arrival[i].y = arrival[i+1].y; arrival[i+1].y = temp;
	    
	    tempi = index[i]; index[i] = index[i+1];index[i+1]=tempi;
	    
	    has_swapped = true; //we have swapped at least once, list may not be sorted yet
	  }
	}
	//if no swaps were made during this pass, the list has been sorted
	if (has_swapped == 0)
			break;
	}
	
	//draw graph
	IplImage *graph = cvCreateImage(cvSize(200,200),IPL_DEPTH_8U,3); 
	cvSet( graph, cvScalar(1)); //cvSetZero(graph);
	FOR(i,(u32)count) {
		cvDrawLine(graph,cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),CV_RGB(0,255,0));
	}

	// erase 40% values
	int offset = int(count*ratio_small);
	int last = count*(1-ratio_big);
	int j;
	FOR(i,offset){
	  j= last-i-1;;
	  dist[i].x = dist[j].x;
	  dist[i].y = dist[j].y;
	  ori[i].x = ori[j].x;
	  ori[i].y = ori[j].y;
// 	  arrival[i].x = arrival[j].x;
// 	  arrival[i].y = arrival[j].y;
	  index[i] = index[j];
	}
	count *= (1-ratio_small-ratio_big);

	/// sort by dist.y and erase 20% extreme values
	// bubble_sort
// 	has_swapped = 0;
// 	temp;
// 	while (true) {
//         has_swapped = false; //reset flag
// 		for(u32 i=0;i<count-1;i++) {
// 			if (dist[i].y > dist[i+1].y) { //if they are in the wrong order
// 				temp = dist[i].x; dist[i].x = dist[i+1].x; dist[i+1].x = temp;
// 				temp = dist[i].y; dist[i].y = dist[i+1].y; dist[i+1].y = temp;
// 				temp = ori[i].x; ori[i].x = ori[i+1].x; ori[i+1].x = temp; 
// 				temp = ori[i].y; ori[i].y = ori[i+1].y; ori[i+1].y = temp; 
//                 has_swapped = true; //we have swapped at least once, list may not be sorted yet
// 			}
// 		}
//         //if no swaps were made during this pass, the list has been sorted
//         if (has_swapped == 0)
// 			break;
// 	}
	
// 	// erase 40% values
// 	FOR(i,(u32)count-oldcount/5) {
// 		dist[i].x = dist[i+int(oldcount/10)].x;
// 		dist[i].y = dist[i+int(oldcount/10)].y;
// 		ori[i].x = ori[i+int(oldcount/10)].x;
// 		ori[i].y = ori[i+int(oldcount/10)].y;
// 	}
// 	count -= count/5;

	FOR(i,(u32)count) {		
		cvDrawLine(graph,cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),cvPoint(100-(int)dist[i].x,100-(int)dist[i].y),CV_RGB(255,0,0));
	}

	cvShowImage("graph",graph);
	cvReleaseImage(&graph);

}

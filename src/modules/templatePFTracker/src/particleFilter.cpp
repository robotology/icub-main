/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff, Carlo Ciliberto 
 *
 * Algorithm taken from R. Hess, A. Fern, "Discriminatively trained particle filters for complex multi-object tracking," cvpr, pp.240-247, 2009 IEEE Conference on Computer Vision and Pattern Recognition, 2009
 *
 * email:   vadim.tikhanoff@iit.it
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

#include <iCub/particleFilter.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

/**********************************************************/
int particle_cmp( const void* p1, const void* p2 ) 
{
    PARTICLEThread::particle* _p1 = (PARTICLEThread::particle*)p1;
    PARTICLEThread::particle* _p2 = (PARTICLEThread::particle*)p2;

    if( _p1->w > _p2->w )
        return -1;
    if( _p1->w < _p2->w )
        return 1;
    return 0;
}
/**********************************************************/

PARTICLEThread::~PARTICLEThread() 
{
    cout << "deleting dynamic objects" << endl;
    if(regions!=NULL)
        free_regions ( regions, num_objects );

    gsl_rng_free ( rng );
    free_histos ( ref_histos, num_objects);  
    if(particles != NULL)
        free ( particles);

    if (temp)
    {
        cout << "releasing temp" << endl;        
        cvReleaseImage(&temp);
    }
    cout << "finished particle thread" << endl;
}
/**********************************************************/
PARTICLEThread::PARTICLEThread() 
{
    firstFrame = true;
    num_objects = 0;
    regions = (CvRect **) malloc(sizeof(CvRect*));
    num_particles = PARTICLES;    /* number of particles */
    rng = gsl_rng_alloc( gsl_rng_mt19937 );
    gsl_rng_set( rng, (unsigned long)time(NULL) );
    i = 0;
    getTemplate = false;
    gotTemplate = false;
    sendTarget = false;
    getImage = false;
    particles = NULL;
    new_particles = NULL;
    temp = NULL;
    ref_histos = NULL;
    tpl = NULL;
    total = 0;
}
/**********************************************************/
void PARTICLEThread::setName(string module) 
{
    this->moduleName = module;
}

/**********************************************************/
bool PARTICLEThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */

    inputPortName = "/" + moduleName + "/image:i";
    imageIn.open( inputPortName.c_str() );

    outputPortName = "/" + moduleName + "/image:o";
    imageOut.open( outputPortName.c_str() );
    
    outputPortNameBlob = "/" + moduleName + "/blob/image:o";
    imageOutBlob.open( outputPortNameBlob.c_str() );

    num_objects = 0;
    init = true;

    updateNeeded=false;
    bestTempl.templ=NULL;
    bestTempl.w=0.0;
    return true;
}
/**********************************************************/
void PARTICLEThread::run() 
{
    while (isStopping() != true) { // the thread continues to run until isStopping() returns true

        getImage =  ( imageIn.getInputCount() > 0 );
        
        if ( !getImage )
            Time::delay(0.1);
        else{
            
            if (init)
            {//Get first RGB image to establish width, height:
                cout << "initializing" << endl;   
                initAll();     
            }
            else
            {
                iCubImage = imageIn.read();
                templateMutex.wait();
                imageIn.getEnvelope(targetTemp);
                templateMutex.post();
            }
 
            frame = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 3 );           
            cvCvtColor((IplImage*)iCubImage->getIplImage(), frame, CV_RGB2BGR);
            frame_blob = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

            if ( tpl != NULL )
            {
                IplImage *ipl_tpl=(IplImage *) tpl->getIplImage();
                tpl_width  = tpl->width();
                tpl_height = tpl->height();
                res_width  = width - tpl_width + 1;
                res_height = height - tpl_height + 1;
                cvReleaseImage(&temp);
                temp = cvCreateImage(cvGetSize(ipl_tpl), ipl_tpl->depth, ipl_tpl->nChannels );
                cvCvtColor(ipl_tpl, temp, CV_RGB2BGR);
                gotTemplate = true;
                firstFrame = true;
                if (num_objects>0)
                    free(*regions);
                num_objects = 0;
                delete tpl;
                tpl = NULL;
                //delete ipl_tpl;
                
                templateMutex.wait();
                while(tempList.size())
                {  
                    delete tempList.back().templ;
                    tempList.pop_back();
                }
                if(bestTempl.templ!=NULL)
                    delete bestTempl.templ;
                bestTempl.w=0.0;
                updateNeeded=false;
                templateMutex.post();
            }

            if (gotTemplate)
            {
                runAll(frame);
            }

            //send it all
            if (imageOut.getOutputCount()>0)
            {
                ImageOf<PixelBgr> &img = imageOut.prepare();
                img.resize(width,height);
                cvCopy( frame, (IplImage *) img.getIplImage());
                imageOut.write();
            }
            if (imageOutBlob.getOutputCount()>0)
            {
                ImageOf<PixelMono> &imgBlob = imageOutBlob.prepare();
                imgBlob.resize(width,height);
                cvCopy( frame_blob, (IplImage *) imgBlob.getIplImage());
                imageOutBlob.write();
            }
            cvReleaseImage(&frame);
            cvReleaseImage(&frame_blob);
        }
    } //while
}
/**********************************************************/
void PARTICLEThread::threadRelease() 
{
    cout << "cleaning up..." << endl;
    cout << "attempting to close ports" << endl;
    imageIn.interrupt();
    imageOut.interrupt();
    imageOutBlob.interrupt();    
    imageIn.close();
    imageOut.close();
    imageOutBlob.close();

    templateMutex.wait();
    while(tempList.size())
    {
        delete tempList.back().templ;
        tempList.pop_back();
    }

    if(bestTempl.templ!=NULL)
        delete bestTempl.templ;
    templateMutex.post();
    cout << "finished closing ports" << endl; 
}

/**********************************************************/
void PARTICLEThread::initAll()
{
    //here some initialization
    iCubImage = imageIn.read();
    width = iCubImage->width();
    height = iCubImage->height();
    init = false;
}
/**********************************************************/
void PARTICLEThread::runAll(IplImage *img)
{
    img_hsv = bgr2hsv( img );
    if (firstFrame)
    {
        w = img->width;
        h = img->height;
            
        while( num_objects == 0 )
        {
            num_objects = get_regionsImage( img, regions );
            if( num_objects == 0 )
                fprintf( stderr, "Problem! seg has issues\n" );
        }   
        if (ref_histos!=NULL)
            free_histos ( ref_histos, num_objects);        

        ref_histos = compute_ref_histos( img_hsv, *regions, num_objects );
        if (particles != NULL)
            free (particles);

        particles= init_distribution( *regions, ref_histos, num_objects, num_particles );
    }
    else
    {
        // perform prediction and measurement for each particle
        for( j = 0; j < num_particles; j++ ) 
        {
            particles[j] = transition( particles[j], w, h, rng );
            s = particles[j].s;
            particles[j].w = likelihood( img_hsv, cvRound(particles[j].y),
            cvRound( particles[j].x ),
            cvRound( particles[j].width * s ),
            cvRound( particles[j].height * s ),
            particles[j].histo );
        }
        // normalize weights and resample a set of unweighted particles
        normalize_weights( particles, num_particles );
        new_particles = resample( particles, num_particles );
        free( particles );
        particles = new_particles;
    }
    qsort( particles, num_particles, sizeof( PARTICLEThread::particle ), &particle_cmp );

    averageMutex.wait();
    for( j = 0; j < num_particles; j++ ) 
        average += particles[j].w;
        
    average = average/num_particles;
    averageMutex.post();

    //display most likely particle ------------------
    color = CV_RGB(255,0,0);
    targetMutex.wait();
    targetTemp.clear();
    //if (imageOut.getOutputCount()>0)
    display_particle( frame, particles[0], color, targetTemp );
    
    if (imageOutBlob.getOutputCount()>0)
        display_particleBlob( frame_blob, particles[0], targetTemp );
    targetMutex.post();
    trace_template( frame, particles[0] );

    cvReleaseImage(&img_hsv);
}
/**********************************************************/
void PARTICLEThread::setTemplate(ImageOf<PixelRgb> *_tpl)
{
    tpl = new ImageOf<PixelRgb> (*_tpl);
}
/**********************************************************/
void PARTICLEThread::pushTarget(Vector &target, Stamp &stamp)
{
    targetMutex.wait();
    for (size_t i=0; i< targetTemp.length(); i++ )
        target.push_back(targetTemp[i]);

    stamp=targetStamp;
    targetMutex.post();
}
/**********************************************************/
float PARTICLEThread::getAverage()
{
    averageMutex.wait();
    float tmpAv = 0.0;
    tmpAv = average;
    averageMutex.post();
    return tmpAv;
    //average.push_back(tmpAv);
}
/**********************************************************/
int PARTICLEThread::get_regionsImage( IplImage* frame, CvRect** regions ) 
{
    firstFrame = false;
    params p;
    CvRect* r;
    int i; //x1, y1, x2, y2, w, h;

    p.orig_img = cvCloneImage( frame );
    p.cur_img = NULL;
    p.n = 0;

    res = cvCreateImage( cvSize( res_width, res_height ), IPL_DEPTH_32F, 1 );

    cvMatchTemplate( frame, temp, res, CV_TM_SQDIFF );
    cvMinMaxLoc( res, &minval, &maxval, &minloc, &maxloc, 0 );

    cvReleaseImage( &(p.orig_img) );    //what was the purpose of this image?
    cvReleaseImage( &res );
    if( p.cur_img )
        cvReleaseImage( &(p.cur_img) );
    p.n = 1;
    // extract regions defined by user; store as an array of rectangles ----------
    
    r = (CvRect*) malloc ( p.n * sizeof( CvRect ) );

    for( i = 0; i < p.n; i++ )
        r[i] = cvRect( minloc.x, minloc.y, tpl_width, tpl_height );
        
    *regions = r;
    
    return p.n;
}
/**********************************************************/
PARTICLEThread::histogram** PARTICLEThread::compute_ref_histos( IplImage* frame, CvRect* regions, int n )
{
    histogram** histos = (histogram**) malloc( n * sizeof( histogram* ) );
    IplImage* tmp;
    int i;

    // extract each region from frame and compute its histogram 
    for( i = 0; i < n; i++ )
    {
        cvSetImageROI( frame, regions[i]);
        tmp = cvCreateImage( cvGetSize(frame), IPL_DEPTH_32F, 3 );
        cvCopy( frame, tmp, NULL );
        cvResetImageROI( frame );
        histos[i] = calc_histogram( &tmp, 1 );
        normalize_histogram( histos[i] );
        cvReleaseImage( &tmp );
    }
    return histos;
}
/**********************************************************/
PARTICLEThread::histogram* PARTICLEThread::calc_histogram( IplImage** imgs, int n ) 
{
    IplImage* img;
    histogram* histo;
    IplImage* h, * s, * v;
    float* hist;
    int i, r, c, bin;
    histo = (histogram*) malloc( sizeof(histogram) );
    histo->n = NH*NS + NV;
    hist = histo->histo;
    memset( hist, 0, histo->n * sizeof(float) );
    for( i = 0; i < n; i++ )
    {
        // extract individual HSV planes from image 
        img = imgs[i];
        h = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        s = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        v = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        cvSplit( img, h, s, v, NULL );

        // increment appropriate histogram bin for each pixel 
        for( r = 0; r < img->height; r++ )
            for( c = 0; c < img->width; c++ )
            {
                bin = histo_bin( pixval32f( h, r, c ),
                pixval32f( s, r, c ),
                pixval32f( v, r, c ) );
                hist[bin] += 1;
            }
        cvReleaseImage( &h );
        cvReleaseImage( &s );
        cvReleaseImage( &v );
    }
    return histo;
}
/**********************************************************/
void PARTICLEThread::free_histos( PARTICLEThread::histogram** histo, int n) 
{
    for (int i = 0; i < n; i++)    
        free (histo[i]);

    free(histo);
}
/**********************************************************/
void PARTICLEThread::free_regions( CvRect** regions, int n) 
{
    for (int i = 0; i < n; i++)    
        free (regions[i]);

   free(regions);
}
/**********************************************************/
void PARTICLEThread::normalize_histogram( PARTICLEThread::histogram* histo ) 
{
    float* hist;
    float sum = 0, inv_sum;
    int i, n;

    hist = histo->histo;
    n = histo->n;

    // compute sum of all bins and multiply each bin by the sum's inverse 
    for( i = 0; i < n; i++ )
        sum += hist[i];
    inv_sum = 1.0f / sum;
    for( i = 0; i < n; i++ )
        hist[i] *= inv_sum;
}
/**********************************************************/
int PARTICLEThread::histo_bin( float h, float s, float v )
{
    int hd, sd, vd;

    // if S or V is less than its threshold, return a "colorless" bin 
    vd = MIN( (int)(v * NV / V_MAX), NV-1 );
    if( s < S_THRESH  ||  v < V_THRESH )
        return NH * NS + vd;

    // otherwise determine "colorful" bin 
    hd = MIN( (int)(h * NH / H_MAX), NH-1 );
    sd = MIN( (int)(s * NS / S_MAX), NS-1 );
    return sd * NH + hd;
}
/**********************************************************/
PARTICLEThread::particle* PARTICLEThread::init_distribution( CvRect* regions, histogram** histos, int n, int p) 
{
    particle* particles;
    int np;
    float x, y;
    int i, j, width, height, k = 0;

    particles = (particle* )malloc( p * sizeof( particle ) );
    np = p / n;

    // create particles at the centers of each of n regions 
    for( i = 0; i < n; i++ ) {
        width = regions[i].width;
        height = regions[i].height;
        x = regions[i].x + (float)(width / 2);
        y = regions[i].y + (float) (height / 2);
        for( j = 0; j < np; j++ ) 
        {
            particles[k].x0 = particles[k].xp = particles[k].x = x;
            particles[k].y0 = particles[k].yp = particles[k].y = y;
            particles[k].sp = particles[k].s = 1.0;
            particles[k].width = width;
            particles[k].height = height;
            particles[k].histo = histos[i];
            particles[k++].w = 0;
        }
    }
    // make sure to create exactly p particles 
    i = 0;
    while( k < p ) 
    {
        width = regions[i].width;
        height = regions[i].height;
        x = regions[i].x + (float) (width / 2);
        y = regions[i].y + (float) (height / 2);
        particles[k].x0 = particles[k].xp = particles[k].x = x;
        particles[k].y0 = particles[k].yp = particles[k].y = y;
        particles[k].sp = particles[k].s = 1.0;
        particles[k].width = width;
        particles[k].height = height;
        particles[k].histo = histos[i];
        particles[k++].w = 0;
        i = ( i + 1 ) % n;
    }
    return particles;
}
/**********************************************************/
PARTICLEThread::particle PARTICLEThread::transition( const PARTICLEThread::particle &p, int w, int h, gsl_rng* rng ) 
{
    float x, y, s;
    PARTICLEThread::particle pn;

    // sample new state using second-order autoregressive dynamics 
    x = pfot_A1 * ( p.x - p.x0 ) + pfot_A2 * ( p.xp - p.x0 ) +
    pfot_B0 * (float)(gsl_ran_gaussian( rng, TRANS_X_STD )) + p.x0;
    pn.x = MAX( 0.0f, MIN( (float)w - 1.0f, x ) );
    y = pfot_A1 * ( p.y - p.y0 ) + pfot_A2 * ( p.yp - p.y0 ) +
    pfot_B0 * (float)(gsl_ran_gaussian( rng, TRANS_Y_STD )) + p.y0;
    pn.y = MAX( 0.0f, MIN( (float)h - 1.0f, y ) );
    s = pfot_A1 * ( p.s - 1.0f ) + pfot_A2 * ( p.sp - 1.0f ) +
    pfot_B0 * (float)(gsl_ran_gaussian( rng, TRANS_S_STD )) + 1.0f;
    pn.s = MAX( 0.1f, s );

    pn.xp = p.x;
    pn.yp = p.y;
    pn.sp = p.s;
    pn.x0 = p.x0;
    pn.y0 = p.y0;
    pn.width = p.width;
    pn.height = p.height;
    pn.histo = p.histo; 
    pn.w = 0;

    return pn;
}
/**********************************************************/
float PARTICLEThread::likelihood( IplImage* img, int r, int c, int w, int h, histogram* ref_histo ) 
{
    IplImage* tmp;
    histogram* histo;
    float d_sq;

    // extract region around (r,c) and compute and normalize its histogram 
    cvSetImageROI( img, cvRect( c - w / 2, r - h / 2, w, h ) );
    tmp = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 3 );
    cvCopy( img, tmp, NULL );
    cvResetImageROI( img );
    histo = calc_histogram( &tmp, 1 );
    cvReleaseImage( &tmp );
    normalize_histogram( histo );

    // compute likelihood as e^{\lambda D^2(h, h^*)} 
    d_sq = histo_dist_sq( histo, ref_histo );
    free( histo );
    return exp( -LAMBDA * d_sq );
}
/**********************************************************/
float PARTICLEThread::histo_dist_sq( histogram* h1, histogram* h2 ) 
{
    float* hist1, * hist2;
    float sum = 0;
    int i, n;

    n = h1->n;
    hist1 = h1->histo;
    hist2 = h2->histo;
    //  According the the Battacharyya similarity coefficient,
    //  D = \sqrt{ 1 - \sum_1^n{ \sqrt{ h_1(i) * h_2(i) } } }

    for( i = 0; i < n; i++ )
        sum += sqrt( hist1[i]*hist2[i] );

    if(sum<1.0f)
        return sqrt(1.0f - sum);
    else
    {
        // should be impossible to get here...
        fprintf(stdout,"Error in similarity computation!\n");
        return 1.0f-sum;
    }
}
/**********************************************************/
void PARTICLEThread::normalize_weights( particle* particles, int n ) 
{
    float sum = 0;
    int i;

    for( i = 0; i < n; i++ )
        sum += particles[i].w;
    for( i = 0; i < n; i++ )
        particles[i].w /= sum;
}
/**********************************************************/
PARTICLEThread::particle* PARTICLEThread::resample( particle* particles, int n ) 
{
    particle* _new_particles;
    int i, j, np, k = 0;

    //new_particles = (particle* )malloc( sizeof( particle ) );
    qsort( particles, n, sizeof( particle ), &particle_cmp );
    _new_particles = (particle* ) malloc( n * sizeof( particle ) );

    for( i = 0; i < n; i++ ) 
    {
        np = cvRound( particles[i].w * n );
        for( j = 0; j < np; j++ ) 
        {
            _new_particles[k++] = particles[i];
        if( k == n )
            goto exit;
        }
    }
    while( k < n )
        _new_particles[k++] = particles[0];

    exit:
    return _new_particles;
}
/**********************************************************/
void PARTICLEThread::display_particle( IplImage* img, const PARTICLEThread::particle &p, CvScalar color, Vector& target ) 
{
    int x0, y0, x1, y1;
    x0 = cvRound( p.x - 0.5 * p.s * p.width );
    y0 = cvRound( p.y - 0.5 * p.s * p.height );
    x1 = x0 + cvRound( p.s * p.width );
    y1 = y0 + cvRound( p.s * p.height );

    cvRectangle( img, cvPoint( x0, y0 ), cvPoint( x1, y1 ), color, 1, 8, 0 );
    
    cvCircle (img, cvPoint((x0+x1)/2, (y0+y1)/2), 3, CV_RGB(255,0 ,0),1);
    cvCircle (img, cvPoint( x0, y0), 3, CV_RGB(0, 255 ,0),1);
    cvCircle (img, cvPoint( x1, y1), 3, CV_RGB(0, 255 ,0),1);

    target.push_back((x0+x1)/2);
    target.push_back((y0+y1)/2);
    target.push_back(x0);
    target.push_back(y0);
    target.push_back(x1);
    target.push_back(y1);
}
/**********************************************************/
void PARTICLEThread::display_particleBlob( IplImage* img, const PARTICLEThread::particle &p, Vector& target ) 
{
    int x0, y0, x1, y1;
    x0 = cvRound( p.x - 0.5 * p.s * p.width );
    y0 = cvRound( p.y - 0.5 * p.s * p.height );
    x1 = x0 + cvRound( p.s * p.width );
    y1 = y0 + cvRound( p.s * p.height );

    int step       = img->widthStep/sizeof(uchar);
    uchar* data    = (uchar *)img->imageData;
    
    for (int i=0; i<img->height; i++)
    {
        for (int j=0; j<img->width; j++)
        {   
            data[i*step+j] = 0;
        }
    }
    cvRectangle(img, cvPoint(x0,y0), cvPoint(x1,y1), cvScalar(255,255, 255), CV_FILLED);
}
/**********************************************************/
void PARTICLEThread::trace_template( IplImage* img, const particle &p )
{
    if (p.w>TEMP_LIST_PARTICLE_THRES_HIGH)
    {
        //create the template image
        TemplateStruct t;
        t.w=p.w;
        t.templ=new ImageOf<PixelRgb>;
        t.templ->resize(cvRound(p.s*p.width),cvRound(p.s*p.height));

        cvSetImageROI(img,cvRect(cvRound(p.x-0.5*p.s*p.width),
                                 cvRound(p.y-0.5*p.s*p.height),
                                 cvRound(p.s*p.width),
                                 cvRound(p.s*p.height)));

        cvCvtColor(img,t.templ->getIplImage(),CV_BGR2RGB);

        cvResetImageROI(img);

        tempList.push_front(t);
        if (tempList.size()>TEMP_LIST_SIZE)
        {
            delete tempList.back().templ;
            tempList.pop_back();
        }
    }
    //keep in check the best template
    templateMutex.wait();
    //detect if the image tracker should be updated with a template from the stored ones
    if(p.w<TEMP_LIST_PARTICLE_THRES_LOW)
        updateNeeded=true;

    int best_idx=-1;
    float best_w=0.0;
    for(unsigned int i=0; i<tempList.size(); i++)
    {
        if(tempList[i].w<best_w)
        {
            best_idx=i;
            best_w=tempList[i].w;
        }
    }

    if(best_idx==-1)
    {
        updateNeeded=false;
    }
    else if(bestTempl.w!=best_w) //avoid releasing and copying the same image
    {
        if(bestTempl.templ==NULL)
            bestTempl.templ=new ImageOf<PixelRgb>;

        *bestTempl.templ=*tempList[best_idx].templ;
        bestTempl.w=best_w;
    }
    templateMutex.post();
}
/**********************************************************/
float PARTICLEThread::pixval32f(IplImage* img, int r, int c) 
{
    return ( (float*)(img->imageData + img->widthStep*r) )[c];
}
/**********************************************************/
IplImage* PARTICLEThread::bgr2hsv( IplImage* bgr ) 
{
    IplImage* bgr32f, * hsv;

    bgr32f = cvCreateImage( cvGetSize(bgr), IPL_DEPTH_32F, 3 );
    hsv = cvCreateImage( cvGetSize(bgr), IPL_DEPTH_32F, 3 );
    cvConvertScale( bgr, bgr32f, 1.0 / 255.0, 0 );
    cvCvtColor( bgr32f, hsv, CV_BGR2HSV );
    cvReleaseImage( &bgr32f );
    return hsv;
}
/**********************************************************/
TemplateStruct PARTICLEThread::getBestTemplate()
{
    TemplateStruct best;
    best.templ=NULL;
    best.w=0.0;

    templateMutex.wait();
    if(updateNeeded)
    {
        best.templ=new ImageOf<PixelRgb>;
        *best.templ=*bestTempl.templ;
        best.w=bestTempl.w;
    }
    templateMutex.post();
    return best;
}
/**********************************************************/
PARTICLEManager::PARTICLEManager() : PeriodicThread(0.02) 
{
    tpl = NULL;
}
/**********************************************************/
PARTICLEManager::~PARTICLEManager() { }
/**********************************************************/
void PARTICLEManager::setName(string module) 
{
    this->moduleName = module;
}
/**********************************************************/
bool PARTICLEManager::threadInit() 
{
    //create all ports
    inputPortNameTemp = "/" + moduleName + "/template/image:i";
    imageInTemp.open( inputPortNameTemp.c_str() );

    outputPortNameTarget = "/" + moduleName + "/target:o";
    target.open( outputPortNameTarget.c_str() );

    disparityPort.open(("/"+moduleName+"/triangulation:io").c_str());
    target3D.open(("/"+moduleName+"/target3d:o").c_str());

    particleThreadLeft = new PARTICLEThread();
    particleThreadRight = new PARTICLEThread();

    particleThreadLeft->setName((moduleName + "/left").c_str());
    particleThreadRight->setName((moduleName + "/right").c_str());

    shouldSend = false;
    particleThreadLeft->start();
    particleThreadRight->start();
    return true;
}
/**********************************************************/
void PARTICLEManager::run() 
{
    TemplateStruct templLeft,templRight;

    // if there is no need for a template update, the NULL pointers are returned
    templLeft=particleThreadLeft->getBestTemplate();
    templRight=particleThreadRight->getBestTemplate();

    ImageOf<PixelRgb> *tmp_tpl=templLeft.w>templRight.w?templLeft.templ:templRight.templ;

    // copy tmp_tpl whether it is NULL or not
    tpl =  tmp_tpl;
    tpl = imageInTemp.read(false);
    
    if (tpl!=NULL) 
    {
        shouldSend = true;
        particleThreadLeft->setTemplate(tpl);
        particleThreadRight->setTemplate(tpl);
    }
    Vector targetTemp;
    Stamp leftStamp,rightStamp;

    particleThreadLeft->pushTarget(targetTemp,leftStamp);
    particleThreadRight->pushTarget(targetTemp,rightStamp);

    float averageTempLeft = 0.0;
    float averageTempRight = 0.0;

    averageTempLeft = particleThreadLeft->getAverage();
    averageTempLeft = averageTempLeft *100;
        //fprintf(stdout,"the average is %lf \n",averageTempLeft *100);


    if (shouldSend)
    { 
        if (target.getOutputCount() > 0 && targetTemp.size() > 4 && 
            fabs(leftStamp.getTime()-rightStamp.getTime())<0.002) 
        {
            //fprintf(stdout,"output %lf %lf %lf %lf\n", targetTemp[0], targetTemp[1], targetTemp[2], targetTemp[3]);
            Stamp propagatedStamp(leftStamp.getCount(),0.5*(leftStamp.getTime()+rightStamp.getTime()));
            target.setEnvelope(propagatedStamp);        
            target.write(targetTemp);
        }
        if (disparityPort.getOutputCount() > 0 && targetTemp.size())
        {
            //fprintf(stdout,"getting ready to send\n");
            Bottle cmd, reply;
            cmd.addDouble(targetTemp[0]);
            cmd.addDouble(targetTemp[1]);
            cmd.addDouble(targetTemp[6]);
            cmd.addDouble(targetTemp[7]);
            fprintf(stdout,"output %lf %lf %lf %lf\n", targetTemp[0], targetTemp[1], targetTemp[6], targetTemp[7]);
            
            disparityPort.write(cmd,reply);
            
            Bottle targets;
            targets.addDouble(reply.get(0).asDouble());
            targets.addDouble(reply.get(1).asDouble());
            targets.addDouble(reply.get(2).asDouble());
            targets.addDouble(0.0);
            targets.addDouble(0.0);
            targets.addDouble(0.0);
            targets.addDouble(averageTempLeft);
            
            target3D.write(targets);
        }
    }
    //clear templates
    if(templLeft.templ!=NULL)
        delete templLeft.templ;
    if(templRight.templ!=NULL)
        delete templRight.templ;
}
/**********************************************************/
void PARTICLEManager::threadRelease() 
{
    cout << "cleaning up Manager..." << endl;
    cout << "attempting to close ports Manager" << endl;
    particleThreadLeft->stop();
    particleThreadRight->stop();

    imageInTemp.interrupt();
    target.interrupt();
    imageInTemp.close();
    target.close();

    disparityPort.interrupt();
    disparityPort.close();

    target3D.interrupt();
    target3D.close();
    
    cout << "deleteing thread " << endl;
    delete particleThreadLeft;
    delete particleThreadRight;
    cout << "finished closing ports Manager" << endl;
}
/**********************************************************/
bool PARTICLEModule::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    moduleName            = rf.check("name", 
                           Value("templatePFTracker"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());

    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerPortName.c_str())) 
    {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);
 
    /* create the thread and pass pointers to the module parameters */
    particleManager = new PARTICLEManager();

    /*pass the name of the module in order to create ports*/
    particleManager->setName(moduleName);    
    /* now start the thread to do the work */
    particleManager->start();
    
    
    return true ;     
}
/**********************************************************/
bool PARTICLEModule::interruptModule()
{
    handlerPort.interrupt();
    return true;
}
/**********************************************************/
bool PARTICLEModule::close()
{
    handlerPort.close();
    /* stop the thread */
    particleManager->stop();
    cout << "deleteing thread " << endl;
    delete particleManager;
    return true;
}
/**********************************************************/
bool PARTICLEModule::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") 
    {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="reset") 
    {
        cout << "reset has been asked "<< endl;
        particleManager->shouldSend=false;
        reply.addString("ok");
    }
    else
    {
        cout << "command not known - type help for more info" << endl;
    }
    return true;
}
/**********************************************************/
bool PARTICLEModule::updateModule()
{
    return true;
}
/**********************************************************/
double PARTICLEModule::getPeriod() 
{
    return 0.1;
}



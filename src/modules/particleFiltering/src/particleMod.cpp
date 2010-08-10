/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff, Andrew Dankers
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

#include "iCub/particleMod.h"
#include "yarp/os/impl/NameClient.h"

bool particleMod::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    moduleName            = rf.check("name", 
                           Value("particleMod"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
    setName(moduleName.c_str());

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
 
    //attachTerminal();                     // attach to terminal (maybe not such a good thing...)

    /* create the thread and pass pointers to the module parameters */
    particleThread = new PARTICLEThread();

    /*pass the name of the module in order to create ports*/
    particleThread->setName(moduleName);

    /* now start the thread to do the work */
    particleThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;      // let the RFModule know everything went well
}


bool particleMod::interruptModule()
{
    handlerPort.interrupt();
    return true;
}


bool particleMod::close()
{
    handlerPort.close();
    /* stop the thread */
    particleThread->stop();
    cout << "deleteing thread " << endl;
    delete particleThread;
    return true;
}


bool particleMod::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else{
			cout << "command not known - type help for more info" << endl;
	}
    return true;
}

/* Called periodically every getPeriod() seconds */

bool particleMod::updateModule() {
    return true;
}

double particleMod::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

PARTICLEThread::PARTICLEThread() {
    firstFrame = true;
	num_objects = 0;
    left_regions = (CvRect **) malloc(sizeof(CvRect*));
    right_regions = (CvRect **) malloc(sizeof(CvRect*));
	num_particles = PARTICLES;    /* number of particles */
	rng = gsl_rng_alloc( gsl_rng_mt19937 );
    gsl_rng_set( rng, time(NULL) );
    i = 0;
    getTemplate = false;
    getImageLeft = false;
    getImageRight = false;
    gotTemplate = false;
    sendTarget = false;
    particles_l = NULL;
    particles_r = NULL;
    new_particles_l = NULL;
    new_particles_r = NULL;
    temp = NULL;
    ref_histos_left = NULL;
    ref_histos_right = NULL;
}

void PARTICLEThread::setName(string module) {
    this->moduleName = module;
}

bool PARTICLEThread::threadInit() {
    /* initialize variables and create data-structures if needed */

    //create all ports
    inputPortNameTemp = "/" + moduleName + "/template/image:i";
    imageInTemp.open( inputPortNameTemp.c_str() );
    
    inputPortNameLeft = "/" + moduleName + "/left/image:i";
    imageInLeft.open( inputPortNameLeft.c_str() );

    inputPortNameRight = "/" + moduleName + "/right/image:i";
    imageInRight.open( inputPortNameRight.c_str() );

    outputPortNameLeft = "/" + moduleName + "/left/image:o";
    imageOutLeft.open( outputPortNameLeft.c_str() );
    
    outputPortNameRight = "/" + moduleName + "/right/image:o";
    imageOutRight.open( outputPortNameRight.c_str() );
    
    outputPortNameLeftBlob = "/" + moduleName + "/leftblob/image:o";
    imageOutLeftBlob.open( outputPortNameLeftBlob.c_str() );
    
    outputPortNameRightBlob = "/" + moduleName + "/rightblob/image:o";
    imageOutRightBlob.open( outputPortNameRightBlob.c_str() );

    outputPortNameTarget = "/" + moduleName + "/target:o";
    target.open( outputPortNameTarget.c_str() );

    init = true;

    return true;
}

void PARTICLEThread::run() {

    while (isStopping() != true) { // the thread continues to run until isStopping() returns true

        getImageLeft =  ( imageInLeft.getInputCount() > 0 );
        getImageRight = ( imageInRight.getInputCount() > 0 );
        
        if ( getImageLeft + getImageRight < 1)
            Time::delay(0.1);

        if ( getImageLeft + getImageRight > 1){
            
            if (init){//Get first RGB image to establish width, height:
                cout << "initializing" << endl;   
                initAll();     
            }
            iCubleft = imageInLeft.read();
            iCubright = imageInRight.read();
            
            left_frame = cvCreateImage(cvSize(width_L,height_L),IPL_DEPTH_8U, 3 );
	        cvCvtColor((IplImage*)iCubleft->getIplImage(), left_frame, CV_RGB2BGR);
	
	        right_frame = cvCreateImage(cvSize(width_R,height_R),IPL_DEPTH_8U, 3 );
	        cvCvtColor((IplImage*)iCubright->getIplImage(), right_frame, CV_RGB2BGR);

            left_frame_blob = cvCreateImage(cvSize(width_L,height_L),IPL_DEPTH_8U, 1 );
	
	        right_frame_blob = cvCreateImage(cvSize(width_R,height_R),IPL_DEPTH_8U, 1 );

            getTemplate =  ( imageInTemp.getInputCount() > 0 );
            
            if ( getTemplate > 0 ){
                tpl = imageInTemp.read(false);
      
                if ( tpl != NULL ){
                    
                    tpl_width  = tpl->width();
	                tpl_height = tpl->height();
                    cout << "got template " << tpl_width << " " << tpl_height << endl; 

                    res_width  = width_L - tpl_width + 1;
	                res_height = height_L - tpl_height + 1;
                    cvReleaseImage(&temp);
                    temp = cvCreateImage(cvSize(tpl_width,tpl_height),IPL_DEPTH_8U, 3 );
	                cvCvtColor((IplImage*)tpl->getIplImage(), temp, CV_RGB2BGR);
                    gotTemplate = true;
                    firstFrame = true;
                    num_objects = 0;
                }
            }
            if (gotTemplate){

                Vector targetTemp;
                runAll(left_frame, right_frame, targetTemp);

                if (target.getOutputCount() > 0){
                    cout << "sending target" << endl;
	                Vector& tmp = target.prepare();
                    tmp = targetTemp;
                    target.write();
                }
            }
            //send it all
            if (imageOutLeft.getOutputCount()>0){
                ImageOf<PixelBgr> &leftImage = imageOutLeft.prepare();
	            leftImage.resize(width_L,height_L);
                cvCopyImage( left_frame, (IplImage *) leftImage.getIplImage());
	            imageOutLeft.write();
            }
            if (imageOutRight.getOutputCount()>0){
                ImageOf<PixelBgr> &RightImage = imageOutRight.prepare();
	            RightImage.resize(width_R,height_R);
                cvCopyImage( right_frame, (IplImage *) RightImage.getIplImage());
	            imageOutRight.write();
            }
            if (imageOutLeftBlob.getOutputCount()>0){
                ImageOf<PixelMono> &leftImageBlob = imageOutLeftBlob.prepare();
	            leftImageBlob.resize(width_L,height_L);
                cvCopyImage( left_frame_blob, (IplImage *) leftImageBlob.getIplImage());
	            imageOutLeftBlob.write();
            }
            if (imageOutRightBlob.getOutputCount()>0){
                ImageOf<PixelMono> &RightImageBlob = imageOutRightBlob.prepare();
	            RightImageBlob.resize(width_R,height_R);
                cvCopyImage( right_frame_blob, (IplImage *) RightImageBlob.getIplImage());
	            imageOutRightBlob.write();
            }
      
            cvReleaseImage(&left_frame);
	        cvReleaseImage(&right_frame); 
            cvReleaseImage(&left_frame_blob);        
            cvReleaseImage(&right_frame_blob);
        }
    } //while
}

void PARTICLEThread::threadRelease() 
{
    cout << "cleaning up..." << endl;

    free_regions ( left_regions, num_objects );
    free_regions ( right_regions, num_objects );
    gsl_rng_free ( rng );
    free_histos ( ref_histos_left, num_objects);
    free_histos ( ref_histos_right, num_objects);
    free ( particles_l );
    free ( particles_r );
    cout << "attempting to close ports" << endl;
    imageInTemp.close();
    imageInLeft.close();
    imageInRight.close();
    imageOutLeft.close();
    imageOutRight.close();
    imageOutLeftBlob.close();
    imageOutRightBlob.close();
    target.close();
    
    if (temp){
        cout << "releasing temp" << endl;        
        cvReleaseImage(&temp);
    }
    yarp::os::impl::NameClient::removeNameClient();
    cout << "finished closing ports" << endl;
}

void PARTICLEThread::initAll()
{
    //here some initialization
    iCubleft = imageInLeft.read();
    iCubright = imageInRight.read();
    width_L = iCubleft->width();
    height_L = iCubleft->height();
    width_R = iCubright->width();
    height_R = iCubright->height();
    init = false;
    
}

void PARTICLEThread::runAll(IplImage *left, IplImage *right, Vector& target){
    
    left_hsv = bgr2hsv( left );
    right_hsv = bgr2hsv( right );

    if (firstFrame){
        w = left->width;
        h = left->height;
        while( num_objects == 0 ){
            num_objects = get_regionsImage( left, left_regions );
            num_objects = get_regionsImage( right, right_regions );
            if( num_objects == 0 )
                fprintf( stderr, "Problem! seg has issues\n" );
        }	
        ref_histos_left = compute_ref_histos( left_hsv, *left_regions, num_objects );
        ref_histos_right = compute_ref_histos( right_hsv, *right_regions, num_objects );

        particles_l = init_distribution( *left_regions, ref_histos_left, num_objects, num_particles );
        particles_r = init_distribution( *right_regions, ref_histos_right, num_objects, num_particles );
    }
    else{
    // perform prediction and measurement for each particle --------------------
        for( j = 0; j < num_particles; j++ ) {
            particles_l[j] = transition( particles_l[j], w, h, rng );
            s = particles_l[j].s;
            particles_l[j].w = likelihood( left_hsv, cvRound(particles_l[j].y),
            cvRound( particles_l[j].x ),
            cvRound( particles_l[j].width * s ),
            cvRound( particles_l[j].height * s ),
            particles_l[j].histo );
        }
    // normalize weights and resample a set of unweighted particles----------------
        normalize_weights( particles_l, num_particles );
        new_particles_l = resample( particles_l, num_particles );
        free( particles_l );
        particles_l = new_particles_l;

        for( j = 0; j < num_particles; j++ ) {
            particles_r[j] = transition( particles_r[j], w, h, rng );
            s = particles_r[j].s;
            particles_r[j].w = likelihood( right_hsv, cvRound(particles_r[j].y),
            cvRound( particles_r[j].x ),
            cvRound( particles_r[j].width * s ),
            cvRound( particles_r[j].height * s ),
            particles_r[j].histo );
        }
        // normalize weights and resample a set of unweighted particles----------------
        normalize_weights( particles_r, num_particles );
        new_particles_r = resample( particles_r, num_particles );
        free( particles_r );
        particles_r = new_particles_r;
    }

    qsort( particles_l, num_particles, sizeof( PARTICLEThread::particle ), &particle_cmp );
    qsort( particles_r, num_particles, sizeof( PARTICLEThread::particle ), &particle_cmp );

    // display most likely particle ------------------
    color = CV_RGB(255,0,0);
    display_particle( left_frame, particles_l[0], color, target );
    display_particle( right_frame, particles_r[0], color, target );

    display_particleBlob( left_frame_blob, particles_l[0], target );
    display_particleBlob( right_frame_blob, particles_r[0], target );

    //cvWaitKey( 5 );
    cvReleaseImage(&left_hsv);
    cvReleaseImage(&right_hsv);
}

int PARTICLEThread::get_regionsImage( IplImage* frame, CvRect** regions ) {
    firstFrame = false;
    params p;
    CvRect* r;
    int i, x1, y1, x2, y2, w, h;

    p.orig_img = cvCloneImage( frame );
    p.cur_img = NULL;
    p.n = 0;

    res = cvCreateImage( cvSize( res_width, res_height ), IPL_DEPTH_32F, 1 );

    cvMatchTemplate( frame, temp, res, CV_TM_SQDIFF );
    cvMinMaxLoc( res, &minval, &maxval, &minloc, &maxloc, 0 );

    cvReleaseImage( &(p.orig_img) );
    cvReleaseImage( &res );
    if( p.cur_img )
        cvReleaseImage( &(p.cur_img) );
    p.n = 1;
    // extract regions defined by user; store as an array of rectangles ----------
    r = (CvRect*) malloc ( p.n * sizeof( CvRect ) );

    for( i = 0; i < p.n; i++ ) {
        r[i] = cvRect( minloc.x, minloc.y, tpl_width, tpl_height );
    }
    *regions = r;
    
    return p.n;
}

PARTICLEThread::histogram** PARTICLEThread::compute_ref_histos( IplImage* frame, CvRect* regions, int n ){
  
    histogram** histos = (histogram**) malloc( n * sizeof( histogram* ) );
    IplImage* tmp;
    int i;

    // extract each region from frame and compute its histogram 
    for( i = 0; i < n; i++ ){
        cvSetImageROI( frame, regions[i]);
        tmp = cvCreateImage( cvGetSize( frame ), IPL_DEPTH_32F, 3 );
        cvCopy( frame, tmp, NULL );
        cvResetImageROI( frame );
        histos[i] = calc_histogram( &tmp, 1 );
        normalize_histogram( histos[i] );
        cvReleaseImage( &tmp );
    }
    return histos;
}

PARTICLEThread::histogram* PARTICLEThread::calc_histogram( IplImage** imgs, int n ) {
    IplImage* img;
    histogram* histo;
    IplImage* h, * s, * v;
    float* hist;
    int i, r, c, bin;
    histo = (histogram*) malloc( sizeof(histogram) );
    histo->n = NH*NS + NV;
    hist = histo->histo;
    memset( hist, 0, histo->n * sizeof(float) );
    for( i = 0; i < n; i++ ){
        // extract individual HSV planes from image 
        img = imgs[i];
        h = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        s = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        v = cvCreateImage( cvGetSize(img), IPL_DEPTH_32F, 1 );
        cvCvtPixToPlane( img, h, s, v, NULL );

        // increment appropriate histogram bin for each pixel 
        for( r = 0; r < img->height; r++ )
            for( c = 0; c < img->width; c++ ){
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

void PARTICLEThread::free_histos( PARTICLEThread::histogram** histo, int n) {
    for (int i = 0; i < n; i++)    
        free (histo[i]);

    free(histo);
}

void PARTICLEThread::free_regions( CvRect** regions, int n) {
    for (int i = 0; i < n; i++)    
        free (regions[i]);

    free(regions);
}


void PARTICLEThread::normalize_histogram( PARTICLEThread::histogram* histo ) {
    float* hist;
    float sum = 0, inv_sum;
    int i, n;

    hist = histo->histo;
    n = histo->n;

    // compute sum of all bins and multiply each bin by the sum's inverse 
    for( i = 0; i < n; i++ )
        sum += hist[i];
    inv_sum = 1.0 / sum;
    for( i = 0; i < n; i++ )
        hist[i] *= inv_sum;
}

int PARTICLEThread::histo_bin( float h, float s, float v ){

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

PARTICLEThread::particle* PARTICLEThread::init_distribution( CvRect* regions, histogram** histos, int n, int p) {
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
        x = regions[i].x + width / 2;
        y = regions[i].y + height / 2;
        for( j = 0; j < np; j++ ) {
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
    while( k < p ) {
        width = regions[i].width;
        height = regions[i].height;
        x = regions[i].x + width / 2;
        y = regions[i].y + height / 2;
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

PARTICLEThread::particle PARTICLEThread::transition( PARTICLEThread::particle p, int w, int h, gsl_rng* rng ) {
	
    float x, y, s;
    PARTICLEThread::particle pn;

    // sample new state using second-order autoregressive dynamics 
    x = pfot_A1 * ( p.x - p.x0 ) + pfot_A2 * ( p.xp - p.x0 ) +
    pfot_B0 * gsl_ran_gaussian( rng, TRANS_X_STD ) + p.x0;
    pn.x = MAX( 0.0, MIN( (float)w - 1.0, x ) );
    y = pfot_A1 * ( p.y - p.y0 ) + pfot_A2 * ( p.yp - p.y0 ) +
    pfot_B0 * gsl_ran_gaussian( rng, TRANS_Y_STD ) + p.y0;
    pn.y = MAX( 0.0, MIN( (float)h - 1.0, y ) );
    s = pfot_A1 * ( p.s - 1.0 ) + pfot_A2 * ( p.sp - 1.0 ) +
    pfot_B0 * gsl_ran_gaussian( rng, TRANS_S_STD ) + 1.0;
    pn.s = MAX( 0.1, s );

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
float PARTICLEThread::likelihood( IplImage* img, int r, int c, int w, int h, histogram* ref_histo ) {
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

float PARTICLEThread::histo_dist_sq( histogram* h1, histogram* h2 ) {
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

    return 1.0 - sum;
}


void PARTICLEThread::normalize_weights( particle* particles, int n ) {
    float sum = 0;
    int i;

    for( i = 0; i < n; i++ )
        sum += particles[i].w;
    for( i = 0; i < n; i++ )
        particles[i].w /= sum;
}

PARTICLEThread::particle* PARTICLEThread::resample( particle* particles, int n ) {
    particle* new_particles;
    int i, j, np, k = 0;

    //new_particles = (particle* )malloc( sizeof( particle ) );

    qsort( particles, n, sizeof( particle ), &particle_cmp );

    new_particles = (particle* ) malloc( n * sizeof( particle ) );

    for( i = 0; i < n; i++ ) {
        np = cvRound( particles[i].w * n );
        for( j = 0; j < np; j++ ) {
            new_particles[k++] = particles[i];
        if( k == n )
            goto exit;
        }
    }
    while( k < n )
        new_particles[k++] = particles[0];

    exit:
    return new_particles;
}

int particle_cmp( const void* p1, const void* p2 ) {
    PARTICLEThread::particle* _p1 = (PARTICLEThread::particle*)p1;
    PARTICLEThread::particle* _p2 = (PARTICLEThread::particle*)p2;

    if( _p1->w > _p2->w )
        return -1;
    if( _p1->w < _p2->w )
        return 1;
    return 0;
}

void PARTICLEThread::display_particle( IplImage* img, PARTICLEThread::particle p, CvScalar color, Vector& target ) {
    int x0, y0, x1, y1;
    x0 = cvRound( p.x - 0.5 * p.s * p.width );
    y0 = cvRound( p.y - 0.5 * p.s * p.height );
    x1 = x0 + cvRound( p.s * p.width );
    y1 = y0 + cvRound( p.s * p.height );

    cvRectangle( img, cvPoint( x0, y0 ), cvPoint( x1, y1 ), color, 1, 8, 0 );
    cvCircle (img, cvPoint((x0+x1)/2, (y0+y1)/2), 3, CV_RGB(255,0 ,0),1);

    target.push_back((x0+x1)/2);
    target.push_back((y0+y1)/2);
}

void PARTICLEThread::display_particleBlob( IplImage* img, PARTICLEThread::particle p, Vector& target ) {
    int x0, y0, x1, y1;
    x0 = cvRound( p.x - 0.5 * p.s * p.width );
    y0 = cvRound( p.y - 0.5 * p.s * p.height );
    x1 = x0 + cvRound( p.s * p.width );
    y1 = y0 + cvRound( p.s * p.height );

    int step       = img->widthStep/sizeof(uchar);
    uchar* data    = (uchar *)img->imageData;
    
    for (int i=0; i<img->height; i++){
		for (int j=0; j<img->width; j++){	
            data[i*step+j] = 0;
        }
    }
    cvRectangle(img, cvPoint(x0,y0), cvPoint(x1,y1), cvScalar(255,255, 255), CV_FILLED);
}

float PARTICLEThread::pixval32f(IplImage* img, int r, int c) {

    return ( (float*)(img->imageData + img->widthStep*r) )[c];
}

IplImage* PARTICLEThread::bgr2hsv( IplImage* bgr ) {
    IplImage* bgr32f, * hsv;

    bgr32f = cvCreateImage( cvGetSize(bgr), IPL_DEPTH_32F, 3 );
    hsv = cvCreateImage( cvGetSize(bgr), IPL_DEPTH_32F, 3 );
    cvConvertScale( bgr, bgr32f, 1.0 / 255.0, 0 );
    cvCvtColor( bgr32f, hsv, CV_BGR2HSV );
    cvReleaseImage( &bgr32f );
    return hsv;
}


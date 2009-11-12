#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

//MY INCLUDES
//client of:
#include <zdfio.h>
#include "objRec.h"

#define NCLASSES 4
#define NN_WIDTH 50
#define NN_HEIGHT 50

using namespace nnfw;
using namespace std;
using namespace iCub::contrib::primateVision;



iCub::contrib::primateVision::ObjRecServer::ObjRecServer( string*cfg_ )
{
  
  cfg=cfg_;
  start();

}

iCub::contrib::primateVision::ObjRecServer::~ObjRecServer()
{
  
}

void iCub::contrib::primateVision::ObjRecServer::run()
{

  Property prop;
  prop.fromConfigFile(cfg->c_str());
  double threshold = prop.findGroup("OBJREC").find("THRESHOLD").asDouble();
  momentum  = prop.findGroup("OBJREC").find("MOMENTUM").asDouble();
  learnRate = prop.findGroup("OBJREC").find("LEARNRATE").asDouble();
  int max_unknowns = prop.findGroup("OBJREC").find("MAX_UNKNOWNS").asInt();

  //probe ZDFServer:
  Port inPort_s;
  inPort_s.open("/objRecServer/input/serv_params"); 
  Network::connect("/objRecServer/input/serv_params", "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/objRecServer/input/serv_params");
  BinPortable<ZDFServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ZDFServerParams zsp = server_response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl;
  int zdf_width = zsp.width;
  int zdf_height = zsp.height;
  int zdf_psb  = zsp.psb;
  IppiSize zdf_size={zdf_width,zdf_height};
  

  //To get data from zdfserver:
  BufferedPort<ZDFServerData > inPort_zdfdata;  
  inPort_zdfdata.open("/objRecServer/input/zdfdata");
  Network::connect("/zdfserver/output/data" , "/objRecServer/input/zdfdata");
  ZDFServerData *zdfData;
 


#if ORIENT
  //For getting orientation data:
  BufferedPort<Bottle> inPort_or;  
  inPort_or.open("/objRec2/input/or");  
  Network::connect("/ocsserver_0/output/featOr" , "/objRec2/input/or");
  Bottle *inBot_or;
  //For getting edge data:
  BufferedPort<Bottle> inPort_e;  
  inPort_e.open("/objRec2/input/edge");  
  Network::connect("/ocsserver_0/output/M" , "/objRec2/input/edge");
  Bottle *inBot_e;
#endif


  int max_width = zdf_width/2;
  int polar_width = 96;
  int polar_height = 128;
  IppiSize polar_size = {polar_width,polar_height};


 printf("%d %d %d\n",polar_width,polar_height,max_width);

  int psb_8u,pol_psb_8u;
  Ipp8u  *pol_seg                  = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled           = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled_stretched = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  inv = cvCreateImage(cvSize(zdf_width,zdf_height), IPL_DEPTH_8U , 1 );
  Ipp8u  *tex_in,*dog_in;


  int up_pix;
  int rightmost_x,rightmost_y;
  int stretch = max_width;
  int rightmost = max_width;
  int roll,rollup = 0;


#if ORIENT
  Ipp8u *orient = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
  Ipp8u *edge   = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
  Ipp8u *orient_in,*edge_in;
  Ipp32s histo[NUM_BINS];
  Ipp32s levels[NUM_BINS+1];
  double av, X, Y;
  int av_orient;
  iCub::contrib::primateVision::Display *d_in_o   = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"IN O");
  iCub::contrib::primateVision::Display *d_in_e   = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"IN E");  
#endif

  //set up NN:
  //initialise the network with a random see
  int seed =time(0);
  nnfw::Random::setSeed(seed); 
  //load the saved model
  net = loadXML( "/home/andrew/src/iCub/src/primateVision/objRec/data/learnedModel.xml" ); 
  const ClusterVec& cl = net->clusters();  //configure the NN clusters
  for( nnfw::u_int i=0; i<cl.size(); i++ ){                
    cl[i]->inputs().zeroing();
    cl[i]->outputs().zeroing();                                
  }
  //configure network
  loadNet();
  
  //For resizing befor NN query:
  temp = cvCreateImage(cvSize(NN_WIDTH,NN_HEIGHT), IPL_DEPTH_8U , 1 );

  const char *label;
  const char *oldlabel;
  double bestval = 0.0;
  double radius = 0.03;
  int unknowns = 0;
  
  
  
  //make server probe replier:
  //Server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/objRecServer/output/serv_params");
  ObjRecServerParams rsp;
  rsp.width = zdf_width;
  rsp.height = zdf_height;
  rsp.mos_width = zsp.mos_width;
  rsp.mos_height = zsp.mos_height;
  rsp.nclasses = NCLASSES;
  rsp.psb = zdf_psb;
  //Replier:
  ObjRecServerReplyParamProbe server_replier;
  server_replier.reply=rsp;
  outPort_s.setReplier(server_replier); 


  //make online output port:
  BufferedPort<ObjRecServerData> outPort_objData; 
  outPort_objData.open("/objRecServer/output/objData"); 
  
  

  label = "unknown";


  
  printf("ObjRecServer: begin..\n");
  //main event loop:
  while (1){
    

    //get seg and CoG from ZDFSERVER:
    zdfData = inPort_zdfdata.read(); //blocking
    tex_in = (Ipp8u*) zdfData->tex.getRawImage();
    dog_in = (Ipp8u*) zdfData->dog.getRawImage();


    //polar transform tex seg about CoG:
    cart2polar_8u(dog_in,zdf_psb,zdf_size, 
		  pol_seg,pol_psb_8u,polar_size, 
		  zdfData->cog_x,zdfData->cog_y);

    //get rightmost pixel in polar map:
    getMaxRightXY_8u(pol_seg,pol_psb_8u,polar_size,&rightmost_x,&rightmost_y);
    up_pix = rightmost_y; //if simply aligning on most extreme protrusion.

    if (up_pix > polar_size.height/2){
      up_pix =  -(polar_size.height - up_pix);
    }





#if ORIENT
    //erode seg a bit to remove boundary effects in orientation extraction:
    


    //get edge map and orientation map from OCSSERVER:
    inBot_or = inPort_or.read(); //blocking
    inBot_e = inPort_e.read(); //blocking

    //make copy of foveal orientation data:
    ippiCopy_8u_C1R((Ipp8u*)&inBot_or->get(0).asBlob()[ocs_psb*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		    ocs_psb,orient,psb_8u,zdf_size);
    //make copy of foveal edge data:
    ippiCopy_8u_C1R((Ipp8u*)&inBot_e->get(0).asBlob()[ocs_psb*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		    ocs_psb,edge,psb_8u,zdf_size);

    //mask orient data for histogram:
    for (int y=0;y<zdf_height;y++){
      for (int x=0;x<zdf_width;x++){
	//mask non-edges and background to 255:
	if (edge[y*psb_8u + x] < THRESH || tex_in[y*zdf_psb + x] == 0 ){ //if not edge within seg.
	  orient[y*psb_8u + x] = 255; //white
	}	  
      }
    }

    //get histogram:
    ippiHistogramEven_8u_C1R(orient,psb_8u,zdf_size,histo,levels,NUM_BINS+1,1,180);
    //for (int n=0;n<NUM_BINS;n++){
    //  printf("%d [%d-%d]:%d\n",n*180/NUM_BINS,levels[n],levels[n+1],histo[n]);
    //}

    //get 'average' orientation of egde features within the segmentation, 
    //taking into account 0-180 wraparound using the ringCoG method!!!!!
    X=0.0,Y=0.0;
    for (int n=0;n<NUM_BINS;n++){
      X += (1.0+histo[n])*sin((IPP_PI/180.0)*n*360/NUM_BINS); //r.sin(theta)
      Y += (1.0+histo[n])*cos((IPP_PI/180.0)*n*360/NUM_BINS); //r.cos(theta)
    }
    av = atan2(X,Y)*(180.0/IPP_PI)/2.0;
    //printf("X:%f, Y:%f - Angle:%f\n",X,Y,av);
    printf("Angle:%f\n",av);

    //rotation upwards required to align with av:
    up_pix = (int) av*polar_size.height/360.0;
#endif





    //IMPOSE ORIENT INVARIANCE: 
    //roll polar image up:
    //handle negative upwards shifts:
    if (up_pix<0){
      //rolling up by -n is equiv to rolling up by (height-abs(n)):
      up_pix = polar_size.height-abs(up_pix);
    }
    rollUp_8u(up_pix,pol_seg,pol_psb_8u,polar_size, pol_seg_rolled,pol_psb_8u);

#if TRANS
    //No! transition slowly! roll TOWARDS up_pix to prevent jumps if angle shifts!:
    //handle case that no shift required:
    if (up_pix == 0){
      //no rolling required, just copy:
      ippiCopy_8u_C1R(pol_seg,pol_psb_8u,pol_seg_rolled,pol_psb_8u,polar_size);
      rollup = 0;
    }

    else{   //need to roll towards up_pix and stop once equal:
      if (rollup<up_pix){
	  rollup += SHIFT;
	}
      else if (rollup>up_pix){
	rollup -= SHIFT;
      }
      //if rollup==up_pix, don't alter rollup    
	
      //handle negative upwards shifts:
      if (rollup<0){
	//rolling up by -n (down by abs(n)) is equivalent to rolling up by (height-abs(n)):
	roll = polar_size.height-abs(rollup);
      }
      //handle upwards shifts:
      else {
	roll = rollup;
      }
	
      //roll!
      rollUp_8u(roll,pol_seg,pol_psb_8u,polar_size,pol_seg_rolled,pol_psb_8u);
    }
#endif      




    //SCALE INVARIANCE:
    //stretch rightmost pix in polar map right to max_width.
    stretchRight_8u(rightmost_x,max_width,pol_seg_rolled,pol_psb_8u, polar_size,  pol_seg_rolled_stretched, pol_psb_8u);

#if TRANS
    //No! transition slowly! stretch TOWARDS max_width to prevent jumps if segmentation shifts!:
    if (rightmost_x == max_width){
      //convert result back to cartesian space immediately:
      polar2cart_8u(pol_seg_rolled,pol_psb_8u,polar_size, out,psb_8u,zdf_size);
    }
    else{
      //shift 'stretch' towards 'max_width':
      if (stretch<max_width){
	stretch += SHIFT;
      }
      else if (stretch>max_width){
	stretch -= SHIFT;
      }
      //shift 'rightmost' towards 'rightmost_x':
      if (rightmost<rightmost_x){
	rightmost += SHIFT;
      }
      else if (rightmost>rightmost_x){
	rightmost -= SHIFT;
      }
      //stretch:
      stretchRight_8u(rightmost,stretch,pol_seg_rolled,pol_psb_8u, polar_size,  pol_seg_rolled_stretched, pol_psb_8u);
    }
#endif




    //convert result back to cartesian space and place in openCV image container 'inv':
    polar2cart_8u(pol_seg_rolled_stretched,pol_psb_8u,polar_size,  (Ipp8u*) inv->imageData,inv->widthStep,zdf_size);


    //resize inv. image into openCV container "temp" for NN query:
    cvResize(inv, temp, CV_INTER_LINEAR);


    
    //**************************************
    //CLASSIFY:
    //set image as inputs to NN:
    extractPixels(temp);
    //run the NN:
    RealVec outputs(NN_out->numNeurons());
    for (int input = 0; input< (int)NN_in->numNeurons(); input++ )
      NN_in->setInput( input, pixelValNorm[input][0] ); 
    net->step();


    //get best output:
    outputs = NN_out->outputs();	
    bestval = 0.0;    
    oldlabel = label;
    if (outputs[0] > bestval){
      label = "bottle";
      bestval = outputs[0];
    }
    if (outputs[1] > bestval){
      label = "fags";
      bestval = outputs[1];
    }
    if (outputs[2] > bestval ){
      label = "talc";
      bestval = outputs[2];
    }

    //if not very sure (unknown)
    if (bestval<threshold){
      //revert to previous label.
      label = oldlabel;
      //only pass 'unknown' if we've had lots of low-conf classifications.
      unknowns++;
      if (unknowns>=max_unknowns){
	label = "unknown";
      }
    }
    else{
      //classification was strong
      unknowns = 0;
    }

    
    printf("ObjRecServer: Mos:[%d,%d][%d,%d] 3D:(%f,%f,%f):%f %s \n",
	   zdfData->mos_xl,zdfData->mos_yl,zdfData->mos_xr,zdfData->mos_yr,
	   zdfData->x,zdfData->y,zdfData->z,bestval,label);
    //**************************************
    
    
    

    //OUTPUT

    //SEND as portable <ObjRecServerData>:
    ObjRecServerData& objData = outPort_objData.prepare();
    objData.resize(zdf_width,zdf_height);
    ippiCopy_8u_C1R((Ipp8u*)inv->imageData,inv->widthStep,
		    (Ipp8u*)objData.tex.getRawImage(),objData.tex.getRowSize(), zdf_size);
    objData.label = label;        
    objData.radius = radius;      
    objData.confidence = bestval; 
    objData.x = zdfData->x;
    objData.y = zdfData->y;
    objData.z = zdfData->z;
    objData.mos_xl = zdfData->mos_xl;
    objData.mos_yl = zdfData->mos_yl;
    objData.mos_xr = zdfData->mos_xr;
    objData.mos_yr = zdfData->mos_yr;
    //send:
    outPort_objData.write();
    

    //}//if update
    
    
  } //while
  
  //never here! 
}







void iCub::contrib::primateVision::ObjRecServer::loadNet(){
  //load the net
  
  NN_in = (BiasedCluster*)net->getByName("in");
  NN_out = (BiasedCluster*)net->getByName("out");
  numInputs = NN_in->numNeurons();
  numOutputs = NN_out->numNeurons();
  NN_hid = (BiasedCluster*)net->getByName("hid");
  in2hid = (DotLinker*)net->getByName("in2hid");
  hid2out = (DotLinker*)net->getByName("hid2out");
  numHiddens = NN_hid->numNeurons();
  //reverse the order	
  UpdatableVec rev_ord(net->order().size());
  rev_ord.assign_reverse(net->order());
  learnNet = new BackPropagationAlgo (net, rev_ord, learnRate);
  learnNet->enableMomentum();
  learnNet->setMomentum(momentum);
}


void iCub::contrib::primateVision::ObjRecServer::extractPixels(IplImage* img){

  int height,width;
  char path[100] ;
  CvScalar s;
  for (int i=0; i<img->height; i++){
    for (int j=0; j<img->width; j++){	

      //get the (i,j) pixel intensity
      s=cvGet2D(img,i,j); 
      //convert to range 0.0->1.0:
      pixelValNorm[inc][0] = s.val[0]/255;		
      //printf("intensity = %lf  %d\n", pixelValNorm[inc][0], inc);
      inc ++;
    }
  }
  inc = 0;

}


void iCub::contrib::primateVision::ObjRecServer::polar2cart_8u(Ipp8u*pol,int psb_pol,IppiSize pol_size, Ipp8u*cart,int psb_cart,IppiSize cart_size){

  ippiSet_8u_C1R(0,cart,psb_cart,cart_size);

  int r,t;
  double at;
  
  for(int y=0;y<cart_size.height;y++){
    for(int x=0;x<cart_size.width;x++){
 
      //radius = dist of this pixel from cog pixel (cart_size.width/2,cart_size.height/2):
      r = (int) sqrt((x-cart_size.width/2.0)*(x-cart_size.width/2.0) + (y-cart_size.height/2.0)*(y-cart_size.height/2.0));

      //theta = 180 + atan(y/x) * rows/360:      
      at = atan2(x-cart_size.width/2.0,y-cart_size.height/2.0);
      if (at == IPP_PI){
	t = 0;
      }
      else{
	t = (int) ( ( 180.0 + at*(180.0/IPP_PI))  *  pol_size.height/360.0);
      }

      if (r<pol_size.width && r>=0 && t<pol_size.height && t>=0){
	cart[y*psb_cart + x] = pol[t*psb_pol + r];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}

    }
  }
  
}


void iCub::contrib::primateVision::ObjRecServer::cart2polar_8u(Ipp8u*cart,int psb_cart,IppiSize cart_size,  Ipp8u* pol,int psb_pol,IppiSize pol_size, int cx,int cy){
  
  ippiSet_8u_C1R(0,pol,psb_pol,pol_size);

  int x,y; 
  
  for(int t=0;t<pol_size.height;t++){
    for(int r=0;r<pol_size.width;r++){
      
      //x = r sin(t):
      x = (int)(r * sin((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.height/2.0 + cx);
      
      //y = r cos(t):
      y = (int)(r * cos((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.width/2.0  + cy);

      if (x>=0 && x<cart_size.width && y>=0 && y<cart_size.height){
        pol[t*psb_pol + r] = cart[y*psb_cart + x];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
	
    }
  }
      
}


void iCub::contrib::primateVision::ObjRecServer::getMaxRightXY_8u(Ipp8u* in, int psb_in,IppiSize sz,int*X,int*Y){
   
  //find rightmost occupied pixel
  
  //loop over image to find rightmost pixel:
  int max_right = 0;
  for (int y=0;y<sz.height;y++){
    for (int x=max_right;x<sz.width;x++){

      if (in[y*psb_in+x]!=0 && x>max_right){
	max_right = x;
	*X = x;
	*Y = y;
      }

    }
  }
  
}


void iCub::contrib::primateVision::ObjRecServer::rollUp_8u(int n,Ipp8u*in,int psb_in, IppiSize sz,Ipp8u*out,int psb_out){

  //roll image up by n rows:
  
  IppiSize sz_1 = {sz.width,sz.height-n};
  IppiSize sz_2 = {sz.width,n};
  
  //copy {nth to last row} of 'in' to 0th row of 'out', height-n rows:
  ippiCopy_8u_C1R(&in[n*psb_in],psb_in,out,psb_out,sz_1);
  //copy {0th to nth row} of 'in' to {(h-n)th} row of 'out', n rows:
  ippiCopy_8u_C1R(in,psb_in,&out[(sz.height-n)*psb_out],psb_out,sz_2);

}


void iCub::contrib::primateVision::ObjRecServer::stretchRight_8u(int max_right, int max_width, Ipp8u* in, int psb_in,IppiSize sz_o, Ipp8u* out, int psb_out){

  //stretch image right to max_width.

  ippiSet_8u_C1R(0,out,psb_out,sz_o);
  
  IppiSize sz_1 = {max_right,sz_o.height};
  IppiRect r_1 = {0,0,max_right,sz_o.height};
  IppiSize sz_2 = {max_width,sz_o.height};
  double xf = ((double)max_width) / ((double)max_right);

  //stretch image right:
  ippiResize_8u_C1R(in,sz_1,psb_in,r_1,  out,psb_out,sz_2,  xf,1.0,IPPI_INTER_LINEAR);

}










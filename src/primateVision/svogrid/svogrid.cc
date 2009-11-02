/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <q3vbox.h>
#include <qimage.h>
#include <ipp.h>

//my includes:
#include "svogrid.h"
#include <recio.h>
#include <depthflowsio.h>
#include <flowsio.h>



#define PI IPP_PI
#define MAX_P 0.99
#define MIN_P 0.01

using namespace iCub::contrib::primateVision;


SVOGRID::SVOGRID(QApplication*a_,string*cfg_)
{

  a=a_;
  cfg = cfg_;
  world_registered=false;

  start();

}


SVOGRID::~SVOGRID()
{

}


void SVOGRID::run()
{

  //CONFIGS:
  Property prop;
  prop.fromConfigFile(cfg->c_str());

  double min_p    = MIN_P;
  double max_p    = MAX_P;
  double no_io    = prop.findGroup("SVOGRID").find("NO_IO").asDouble();
  double io_no    = prop.findGroup("SVOGRID").find("IO_NO").asDouble();
  double no_no    = prop.findGroup("SVOGRID").find("NO_NO").asDouble();
  double io_io    = prop.findGroup("SVOGRID").find("IO_IO").asDouble();
  min_z           = prop.findGroup("SVOGRID").find("MIN_Z").asDouble();
  max_z           = prop.findGroup("SVOGRID").find("MAX_Z").asDouble();
  pixels_per_box  = prop.findGroup("SVOGRID").find("PIXELS_PER_BOX").asInt();
  project_texture = prop.findGroup("SVOGRID").find("PROJECT_TEXTURE").asInt();
  rear_wall       = prop.findGroup("SVOGRID").find("REAR_WALL").asInt(); 
  double decay    = prop.findGroup("SVOGRID").find("DECAY").asDouble();
  

  //REC SERVER PARAMS:
  Port inPort_s;
  inPort_s.open("/svogrid/input/rec_serv_params");     // Give it a name on the network.
  Network::connect("/svogrid/input/rec_serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/svogrid/input/rec_serv_params");
  BinPortable<RecServerParams> server_response_r; 
  Bottle empty;
  inPort_s.write(empty,server_response_r);
  RecServerParams rsp = server_response_r.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;

  //DEPTHFLOW SERVER PARAMS:
  Port inPort_dfs;
  inPort_dfs.open("/svogrid/input/df_serv_params");     // Give it a name on the network.
  Network::connect("/svogrid/input/df_serv_params", "/depthflowserver/output/serv_params");
  Network::connect("/depthflowserver/output/serv_params", "/svogrid/input/df_serv_params");
  BinPortable<DepthflowServerParams> server_response_df; 
  inPort_dfs.write(empty,server_response_df);
  DepthflowServerParams dfsp = server_response_df.content();
  std::cout << "DepthflowServer Probe Response: " << dfsp.toString() << std::endl;

  //FLOW SERVER PARAMS:
  Port inPort_fs;
  inPort_fs.open("/svogrid/input/f_serv_params");     // Give it a name on the network.
  Network::connect("/svogrid/input/f_serv_params", "/flowserver_l/output/serv_params");
  Network::connect("/flowserver_l/output/serv_params", "/svogrid/input/f_serv_params");
  BinPortable<FlowServerParams> server_response_f; 
  inPort_fs.write(empty,server_response_f);
  FlowServerParams fsp = server_response_f.content();
  std::cout << "FlowServer Probe Response: " << fsp.toString() << std::endl;


  mos_width  = rsp.mos_width;
  mos_height = rsp.mos_height;
  focus      = rsp.focus;
  width      = dfsp.width;
  height     = dfsp.height;
  psb        = dfsp.psb;
  psb32f     = dfsp.psb32f;
  range      = dfsp.range;
  offset     = dfsp.offset;
  baseline   = dfsp.baseline;
  flow_scale = fsp.subflow;
  numx_frame = (int) (width/pixels_per_box);
  numy_frame = (int) (height/pixels_per_box);
  numx = (int)(numx_frame*((double)mos_width/(double)width));
  numy = (int)(numy_frame*((double)mos_width/(double)width));
  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;
  IppiSize gridsize_xy;
  gridsize_xy.width  = numx;
  gridsize_xy.height = numy;

 
  //IM PORTS:
  //Y:
  BufferedPort<Bottle> inPort_ly;      // Create a ports
  inPort_ly.open("/svogrid/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ye" , "/svogrid/input/rec_ly");
  Bottle *inBot_ly;
  //U:
  BufferedPort<Bottle> inPort_lu;      // Create a ports
  inPort_lu.open("/svogrid/input/rec_lu");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ue" , "/svogrid/input/rec_lu");
  Bottle *inBot_lu;
  //V:
  BufferedPort<Bottle> inPort_lv;      // Create a ports
  inPort_lv.open("/svogrid/input/rec_lv");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ve" , "/svogrid/input/rec_lv");
  Bottle *inBot_lv;

  //DEPTH:
  BufferedPort<Bottle> inPort_depth;      // Create a ports
  inPort_depth.open("/svogrid/input/df_depth");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/depth" , "/svogrid/input/df_depth");
  Bottle *inBot_depth;
  Ipp32f* depth;
  DepthflowResultParams*df_res_params;

  //MFLOW PORTS:
  //x:
  BufferedPort<Bottle> inPort_mflow_x;      // Create a ports
  inPort_mflow_x.open("/svogrid/input/f_mflow_x");     // Give it a name on the network.
  Network::connect("/flowserver_l/output/x" , "/svogrid/input/f_mflow_x");
  Bottle *inBot_mflow_x;
  Ipp8u* flowx;
  //y:
  BufferedPort<Bottle> inPort_mflow_y;      // Create a ports
  inPort_mflow_y.open("/svogrid/input/f_mflow_y");     // Give it a name on the network.
  Network::connect("/flowserver_l/output/y" , "/svogrid/input/f_mflow_y");
  Bottle *inBot_mflow_y;
  Ipp8u* flowy;
  //z (depthflow):
  BufferedPort<Bottle> inPort_depthflow;      // Create a ports
  inPort_depthflow.open("/svogrid/input/df_depthflow");     // Give it a name on the network.
  Network::connect("/depthflowserver/output/depthflow" , "/svogrid/input/df_depthflow");
  Bottle *inBot_depthflow;
  Ipp32f* flowz;

 

  //get first hd,px,py params:
  inBot_depth = inPort_depth.read();
  if (inBot_depth!=NULL){
    depth = (Ipp32f*) inBot_depth->get(0).asBlob();
    df_res_params = (DepthflowResultParams*) inBot_depth->get(1).asBlob();
  }
  else{printf("Failed to read depthflow params\n");}
  px = df_res_params->px; //centre of l-r overlap x
  py = df_res_params->py; //centre of l-r overlap y
  hd = df_res_params->hd;
  mind = df_res_params->mind;
  maxd = df_res_params->maxd;


  box_l_min = fabs( depth23D(0,0,min_z).x - depth23D(pixels_per_box,0,min_z).x );
  box_l_max = fabs( depth23D(0,0,max_z).x - depth23D(pixels_per_box,0,max_z).x );

  //***calc num_z:***
  numz = 0;
  //at min_z, box edge size is box_l_min.
  //at max_z, box edge size is box_l_max
  //therefore at z, have box edge size b = ((bmax-bmin)/(zmax-zmin))(z-zmin) + bmin
  double box_l = box_l_min;
  double tmp_z = min_z;
  while (tmp_z <= max_z){
    numz++;
    //front of next cube:
    tmp_z+=box_l;
    //length of next cube at tmp_z:
    box_l = ((box_l_max-box_l_min)/(max_z-min_z))*(tmp_z-min_z) + box_l_min;
  }
  max_z = tmp_z;
  //*****************

  double io = 1.0/((double)numz); //% of OG occupied (io) = x*y / x*y*z = 1/z
  double no = 1.0 - io;
  io_ll = log10(io/no);
  occ   = log10(io_io/io_no);
  emp   = log10(no_io/no_no);
  max_l = log10(max_p/(1.0-max_p));
  min_l = log10(min_p/(1.0-min_p));



  OG = make_og();

  //ray max record:
  ray_max_cell = ippiMalloc_8u_C1(numx,numy,&psb_cell);

  //flow binning stuff:
  int **binnedflowx = ( int**) malloc(sizeof(int*)*numx_frame);
  int **binnedflowy = ( int**) malloc(sizeof(int*)*numx_frame);
  double **binnedflowz = ( double**) malloc(sizeof(double*)*numx_frame);
  int **numbinsx = ( int**) malloc(sizeof(int*)*numx_frame);
  int **numbinsy = ( int**) malloc(sizeof(int*)*numx_frame);
  int **numbinsz = ( int**) malloc(sizeof(int*)*numx_frame);
  for (int x=0; x<numx_frame; x++){
    binnedflowx[x]= (int*)  malloc(sizeof(int)*numy_frame);
    binnedflowy[x]= (int*)  malloc(sizeof(int)*numy_frame);
    binnedflowz[x]= (double*) malloc(sizeof(double)*numy_frame);
    numbinsx[x]= (int*)  malloc(sizeof(int)*numy_frame);
    numbinsy[x]= (int*)  malloc(sizeof(int)*numy_frame);
    numbinsz[x]= (int*)  malloc(sizeof(int)*numy_frame);
    for (int y=0; y<numy_frame; y++){
      binnedflowx[x][y]=0;
      binnedflowy[x][y]=0;
      binnedflowz[x][y]=0.0;
      numbinsx[x][y]=0;
      numbinsy[x][y]=0;
      numbinsz[x][y]=0;
    }
  }
  
   

  point3D po;
  cell co,tc;
  int binx,biny;
  IppiPoint p;
  double zbase;





  tex_r = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_g = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_b = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_a = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_y = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_u = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_v = new QImage(pixels_per_box,pixels_per_box,8,256);
  tex_rgba = new QImage(pixels_per_box,pixels_per_box,32,256);
  texsize.width=pixels_per_box;
  texsize.height=pixels_per_box;

  rec_im_ly = ippiMalloc_8u_C1(width,height,&psb);
  rec_im_lu = ippiMalloc_8u_C1(width,height,&psb);
  rec_im_lv = ippiMalloc_8u_C1(width,height,&psb);
  depth     = ippiMalloc_32f_C1(width,height,&psb32f);






  //main event loop:
  while(1){
    
    
    inBot_ly = inPort_ly.read(false);
    inBot_lu = inPort_lu.read(false);
    inBot_lv = inPort_lv.read(false);
    inBot_mflow_x = inPort_mflow_x.read(false);
    inBot_mflow_y = inPort_mflow_y.read(false);
    inBot_depthflow = inPort_depthflow.read(false);
    inBot_depth = inPort_depth.read();


  
        
    //Got depth??
    if (inBot_depth!=NULL){

      depth = (Ipp32f*) inBot_depth->get(0).asBlob();
      df_res_params = (DepthflowResultParams*) inBot_depth->get(1).asBlob();

      px = df_res_params->px; //centre of l-r overlap x
      py = df_res_params->py; //centre of l-r overlap y
      hd = df_res_params->hd;
      mind = df_res_params->mind;
      maxd = df_res_params->maxd;
      
      //calculate near and far horopter planes:
      //hnear:
      pn.bl = depth23D(0,height-1,mind);
      pn.tl = depth23D(0,0,mind);
      pn.tr = depth23D(width-1,0,mind);
      pn.br = depth23D(width-1,height-1,mind);
      
      //hfar:
      pf.bl = depth23D(0,height-1,maxd);
      pf.tl = depth23D(0,0,maxd);
      pf.tr = depth23D(width-1,0,maxd);
      pf.br = depth23D(width-1,height-1,maxd);
      
  
      //calculate measured volume bounds:
      tc = point2Cell(pn.bl);
      zmin=tc.z;
      xmax=tc.x + 1;
      ymin=tc.y;
      tc = point2Cell(pf.tr);
      zmax=tc.z;
      xmin=tc.x + 1;
      ymax=tc.y;
      
      if (zmax>(numz-1)){zmax = numz-1;}


 
  
      //if there is a representable measured volume:
      if (zmin<zmax){ 

	//INTEGRATE NEW DEPTH DATA INTO OG:  
	for (int y=0;y<height;y++){
	  for (int x=0;x<width;x++){
	    //if we got an 'occupied' measurement
	    if(depth[y*psb32f/4+x]!=0.0){
	      po = depth23D(x,y,depth[y*psb32f/4+x]);
	      co = point2Cell(po);
	      ray_occupied(co);
	    }
	    //if we measured empty along the whole ray
	    else{
	      po = depth23D(x,y,1.0);
	      co = point2Cell(po);	 
	      ray_empty(co);
	    }	    
	  }
	}
	

	//GET RAY MAX:
	//reset ray_max to rear wall position for all rays in OG:
	ippiSet_8u_C1R((numz-1),ray_max_cell,psb_cell,gridsize_xy);
	//Update max cell in each ray in measurable volume: 
	for (int y=ymin;y<=ymax;y++){
	  for (int x=xmin;x<=xmax;x++){
	    ray_max = OG[x][y][zmin].occupancy;
	    for (int z=zmin;z<zmax;z++){
	      //if it's stronger:
	      if (OG[x][y][z].occupancy>ray_max){
		//record it as the occupied cell for that ray:
		ray_max_cell[y*psb_cell+x] = z;
		ray_max = OG[x][y][z].occupancy;
	      }
	    }
	  }
	}

	//decay entire OG towards io_ll;
	for (int z=0;z<numz;z++){
	  for (int y=0;y<numy;y++){
	    for (int x=0;x<numx;x++){
	      OG[x][y][z].occupancy += (1.0-decay)*(io_ll-OG[x][y][z].occupancy);
	    }
	  }
	}
	
      }//if volume
      
      //display:
      if (world_registered){



	if (inBot_ly!=NULL){ippiCopy_8u_C1R((Ipp8u*)inBot_ly->get(0).asBlob(),psb,rec_im_ly,psb,srcsize);}
	if (inBot_lu!=NULL){ippiCopy_8u_C1R((Ipp8u*)inBot_lu->get(0).asBlob(),psb,rec_im_lu,psb,srcsize);}
	if (inBot_lv!=NULL){ippiCopy_8u_C1R((Ipp8u*)inBot_lv->get(0).asBlob(),psb,rec_im_lv,psb,srcsize);}


	a->lock();
	world->callback();
	a->unlock();      
      }
            
    }//got depth and tex
    
    
    

    
    //got flow?
    if (inBot_mflow_x!=NULL && inBot_mflow_y!=NULL && inBot_depthflow!=NULL){
     
      flowx = (Ipp8u*) inBot_mflow_x->get(0).asBlob();  
      flowy = (Ipp8u*) inBot_mflow_y->get(0).asBlob();
      flowz = (Ipp32f*) inBot_depthflow->get(0).asBlob();
      df_res_params = (DepthflowResultParams*) inBot_depthflow->get(1).asBlob();

      
      //3D FLOW:
      //reset bin counters and accumulators:
      for (int y = 0;y<numy_frame;y++){
	for (int x = 0;x<numx_frame;x++){
	  numbinsx[x][y] = 0;
	  numbinsy[x][y] = 0;
	  numbinsz[x][y] = 0;
	  binnedflowx[x][y] = 0;
	  binnedflowy[x][y] = 0;
	  binnedflowz[x][y] = 0.0;
	}
      }
      
      //accumulate x,y flow vectors into bins of size pixels_per_box:
      //offset by 12 pixels!!
      for (int y = 0;y<height;y++){
	for (int x = 0;x<width;x++){
	  binx = (int) floor((double) ((x+12)*flow_scale)/((double) pixels_per_box) );
	  biny = (int) floor((double) ((y+12)*flow_scale)/((double) pixels_per_box) );
	  
	  if (binx>=0 && binx<numx_frame &&
	      biny>=0 && biny<numy_frame){
	    
	    //left:
	    if (flowx[y*psb+x] >0 && 
		flowx[y*psb+x] <8 &&
		flowy[y*psb+x] >0 && 
		flowy[y*psb+x] <8 ){
	      binnedflowx[binx][biny] += flowx[y*psb+x]-4;
	      numbinsx[binx][biny]++;
	      binnedflowy[binx][biny] += flowy[y*psb+x]-4;
	      numbinsy[binx][biny]++;
	    }
	    
	    //right:
	    //if (flowx[y*psb+x] >0 && 
	    //flowx[y*psb+x] <8 &&
	    //flowy[y*psb+x] >0 && 
	    //flowy[y*psb+x] <8 ){
	    //binnedflowx[binx][biny] += flowx[y*psb+x]-4;
	    //numbinsx[binx][biny]++;
	    //binnedflowy[binx][biny] += flowy[y*psb+x]-4;
	    //numbinsy[binx][biny]++;
	    //}
	    
	  }
	}
      }
      
      //accumulate z flow bins:
      //offset by (px/2, py/2)
      for (int y = 0;y<height;y++){
	for (int x = 0;x<width;x++){
	  
	  binx = (int) floor( (double)((x+abs(px)/2)/((double) pixels_per_box)) );
	  biny = (int) floor( (double)((y+abs(py)/2)/((double) pixels_per_box)) );
	  if (binx>=0 && binx<numx_frame &&
	      biny>=0 && biny<numy_frame &&
	      fabs(flowz[y*width+x])>0.01){ 
	    binnedflowz[binx][biny] += flowz[y*width+x];
	    numbinsz[binx][biny]++;
	  }
	}
      }
      
      //work out average flows for each bin:
      for (int y = 0;y<numy_frame;y++){
	for (int x = 0;x<numx_frame;x++){
	  
	  if (numbinsx[x][y]!=0){
	    binnedflowx[x][y] *= -flow_scale;
	    binnedflowx[x][y] /= numbinsx[x][y];
	  }
	  else {binnedflowx[x][y] = 0;}
	  
	  if (numbinsy[x][y]!=0){
	    binnedflowy[x][y] *= -flow_scale;
	    binnedflowy[x][y] /= numbinsy[x][y];
	  }
	  else {binnedflowy[x][y] = 0;}
	  
	  if (numbinsz[x][y]!=0){
	    binnedflowz[x][y] /= ((double) -numbinsz[x][y]);
	  }
	  else {binnedflowz[x][y] = 0.0;}
	  
	}
      }
      
      
      
      
      //now have flow vectors for measured region.
      //(x,y are in pixels, z is in meters)
      //look for occupied cells in ogrid and superimpose velocity:
      for (int z = 0;z<numz;z++){
	for (int y = 0;y<numy;y++){
	  for (int x = 0;x<numx;x++){
	    if (OG[x][y][z].occupancy>io_ll){
	      //get image indices for this occupied cell:
	      co.x = x; co.y=y; co.z=z;
	      cell2im(co,&p);
	      //convert to bin indices:
	      binx = p.x/pixels_per_box;
	      biny = p.y/pixels_per_box;
	      zbase=cell2Point(co).z;
	      if (binx>=0 && binx<numx_frame &&
		  biny>=0 && biny<numy_frame  &&
		  zbase + binnedflowz[binx][biny] < pf.bl.z &&
		  zbase + binnedflowz[binx][biny] > pn.bl.z &&
		  fabs(binnedflowz[binx][biny]) < (pf.bl.z-pn.bl.z)*0.33 ){
		//save to ogrid:
		OG[x][y][z].vel.z = binnedflowz[binx][biny];
		//convert x,y flow to meters:
		po = cell2Point(co);
		OG[x][y][z].vel.x = ((double) binnedflowx[binx][biny]) * po.z/((double)focus);
		OG[x][y][z].vel.y = ((double) binnedflowy[binx][biny]) * po.z/((double)focus);
	      }
	      else {OG[x][y][z].vel.z=0.0; OG[x][y][z].vel.x=0.0; OG[x][y][z].vel.y=0.0;}
	    }
	    else {OG[x][y][z].vel.z=0.0; OG[x][y][z].vel.x=0.0; OG[x][y][z].vel.y=0.0;}
	  }
	}
      }
      
      
    }//got flow
    
    
    
    
    
    //Don't blow out ports!
    if (inBot_mflow_x   ==NULL && 
	inBot_mflow_y   ==NULL &&
	inBot_depthflow ==NULL &&
	inBot_depth     ==NULL &&
	inBot_ly        ==NULL &&
	inBot_lu        ==NULL &&
	inBot_lv        ==NULL ){
      printf("No Input\n");
      usleep(5000);
    }
    

  }//WHILE
  

  //never here  

}







void SVOGRID::ray_occupied(cell c_)
{


  int ray_z = 0;
  int ray_z_max = zmax;
  int ray_x= c_.x;
  int ray_y= c_.y;
  
  //look along the ray passing through this cell
  while (ray_z<ray_z_max){
    
    //decrease cells infront of point according to (measured empty):
    if (ray_z<c_.z){
      if (OG[ray_x][ray_y][ray_z].occupancy + emp >= min_l){
	OG[ray_x][ray_y][ray_z].occupancy += emp;
      }
      else { OG[ray_x][ray_y][ray_z].occupancy = min_l;}
    }
    
    //increase the cell that the point is in according to (measured occupied):
    else if (ray_z==c_.z){
      if (OG[ray_x][ray_y][ray_z].occupancy+occ<=max_l){
	OG[ray_x][ray_y][ray_z].occupancy+=occ;
      }
      else{OG[ray_x][ray_y][ray_z].occupancy = max_l;}
    }
    
    //next cell on ray
    ray_z++;
  }
}


void SVOGRID::ray_empty(cell c_)
{	
  
  int ray_z = 0;
  int ray_z_max = zmax;
  int ray_x= c_.x;
  int ray_y= c_.y;
  
  //look along the ray passing through this cell
  while (ray_z<ray_z_max){
  
    //decrease cells in measured volume according to (measured empty):
    if (OG[ray_x][ray_y][ray_z].occupancy + emp>=min_l){
      OG[ray_x][ray_y][ray_z].occupancy += emp;
    }
    else{OG[ray_x][ray_y][ray_z].occupancy = min_l;}

    //next cell on ray
    ray_z++;
  }
}


void SVOGRID::cell2im(cell c_, IppiPoint* p_){

  point3D p3_ = cell2Point(c_);

  double D = focus*baseline/p3_.z;
  p_->x =(int)(  -px - p3_.x*D/baseline   );
  p_->y =(int)(  -py - p3_.y*D/baseline   );
}

void SVOGRID::get_cell_params(int cell_z_,double *z_front_, double *len_){

  //at min_z box edge size is box_l_min.
  //at max_z box edge size is box_l_max
  //therefore at z, have box edge size b = ((bmax-bmin)/(zmax-zmin))(z-zmin) + bmin

  //front face of cube:
  *z_front_=min_z;
  *len_=box_l_min;
  for (int i=0;i<cell_z_;i++){
    //front of next cube:
    (*z_front_) += (*len_); 
    //length of next cube at z_front:
    (*len_) = ((box_l_max-box_l_min)/(max_z-min_z))*(*z_front_-min_z) + box_l_min;
  }
}




QImage* SVOGRID::get_tex_y(cell c_){

  IppiPoint p_;
  
  //get texture square bounds in yl:
  cell2im(c_,&p_);
  
  //copy texture map from "yl", if within bounds:
  if (p_.y-pixels_per_box/2+1 >=0 && p_.x-pixels_per_box/2+1>=0 &&
      p_.y+pixels_per_box/2 < height && p_.x+pixels_per_box/2 < width){

    
    ippiCopy_8u_C1R(&rec_im_ly[psb*(p_.y-pixels_per_box/2+1) + (p_.x-pixels_per_box/2+1)],
 		    psb,
 		    tex_y->bits(),
 		    tex_y->width(),
 		    texsize);
  }
  
  return tex_y;

}

QImage* SVOGRID::get_tex_u(cell c_){

  IppiPoint p_;

  //get texture square bounds in yl:
  cell2im(c_,&p_);
	  
  //copy texture map from "ul", if within bounds:
  if (p_.y-pixels_per_box/2+1 >=0 && p_.x-pixels_per_box/2+1>=0 &&
      p_.y+pixels_per_box/2 < height && p_.x+pixels_per_box/2 < width){

    
    ippiCopy_8u_C1R(&rec_im_lu[psb*(p_.y-pixels_per_box/2+1) + (p_.x-pixels_per_box/2+1)],
 		    psb,
 		    tex_u->bits(),
 		    tex_u->width(),
 		    texsize);
  }

  return tex_u;

}

QImage* SVOGRID::get_tex_v(cell c_){

  IppiPoint p_;

  //get texture square bounds in yl:
  cell2im(c_,&p_);
	  
  //copy texture map from "vl", if within bounds:
  if (p_.y-pixels_per_box/2+1 >=0 && p_.x-pixels_per_box/2+1>=0 &&
      p_.y+pixels_per_box/2 < height && p_.x+pixels_per_box/2 < width){
    
    ippiCopy_8u_C1R(&rec_im_lv[psb*(p_.y-pixels_per_box/2+1) + (p_.x-pixels_per_box/2+1)],
 		    psb,
 		    tex_v->bits(),
 		    tex_v->width(),
 		    texsize);
  }

  return tex_v;

}

QImage* SVOGRID::get_tex_rgba(cell c_){

  
  //planar yuv src
  QImage* pyuv_q[3];
  pyuv_q[0]= get_tex_y(c_);
  pyuv_q[1]= get_tex_u(c_);
  pyuv_q[2]= get_tex_v(c_);
  
  Ipp8u* pyuv[3];
  pyuv[0]= pyuv_q[0]->bits();
  pyuv[1]= pyuv_q[1]->bits();
  pyuv[2]= pyuv_q[2]->bits();
  
  //planar rgb dst
  Ipp8u* prgb[3];
  prgb[0]= tex_r->bits();
  prgb[1]= tex_g->bits();
  prgb[2]= tex_b->bits();

  //make planar rgb:
  ippiYUVToRGB_8u_P3R(&pyuv[0],tex_rgba->width(),&prgb[0],tex_rgba->width(),texsize);

  //convert to pixel-order rgba:
  const Ipp8u* psrc[4];
  psrc[0]= tex_r->bits();
  psrc[1]= tex_g->bits();
  psrc[2]= tex_b->bits();
  psrc[3]= tex_a->bits();
 
  ippiCopy_8u_P4C4R(psrc,tex_rgba->width(),tex_rgba->bits(),tex_rgba->width()*4,texsize);

  return tex_rgba;
}


og_data*** SVOGRID::make_og()
{
 og_data*** G = (og_data***) malloc(sizeof(og_data**)*numx);

  for (int x=0;x < numx; x++){
    G[x] = (og_data**) malloc(sizeof(og_data*)*numy);
    for (int y=0;y < numy; y++){
      G[x][y] = (og_data*) malloc(sizeof(og_data)*numz);
      for (int z=0;z < numz; z++){
	//Bayesian:
	//G[x][y][z]=io;
	//Log-Likelihood:
	G[x][y][z].occupancy = io_ll;
	G[x][y][z].ray_max = false;
	G[x][y][z].vel.x = 0.0;
	G[x][y][z].vel.y = 0.0;
	G[x][y][z].vel.z = 0.0;
      }
    }
  }
  return G;
}


point3D SVOGRID::depth23D(int x, int y, double depth_){
  
  point3D p;

  p.z = depth_;
  p.x = -(x+px)*depth_/focus;
  p.y = -(y+py)*depth_/focus;

  return p;
}


cell SVOGRID::point2Cell(point3D p)
{
  cell c;

  //calc grid z:
  c.z = 0;
  double tmp_z = min_z;
  double box_len = box_l_min;
  while (tmp_z<p.z){
    tmp_z+=box_len;
    box_len = ((box_l_max-box_l_min)/(max_z-min_z))*(tmp_z-min_z) + box_l_min;
    c.z++;
  }

  c.x = (int)floor((p.x+box_len*numx/2)/box_len); 
  c.y = (int)floor((p.y+box_len*numy/2)/box_len); 


  //sanity checks:
  if (c.z>=numz){
    c.z=numz -1;
  }
  if (c.z<0){
    c.z=0;
  }
  if (c.x>=numx){
    c.x=numx -1;
  }
  if (c.x<0){
    c.x=0;
  }
  if (c.y>=numy){
    c.y=numy -1;
  }
  if (c.y<0){
    c.y=0;
  }


  return c;
}

point3D SVOGRID::cell2Point(cell c)
{
  point3D p;

  //front of cube:
  double z_front=min_z;
  double len=box_l_min;
  for (int i=0;i<c.z;i++){
    z_front += len; 
    len = ((box_l_max-box_l_min)/(max_z-min_z))*(z_front-min_z) + box_l_min;
  }

  p.z = z_front + len/2;
  p.x = c.x*len - numx/2*len + len/2;
  p.y = c.y*len - numy/2*len + len/2;

  return p;
}



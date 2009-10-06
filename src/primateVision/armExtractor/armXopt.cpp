#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "energy.h"
#include "armXopt.h"


//use NDT or RANK comparision?
#define RANK0_NDT1 1 //0 (NDT FASTER)

//NDT:
#define NDTX     1
#define NDTY     1
#define NDTSIZE  4 //4 or 8: 4
#define NDTEQ    0 //0

//RANK:
#define RANKY    1 //1 or 2: 1
#define RANKX    1  //1 or 2: 1
#define RANKSIZE 9  //9 or 25: 9
//RANKSISE = (RANKX*2+1)*(RANKY*2+1) //e.g., 15 for (2,1) :)

#define DRANGE    16
#define MIN_TEX   10

#define NZD   1
#define N_NZD 0

#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) < (b) ? (a) : (b))


iCub::contrib::primateVision::ArmXOpt::ArmXOpt(IppiSize im_size_, int psb_in_, Parameters *_params)
{

  params = _params;   
  nmaps=2;
  im_size.width = im_size_.width;
  im_size.height = im_size_.height;
  psb_in = psb_in_;
  
  im_sz.x = im_size.width;
  im_sz.y = im_size.height;

  out = ippiMalloc_8u_C1(im_size.width,im_size.height,&psb); 

  ptr_im = (void**) malloc(im_size.width*im_size.height*sizeof(void*));
  len_nv = im_size.width;

  clear();

}


iCub::contrib::primateVision::ArmXOpt::~ArmXOpt()
{

}


void iCub::contrib::primateVision::ArmXOpt::clear()
{

  ippiSet_8u_C1R(0,out,psb,im_size);

}


void iCub::contrib::primateVision::ArmXOpt::generate_permutation(int *buf, int n)
{
  int i, j;
  
  for (i=0; i<n; i++) buf[i] = i;
  for (i=0; i<n-1; i++)
    {
      j = i + (int) (((double)rand()/RAND_MAX)*(n - i));
      int tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp;
    }
}


void iCub::contrib::primateVision::ArmXOpt::proc(Ipp8u* im_l_,Ipp8u* im_r_,Ipp8u* im_d_, Ipp8u*dog_l_,Ipp8u*dog_r_,Ipp8u*p_sim_,Ipp8u*p_feedback_)
{

  im_l=im_l_;
  im_r=im_r_;
  im_d=im_d_;
  dog_l = dog_l_;
  dog_r = dog_r_;
  p_sim = p_sim_;
  p_feedback = p_feedback_;

  int a;
  int buf_num;
  int label;
  int E_old;
  unsigned int seed = time(NULL);
  srand(1);
  int *permutation = new int[nmaps];
  bool *buf = new bool[nmaps];


  //initial energy:
  compute_energy();
  

  //optimise:
  for (int i=0; i<nmaps; i++) buf[i] = false;
  buf_num = nmaps;
  for (int iter=0; iter<params->iter_max && buf_num>0; iter++)
    {
      if (iter==0 || params->randomize_every_iteration)
	generate_permutation(permutation, nmaps);

      for (int index=0; index<nmaps; index++)
	{
	  label = permutation[index];
	  if (buf[label]) continue;
	  
	  a = label;
	  E_old = E;
	  expand(a);
	  
	  if (E_old == E)
	    {
	      if (!buf[label]) { buf[label] = true; buf_num--; }
	    }
	  else
	    {
	      for (int i=0; i<nmaps; i++) buf[i] = false;
	      buf[label] = true;
	      buf_num = nmaps - 1;
	    }
	}
    }
  
  
  delete permutation;
  delete buf;
}


int iCub::contrib::primateVision::ArmXOpt::likelihood(Coord c, int label)
{

  //The probability of the images being the way they are,
  //given the hypothesized configuration.
  //return low penalty if likely.


  int penalty = 0;
 

  //GET DISPARITY:

  //int d_L = im_d[c.x + psb_in*c.y]-DRANGE;
  //(dist from zd)

  //RANK/NDT TRANSFORM:
  int ndt1[NDTSIZE];
  int ndt2[NDTSIZE];
  int rank1[RANKSIZE];
  int rank2[RANKSIZE];
  double cmp_res;
  int koffsetx;
  int koffsety;
  if (RANK0_NDT1==0){
    koffsetx=RANKX;
    koffsety=RANKY;
  }
  else{
    koffsetx=NDTX;
    koffsety=NDTY;
  }

  Coord cr;  
  double p_tmp;
  double p_d_c = 0.0;
  int d_c;
  //check all possible disparities of site c:
  //if within frame
  if (c.x-koffsetx>=0 && c.x+koffsetx<im_size.width  &&
      c.y-koffsety>=0 && c.y+koffsety<im_size.height ){
    //if textured
    if (dog_l[c.x + c.y*psb_in] >= params->bland_dog_thresh){
      get_ndt(c,im_l,psb_in,ndt1);
      //get_rank(c,im_l,psb_in,rank1);
      for (int i=0;i<=2*DRANGE;i++){
	cr.x = c.x+i-DRANGE;
	cr.y = c.y;
	//if within frame  
	if (cr.x-koffsetx>=0 && cr.x+koffsetx<im_size.width  &&
	    cr.y-koffsety>=0 && cr.y+koffsety<im_size.height ){
	  //if textured
	  if (dog_r[cr.x + cr.y*psb_in] >= params->bland_dog_thresh){
	    get_ndt(cr,im_r,psb_in,ndt2);
	    //get_rank(c,im_r,psb_in,rank2);
	    p_tmp = cmp_ndt(ndt1,ndt2);
	    //p_tmp = cmp_rank(rank1,rank2);
	    if (p_tmp>p_d_c){
	      p_d_c = p_tmp;
	      d_c = i-DRANGE;
	    }
	  }	
	}
      }
    }
  }
  

  //Radius:
  double rmax = sqrt((im_size.width/2.0)*(im_size.width/2.0) 
		     +(im_size.height/2.0)*(im_size.height/2.0));
  double r = sqrt((c.x-im_size.width/2.0)*(c.x-im_size.width/2.0) 
		  +(c.y-im_size.height/2.0)*(c.y-im_size.height/2.0));





  //penalties associated with labeling site 'near zero disparity' (nzd): 
  if (label==NZD){ //WHITE!

    //DISPARITY NOT FOUND but site textured. It's
    //probably beyond disparity measurement range.
    //It's not likely to be at nzd, so apply penalty:
    if (p_d_c<=0.5 && dog_l[c.x + c.y*psb_in] >= params->bland_dog_thresh){
      penalty += params->nzd_dmap_bg_data_penalty;
    }

    //DISPARITY penalty:
    //Else the further away from zd, the bigger the penalty.
    else if(p_d_c>0.5) {
      penalty += (int)round( ((double)params->nzd_dmap_data_penalty) * ((double)abs(d_c))/DRANGE );
    }

    //RADIAL penalty:
    //The furthar away from origin, bigger the penalty up to rad_penalty:
    penalty += (int)round(((double)params->rad_data_penalty) * r/rmax);

    //SIMULATOR penalty:
    //small if white:
    penalty += (int)round(((double)params->sim_data_penalty) * (1.0-p_sim[c.x + psb_in*c.y]/255.0));
        
    //NZD FEEDBACK penalty:
    //if white, small penalty.
    penalty += (int)round(((double)params->nzd_fb_data_penalty) * (1.0-p_feedback[c.x + psb_in*c.y]/255.0));

  }

  
  //else penalties associated with labelling it !nzd:  
  else{ //BLACK!
    
    //ZD penalty:
    //The more likely it's at exactly zd, the more zd penalty:
    if (d_c==0 && p_d_c>0.5){
      penalty += (int)round(((double)params->nnzd_zd_data_penalty) * (p_d_c-0.5)/(1.0-0.5));
    }
    
    //DISPARITY penalty:
    //But, if there is a disparity measurement,
    //it is more likely to be foreground, so apply fg penalty.
    //Also, the closer to zd, the less likely to be bg so apply dmap penalty,
    //smallest penalty at DRANGE, max penalty at 0. 
    else if (p_d_c>0.5){
      penalty += params->nnzd_dmap_fg_data_penalty;
      penalty += (int)round(((double)params->nnzd_dmap_data_penalty) * (1.0-((double)abs(d_c))/DRANGE));
    }

    //RADIAL penalty:
    //The closer to the origin, the bigger the penalty up to radial_penalty:
    penalty += (int)round(((double)params->rad_data_penalty) * (1.0-r/rmax));

    //SIMULATOR penalty 
    //small if black:
    penalty += (int)round(((double)params->sim_data_penalty) * (p_sim[c.x + psb_in*c.y]/255.0));
    
    //!NZD FEEDBACK penalty:
    //if white, big penalty
    penalty += (int)round(((double)params->nnzd_fb_data_penalty) * (p_feedback[c.x + psb_in*c.y]/255.0));

  }

  
  return penalty; 
}




int iCub::contrib::primateVision::ArmXOpt::prior_smoothness(Coord p, Coord np, int label, int nlabel)
{

  int penalty = 0;
 

  //if both labels hypothesized same, no smoothness penalty!! 


  //BUT if hypothesized different, 
  if (label != nlabel){

    //always a tiny penalty for labelling neighbours differently:
    penalty += params->neighbour_smoothness_penalty;

    int d_I = im_l[p.x + psb_in*p.y] - im_l[np.x + psb_in*np.y];
    double sigma = 2.0*((double)params->int_smoothness_3sigmaon2)/3.0;
    double p_int_smooth = exp(-(d_I*d_I)/(2.0*sigma*sigma));


    //primary smoothness penalty if they have similar intensity (no intensity shift):
    if (p_int_smooth > 0.5){
      //primary int smoothness penalty:
      penalty += params->int_smoothness_penalty;
    }


    //But if it's not smooth in intensity, it may still be smooth in disparity,
    //so we can reduce the set of segmentation edges.
    else{
      
      //penalty if they are both at similar disparity (no disparity shift):
      //we already know they are not smooth in intensity, textured!
      double p_tmp;
      int ndt1[NDTSIZE];
      int ndt2[NDTSIZE];
      int rank1[RANKSIZE];
      int rank2[RANKSIZE];
      int koffsetx;
      int koffsety;
      if (RANK0_NDT1==0){
	koffsetx=RANKX;
	koffsety=RANKY;
      }
      else{
	koffsetx=NDTX;
	koffsety=NDTY;
      }
      Coord pr;
      
      double p_d_p = 0.0;
      int d_p;
      //check all possible disparities of pixel p:
      //if within frame:
      if (p.x-koffsetx>=0 && p.x+koffsetx<im_size.width  &&
	  p.y-koffsety>=0 && p.y+koffsety<im_size.height ){
	//if textured:
	if (dog_l[p.x + p.y*psb_in] >= params->bland_dog_thresh){
	  get_ndt(p,im_l,psb_in,ndt1);
	  //get_rank(p,im_l,psb_in,rank1);
	  for (int i=0;i<=2*DRANGE;i++){
	    pr.x = p.x+i-DRANGE;
	    pr.y = p.y;
	    //if within frame:
	    if (pr.x-koffsetx>=0 && pr.x+koffsetx<im_size.width  &&
		pr.y-koffsety>=0 && pr.y+koffsety<im_size.height ){
	      //if textured:
	      if (dog_r[pr.x + pr.y*psb_in] >= params->bland_dog_thresh){
		get_ndt(pr,im_r,psb_in,ndt2);
		//get_rank(pr,im_r,psb_in,rank2);
		p_tmp = cmp_ndt(ndt1,ndt2);
		//p_tmp = cmp_rank(rank1,rank2);
		if (p_tmp>p_d_p){
		  p_d_p = p_tmp; 
		  d_p = i-DRANGE;
		}
	      }
	    }
	  }
	}
      }
      
      double p_d_np = 0.0;
      int d_np;
      //check all possible disparities of pixel np:
      //if within frame:
      if (np.x-koffsetx>=0 && np.x+koffsetx<im_size.width  &&
	  np.y-koffsety>=0 && np.y+koffsety<im_size.height ){
	//if textured:
	if (dog_l[np.x + np.y*psb_in] >= params->bland_dog_thresh){
	  get_ndt(np,im_l,psb_in,ndt1);
	  //get_rank(np,im_l,psb_in,rank1);
	  for (int i=0;i<=2*DRANGE;i++){
	    pr.x = np.x+i-DRANGE;
	    pr.y = np.y;
	    //if within frame:
	    if (pr.x-koffsetx>=0 && pr.x+koffsetx<im_size.width  &&
		pr.y-koffsety>=0 && pr.y+koffsety<im_size.height ){
	      //if textured:
	      if (dog_r[pr.x + pr.y*psb_in] >= params->bland_dog_thresh){
		get_ndt(pr,im_r,psb_in,ndt2);
		//get_rank(pr,im_r,psb_in,rank2);
		p_tmp = cmp_ndt(ndt1,ndt2);
		//p_tmp = cmp_rank(rank1,rank2);
		if (p_tmp>p_d_np){
		  p_d_np = p_tmp; 
		  d_np = i-DRANGE;
		}
	      }
	    }
	  }
	}
      }


      //if both out of the disparity range (texture but no disp) 
      //both background, should be labeled same:
      if ((p_d_p<=0.5  && dog_l[p.x + p.y*psb_in]   >= params->bland_dog_thresh) &&
	  (p_d_np<=0.5 && dog_l[np.x + np.y*psb_in] >= params->bland_dog_thresh) ){
      	penalty += params->dmap_bg_smoothness_penalty;
      }
      
      //else if both disparities confident and within 1:
      else if(abs(d_p - d_np)<=1 && p_d_np>0.5 && p_d_p>0.5){
	//it's smooth, so apply additional disp smoothness penalty!
	penalty += params->disp_smoothness_penalty;
      }
      
            
    }
  
  }
  
  return penalty;
  
}




/* computes current energy */
int iCub::contrib::primateVision::ArmXOpt::compute_energy()
{

  int d,dq;
  Coord p,q;
  
  E = 0;
  for (p.y=1; p.y<im_size.height-1; p.y++)
    for (p.x=1; p.x<im_size.width-1; p.x++)
      {
	d = out[p.y*psb + p.x];
	//non-neighbourhood terms:
	E += likelihood(p, d);
	
	for (int k=0; k<NEIGHBOR_NUM; k++)
	  {
	    q = p + NEIGHBORS[k];
	    
	    if (q>=Coord(1,1) && q<im_sz-Coord(1,1))
	      {
		dq = out[q.y*psb + q.x];
		//neighbourhood terms:
		E += prior_smoothness(p, q, d, dq);
	      }
	  }
      }

  return E;
  
}






#define VAR_ACTIVE ((Energy::Var)0)

#define ALPHA_SINK
/*
  if ALPHA_SINK is defined then interpretation of a cut is as follows:
  SOURCE means initial label
  SINK   means new label \alpha
  
  if ALPHA_SINK is not defined then SOURCE and SINK are swapped
*/
#ifdef ALPHA_SINK
#define ADD_TERM1(var, E0, E1) add_term1(var, E0, E1)
#define ADD_TERM2(var1, var2, E00, E01, E10, E11) add_term2(var1, var2, E00, E01, E10, E11)
#define VALUE0 0
#define VALUE1 1
#else
#define ADD_TERM1(var, E0, E1) add_term1(var, E1, E0)
#define ADD_TERM2(var1, var2, E00, E01, E10, E11) add_term2(var1, var2, E11, E10, E01, E00)
#define VALUE0 1
#define VALUE1 0
#endif

void error_function(char *msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(1);
}

void iCub::contrib::primateVision::ArmXOpt::expand(int a)
{
  Coord p,q;
  int d,dq;
  Energy::Var var, qvar;
  int E_old, E00, E0a, Ea0;
  int k;
  
  
  Energy *e = new Energy(error_function);
  
  
  /* non-neighbourhood terms */
  for (p.y=1; p.y<im_size.height-1; p.y++)
    for (p.x=1; p.x<im_size.width-1; p.x++)
      {
	d = out[p.y*psb + p.x];
	if (a == d)
	  {
	    ptr_im[p.y*len_nv + p.x] = VAR_ACTIVE;
	    e->add_constant(likelihood(p, d));
	  }
	else
	  {
	    ptr_im[p.y*len_nv + p.x] = var = e->add_variable();
	    e->ADD_TERM1(var, likelihood(p, d), likelihood(p, a));
	  }
      }
  

  /* neighbourhood terms */
  for (p.y=1; p.y<im_size.height-1; p.y++)
    for (p.x=1; p.x<im_size.width-1; p.x++)
      {	
	d = out[p.y*psb + p.x];
	var = (Energy::Var) ptr_im[p.y*len_nv + p.x];
	for (k=0; k<NEIGHBOR_NUM; k++)
	  {
	    q = p + NEIGHBORS[k];
	    if ( ! ( q>=Coord(1,1) && q<im_sz-Coord(1,1) ) ) continue;
	    qvar = (Energy::Var) ptr_im[q.y*len_nv + q.x];
	    dq = out[q.y*psb + q.x];
	    if (var != VAR_ACTIVE && qvar != VAR_ACTIVE)
	      E00 = prior_smoothness(p, q, d, dq);
	    if (var != VAR_ACTIVE)
	      E0a = prior_smoothness(p, q, d, a);
	    if (qvar != VAR_ACTIVE)
	      Ea0 = prior_smoothness(p, q, a, dq);
	    if (var != VAR_ACTIVE)
	      {
		if (qvar != VAR_ACTIVE) e->ADD_TERM2(var, qvar, E00, E0a, Ea0, 0);
		else                    e->ADD_TERM1(var, E0a, 0);
	      }
	    else
	      {
		if (qvar != VAR_ACTIVE) e->ADD_TERM1(qvar, Ea0, 0);
	      }
	  }
      }
  
  E_old = E;
  E = e->minimize();
  
  if (E < E_old)
    {
      for (p.y=1; p.y<im_size.height-1; p.y++)
	for (p.x=1; p.x<im_size.width-1; p.x++)
	  {
	    var = (Energy::Var) ptr_im[p.y*len_nv + p.x];
	    if (var!=VAR_ACTIVE && e->get_var(var)==VALUE1)
	      {
		out[p.y*psb + p.x] = a;
	      }
	  }
    }
  
  delete e;
  
}


void iCub::contrib::primateVision::ArmXOpt::get_ndt(Coord c,Ipp8u * im, int w, int*list)
{

  Coord n;

  int ndt_ind = 0;
  n = c+Coord(1,0);     
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;  

  ndt_ind++;
  n = c+Coord(0,1);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;

  ndt_ind++;
  n = c+Coord(-1,0);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;

  ndt_ind++;
  n = c+Coord(0,-1);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;
  

  if (NDTSIZE>4){

    //diagonals:
    ndt_ind++;
    n = c+Coord(1,1);     
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;  

    ndt_ind++;
    n = c+Coord(1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;
  }
  
}

double iCub::contrib::primateVision::ArmXOpt::cmp_ndt(int*ndt_l, int*ndt_r)
{

  int s=0;

  for (int count=0;count<NDTSIZE;count++){
    if(ndt_l[count]==ndt_r[count]){
      s++;
    }
  }

  return ((double)s)/((double)NDTSIZE);

}


void iCub::contrib::primateVision::ArmXOpt::get_rank(Coord c,Ipp8u *im, int w, int*list)
{
  Coord n;
  int i = 0;

  for (int x=-RANKX;x<=RANKX;x++){
    for (int y=-RANKY;y<=RANKY;y++){
      
      n = c+Coord(x,y);
      list[i] = im[n.y*w + n.x];
      i++;

    }
  }

}

double iCub::contrib::primateVision::ArmXOpt::cmp_rank(int*l1, int*l2)
{ 
  int n1 = 0; //number of non-ties for x
  int n2 = 0; //number of non-ties for y
  int is = 0;
  
  int a1,a2,aa;
  
  double tau;//,svar,z,prob;

  for(int j=0;j<RANKSIZE;j++) {
    for(int k=j+1;k<RANKSIZE;k++) {
      a1 = l1[j] - l1[k];
      a2 = l2[j] - l2[k];
      aa = a1*a2;
      if(aa) {
	++n1;
	++n2;
  	
	aa > 0 ? ++is : --is;
	
      } else {
	if(a1) ++n1;
	if(a2) ++n2;
      }
    }
  }
  
  tau = (is) / (sqrt(n1) * sqrt(n2));
  // svar = (4.0 * n + 10.0) / (9.0 * n * (n - 1.0));
  // z = tau / sqrt(svar);
  // prob = erfcc(abs(z) / 1.4142136);

  if (tau < 0.0){tau=0.0;}

  return tau;
}




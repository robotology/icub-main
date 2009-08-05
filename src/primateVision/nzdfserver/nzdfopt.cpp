#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "energy.h"

#include "nzdfopt.h"


#define RANKX  1 //2
#define RANKY  1
#define RANKSIZE 9 //15
//RANKSISE = (RANKX*2+1)*(RANKY*2+1) //15 for (2,1) :)

#define DRANGE    16
#define MIN_TEX   10

#define NZD   1
#define N_NZD 0

#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) < (b) ? (a) : (b))


iCub::contrib::primateVision::NZDFOpt::NZDFOpt(IppiSize im_size_, int psb_in_, Parameters *_params)
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

iCub::contrib::primateVision::NZDFOpt::~NZDFOpt()
{

}


double iCub::contrib::primateVision::NZDFOpt::cmp_rank(int*l1, int*l2)
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


void* iCub::contrib::primateVision::NZDFOpt::create_list(Coord c,Ipp8u *im, int w, int*list)
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




void iCub::contrib::primateVision::NZDFOpt::clear()
{

  ippiSet_8u_C1R(0,out,psb,im_size);

}


void iCub::contrib::primateVision::NZDFOpt::generate_permutation(int *buf, int n)
{
  int i, j;
  
  for (i=0; i<n; i++) buf[i] = i;
  for (i=0; i<n-1; i++)
    {
      j = i + (int) (((double)rand()/RAND_MAX)*(n - i));
      int tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp;
    }
}


void iCub::contrib::primateVision::NZDFOpt::proc(Ipp8u* im_l_,Ipp8u* im_r_,Ipp8u* im_d_, Ipp8u*dog_l_,Ipp8u*dog_r_)
{

  im_l=im_l_;
  im_r=im_r_;
  im_d=im_d_;
  dog_l = dog_l_;
  dog_r = dog_r_;


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

bool iCub::contrib::primateVision::NZDFOpt::is_tex(int*lst){

  int max=0;
  int min=999;

  for (int i=0;i<RANKSIZE;i++){
    min = MIN(min,lst[i]);
    max = MAX(max,lst[i]);
  }


  if ((max-min)>=MIN_TEX){
    return true;
  }
  else {
    return false;
  }

}




int iCub::contrib::primateVision::NZDFOpt::likelihood(Coord c, int label)
{

  //the probability of the images being the way they are,
  //given the hypothesized configuration.
  //return low penalty if likely.


  int penalty = 0;
 

  //GET DISPARITY:
  double d_L = (double)(im_d[c.x + psb_in*c.y]-DRANGE);
  //(-DRANGE..DRANGE)


  //if labeled 'near zero disparity': 
  if (label==NZD){

    //Disparity penalty the furthar away from zero disparity:
    penalty += (int)(params->disparity_penalty * fabs(d_L)/DRANGE);
        
  }

  

  
  //else if labeled !nzd,  
  else{

    //penalty if likely it's at exactly zd:
    //GET ZD-NESS:
    double p_zd=0.1;
    int listl[RANKSIZE];
    int listr[RANKSIZE];
    if (c.x-RANKX>=0 && c.x+RANKX<im_size.width  &&
	c.y-RANKY>=0 && c.y+RANKY<im_size.height ){
      create_list(c,im_l,psb_in,listl);
      create_list(c,im_r,psb_in,listr);
      //if (is_tex(listl) || is_tex(listr)){ //too slow! below from zdfserver:
      if (dog_l[c.x + c.y*psb_in] >= params->bland_dog_thresh ||
	  dog_r[c.x + c.y*psb_in] >= params->bland_dog_thresh ){
	p_zd = cmp_rank(listl,listr);
      }
    }

    //The more likely it's at zd, more penalty:
    penalty += (int)(params->zd_penalty * p_zd);
    
   
  }
  
  return penalty; 
}




int iCub::contrib::primateVision::NZDFOpt::prior_smoothness(Coord p, Coord np, int label, int nlabel)
{

  //SMOOTHNESS STUFF

  //we want segmentation edges to occur preferentially at
  //disparity edges. We can't detect these very accurately, 
  //so where that breaks down, image edges are also OK
  //because disp edges coincide with image edges.. but not all image edges!
  //Disparity map edges also tend to 'oversegment'.


  int penalty = 0;
 

  //if both labels hypothesized same, no penalty. 


  //BUT if hypothesized different, 
  if (label != nlabel){
    
    //If there is no disparity edge, assigning different labels
    //is a bad choice for the nzdf and warrants penalty.
    //Firstly, a disparity edge is also an intensity edge, 
    //so if it's smooth in intensity, it's smooth in disparity.
    //In that case, there's no need to proceed to checking 
    //disparity smoothness, so we can save some computation.
  

  

    //INTENSITY SMOOTHNESS:
    int d_I = im_l[p.x + psb_in*p.y] - im_l[np.x + psb_in*np.y];
    double sigma = 2.0*params->intensity_smoothness_3sigmaon2/3.0;
    double p_int_edge = 1.0 - exp(-(d_I*d_I)/(2.0*sigma*sigma));

    //if it's likely it's NOT an intensity edge, 
    //return BIG penalty immediately, 
    //(can assume p_disp_edge is zero!):
    if (p_int_edge<0.5){
      //assume p_disp_smooth ~= 1.0;
      penalty += (int) (params->smoothness_penalty*(1.0-p_int_edge));
    }




    //else, there's likely to be an edge, 
    //so we're going to have to do some computation
    //to check it's disparity smoothness:
    else{
      

      //DISPARITY SMOOTHNESS:

      double p_d_p[DRANGE*2+1];
      double p_d_np[DRANGE*2+1];
      int listl[RANKSIZE];
      int listr[RANKSIZE];
      Coord pr;
      int ind;

      //check all possible disparities of pixels p and np:
      //find the best smoothness prob (check +-1):
      //location is textured, as is already shown to be edge-like!!

      //p:
      if (p.x-RANKX>=0 && p.x+RANKX<im_size.width  &&
	  p.y-RANKY>=0 && p.y+RANKY<im_size.height ){
	create_list(p,im_l,psb_in,listl);
	for (int i=0;i<2*DRANGE+1;i++){
	  pr.x = p.x+i-DRANGE;
	  pr.y = p.y;
	  if (pr.x-RANKX>=0 && pr.x+RANKX<im_size.width  &&
	      pr.y-RANKY>=0 && pr.y+RANKY<im_size.height ){
	    create_list(pr,im_r,psb_in,listr);
	    p_d_p[i] = cmp_rank(listl,listr);
	  }
	  else{
	    //out of image!
	    p_d_p[i] = 0.1; 
	  }
	}
      }
      else{
	//out of image!
	for (int i=0;i<DRANGE*2+1;i++){
	  p_d_p[i] = 0.1;
	}
      }
      
      //np:
      if (np.x-RANKX>=0 && np.x+RANKX<im_size.width  &&
	  np.y-RANKY>=0 && np.y+RANKY<im_size.height ){
	create_list(np,im_l,psb_in,listl);
	for (int i=0;i<DRANGE*2+1;i++){
	  pr.x = np.x+i-DRANGE;
	  pr.y = np.y;
	  if (pr.x-RANKX>=0 && pr.x+RANKX<im_size.width  &&
	      pr.y-RANKY>=0 && pr.y+RANKY<im_size.height ){
	    create_list(pr,im_r,psb_in,listr);
	    p_d_np[i] = cmp_rank(listl,listr);
	  }
	  else{
	    //out of image!
	    p_d_np[i] = 0.1;
	  }
	}
      }
      else{
	//out of image!
	for (int i=0;i<DRANGE*2+1;i++){
	  p_d_np[i] = 0.1;
	}
      }
      
      
      
      
      //ok, we now have the disparity probabilities for p and np.
      

      //find the most likely 'smooth' combination
      //best -1,0,+1 nzd disp prob combo:
      double p_disp_smooth=0.0;
      //0:
      for (int i=0;i<DRANGE*2+1;i++){
	if (p_d_p[i]*p_d_np[i]>p_disp_smooth){p_disp_smooth = p_d_p[i]*p_d_np[i];}
      }
      //1:
      for (int i=0;i<DRANGE*2;i++){
	if (p_d_p[i]*p_d_np[i+1]>p_disp_smooth){p_disp_smooth = p_d_p[i]*p_d_np[i+1];}
      }
      //-1:
      for (int i=1;i<=DRANGE*2+1;i++){
	if (p_d_p[i]*p_d_np[i-1]>p_disp_smooth){p_disp_smooth = p_d_p[i]*p_d_np[i-1];}
      }
      
      


      //However, there is a special case..
      //If disp was not (confidently) obtained for BOTH n and np, 
      //it is likely that both are beyond DRANGE (background), 
      //so actually should have the same label!
      double p_best = 0.0;
      for (int i=0;i<DRANGE*2+1;i++){
	if (p_d_p[i] >p_best){p_best = p_d_p[i];}
	if (p_d_np[i]>p_best){p_best = p_d_np[i];}
      }
      if (p_best<0.5){
	p_disp_smooth = 1.0;
      }




      //now have disps of p, np; and confidences.    
      penalty +=  (int) (params->smoothness_penalty * (1.0-p_int_edge)*p_disp_smooth);

    }


    
  }
  
  
  return penalty;
  
}




/* computes current energy */
int iCub::contrib::primateVision::NZDFOpt::compute_energy()
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

void iCub::contrib::primateVision::NZDFOpt::expand(int a)
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

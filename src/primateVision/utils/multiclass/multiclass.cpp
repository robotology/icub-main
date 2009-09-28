#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "multiclass.h"
#include "energy.h"



iCub::contrib::primateVision::MultiClass::MultiClass(IppiSize im_size_, int psb_in_, int n_, Parameters *_params)
{


  params = _params;   
  nmaps=n_;
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

iCub::contrib::primateVision::MultiClass::~MultiClass()
{

}


void iCub::contrib::primateVision::MultiClass::clear()
{

  ippiSet_8u_C1R(0,out,psb,im_size);

}


void iCub::contrib::primateVision::MultiClass::generate_permutation(int *buf, int n)
{
  int i, j;
  
  for (i=0; i<n; i++) buf[i] = i;
  for (i=0; i<n-1; i++)
    {
      j = i + (int) (((double)rand()/RAND_MAX)*(n - i));
      int tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp;
    }
}


void iCub::contrib::primateVision::MultiClass::proc(Ipp8u* im_, Ipp8u** prob_)
{

  im=im_;
  prob=prob_;

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



int iCub::contrib::primateVision::MultiClass::likelihood(Coord c, int d)
{

  //the probability of the images being the way they are,
  //given the hypothesized configuration.
  //return low penalty if likely.

  //prob[x][y] is {0..255}.

  int penalty = (int)( params->data_penalty * 
		       (1.0 - (prob[d][c.y*psb_in + c.x]/255.0)) );
  return penalty;

}




int iCub::contrib::primateVision::MultiClass::prior_intensity_smoothness(Coord p, Coord np, int label, int nlabel)
{

  //configuration preferences
  //intensity smoothness: similar neighbours should have the same label.
  //NEIGHBOURHOOD SYSTEM
  
  int penalty;

  //if hypothesized as the same, no penalty
  if (label == nlabel) {penalty = 0;}


  //Otherwise, if hypothesized different:
  else{

    //Always prefer continuous soloutions,
    //so immediately penalise hypothetical solutions involving
    //neighbours in different classes, this will help reduce segmented area:
    penalty = params->smoothness_penalty_base;

    //INTENSITY SMOOTHNESS:
    int d_I = im[p.x + psb_in*p.y] - im[np.x + psb_in*np.y];
    double sigma = 2.0*params->smoothness_3sigmaon2/3.0;
    double p_int_edge = 1.0 - exp(-(d_I*d_I)/(2.0*sigma*sigma));
    
    //If it's likely that it's an intensity edge, 
    //return less additional penalty.
    //if it's not much of an edge, return 
    //more additional penalty:
    penalty += (int) (params->smoothness_penalty*(1.0-p_int_edge));
  }
 

  return penalty;
}




/* computes current energy */
int iCub::contrib::primateVision::MultiClass::compute_energy()
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
		E += prior_intensity_smoothness(p, q, d, dq);
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

void iCub::contrib::primateVision::MultiClass::expand(int a)
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
	      E00 = prior_intensity_smoothness(p, q, d, dq);
	    if (var != VAR_ACTIVE)
	      E0a = prior_intensity_smoothness(p, q, d, a);
	    if (qvar != VAR_ACTIVE)
	      Ea0 = prior_intensity_smoothness(p, q, a, dq);
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

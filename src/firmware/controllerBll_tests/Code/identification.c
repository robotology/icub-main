#include "identification.h"
#include "can1.h"

//IDENTIFICATION VARIABLES

Int16 tCount	[JN]=INIT_ARRAY(0);
Int16 timeCount	[JN]=INIT_ARRAY(0);
float w			[JN]=INIT_ARRAY(1.0);
float w_step	[JN]=INIT_ARRAY(1.0);
float wt		[JN]=INIT_ARRAY(0.0);
float wt0		[JN]=INIT_ARRAY(0.0);
float f			[JN]=INIT_ARRAY(0.0);
float f0		[JN]=INIT_ARRAY(1.0);
float fin		[JN]=INIT_ARRAY(0.0);
float k			[JN]=INIT_ARRAY(1.0);
float alpha		[JN]=INIT_ARRAY(0.0);
Int16 cnt		[JN]=INIT_ARRAY(0);
Int16 deltap	[JN]=INIT_ARRAY(0);
Int16 PWMRamp	[JN]=INIT_ARRAY(0);
Int16 _fout		[JN]=INIT_ARRAY(0);

float PI=3.14159265359;
Int16 _Fmax=150;
bool openloop_identif = true;


double sin(double rad)
{
	double sine;
	if (rad < 0)
	sine = rad*(1.27323954 + .405284735 * rad);
	else
	sine = rad*(1.27323954 - 0.405284735 * rad);

	if (sine < 0)
	sine = sine*(-0.225 * (sine + 1) + 1);
	else
	sine = sine*(0.225 * (sine - 1) + 1);
	return sine;
}

void compute_identif_wt(int j)
{
	wt[j]=(2*PI*w[j]*tCount[j]/1000+wt0[j]);
	_fout[j]=10*w[j];
  	if(wt[j]>=PI)
   	{
    	tCount[j]=0 ;
    	
       	wt0[j]=wt[j]-2*PI;
       	wt[j]=(wt0[j]);
    }
    
    tCount[j]++;
   	timeCount[j]++;
   	
    //    w=(float)_ki[1]/100.0;//w+1;//
       	//
     if(timeCount[j]>4000) //millisec
   	{  	
   		w[j]=w[j]+w_step[j];
     	if(w[j]>_Fmax)  
     	{
     		w[j]=_Fmax;	
     	}
       	
       	timeCount[j]=0;
    }  
}

void compute_sweep_wt(int j)
{
	f[j]=f0[j]+fin[j]+k[j]/2*tCount[j]/1000;
	if(f[j]>100) f[j]=100;
		
	wt[j]=2*PI*f[j]*tCount[j]/1000+alpha[j];
	tCount[j]++;
		
	if(wt[j]>=PI)
	{
		fin[j]=fin[j]+k[j]/2*tCount[j]/1000;
		alpha[j]=wt[j]-2*PI;
    	tCount[j]=0;
	}
    
	_fout[j]=100*f[j];
 	timeCount[j]++;
}

void reset_identif(int j)
{
	 tCount[j] = 0;
	 _fout[j]=0;
	 w[j]=1.0;
	 w_step[j]=1;
	 wt[j]=0.0;
	 wt0[j]=0.0;
	 f[j]=0;
	 f0[j]=1.0;
	 fin[j]=0.0;
	 k[j]=1.0;	
	 alpha[j]=0;
	 PWMRamp[j]=0;
	 timeCount[j] = 0;
	 cnt[j]=0;
	 deltap[j]=0;
	 PWMRamp[j]=0;
}
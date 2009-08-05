#include "hist.h"
#include "stdlib.h"
#include <stdio.h>


hist_performer::hist_performer()
{
	unsigned int i=0;
	for (i=0; i<BINS; i++)	hist[i]=0;
	number_of_samples =0;
}

hist_performer::~hist_performer()
{
}

bool hist_performer::add_sample(signed short sample)
{
	number_of_samples++;
	float delta=(65535/BINS);
	long unsigned int bin = sample/delta;
	hist[bin]++;

	if ((number_of_samples%500)==0) return true;
	else return false;
}

void hist_performer::do_hist()
{
  system("cls");
  int i=0, j=0; 
  int blocks=0;
  int max=0;
  static long unsigned int maxi=0;
  static unsigned int block_step=100;

  for ( i = 0; i < BINS; i++ )
  {
	  if (hist[i]>max) 
	  {
		  max=hist[i];
		    if (number_of_samples<3000) maxi=i;
	  }
  }

  for ( i = maxi-50; i < maxi+50; i++ )
  //for ( i = 0; i < 100; i++ )
  {
/*
		if (fft[i]>120)
			printf ( "*"); 
		else
			printf ( " "); 
*/
	 	printf ( "  %12d %12d	",  i, hist[i]);
		if (max>block_step*100) block_step=block_step*2;
		blocks=hist[i]/block_step;
		//if (blocks>100) blocks = 100;
		for (j=0;j<blocks;j++) 
		{
			printf ( "%c",178);
		}
		for (;j<100;j++)
		{
			//printf ( ".");
		}
		printf ( "\n");
  }
  printf ("1 block = %d units\n", block_step);

}
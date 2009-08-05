#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/time.h>

struct timeval ttv, tv;
struct timezone ttz; 


/*****************************************************************/
void resetTimer(void)
{
        gettimeofday(&ttv, &ttz);
	//fprintf(stderr,"reset: %dsec %dusec\n",ttv.tv_sec,ttv.tv_usec);
}

/*****************************************************************/
double getTimer(void)
{
  double dtime;
        gettimeofday(&tv, &ttz);
	dtime = (tv.tv_sec - ttv.tv_sec)*1000.0;
	dtime += ((tv.tv_usec - ttv.tv_usec) / 1000.0);

        return dtime;
}



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "MMXConvolve.h"
#include "MMXConvert.h"
#include "MMXFlow.h"
#include "MMXUtils.h"
#include "MMXKernel.h"

#define MIN_KER 3
#define MAX_KER 10
#define MAX_IMG 512

struct timeval init;

void initTime() {
	struct timezone ttz;
	gettimeofday(&init, &ttz);
}
  
float getTime() {
	struct timezone ttz;
	struct timeval now;

	gettimeofday(&now, &ttz);
	return (now.tv_sec - init.tv_sec)*1000 +
		(now.tv_usec - init.tv_usec)/1000.0;
}

inline unsigned long long get_cpuctr(void)
{
	unsigned long long value;
	/* read the pentium cycle counter */
	__asm__(".byte 0x0f,0x31"
		:	"=a"(((unsigned long *) &value)[0]),
		"=d"(((unsigned long *) &value)[1]));
	return value;
}

int bench(char *msg, mmtype type, int flag) {
	int i,in,kn;
	MMXMatrix *I,*K,*C;
	mmtype type2;
	float t;
	int n;
	unsigned long long clocks=0;

	if (type==MMT_F_32) type2 = type;
	else type2 = MMT_S_32;

	printf("%s\n",msg);
	printf(" MOPS    ");
	for (kn=MIN_KER;kn<=MAX_KER;kn+=2)
		printf(" %02dx%02d  ",kn,kn);
	printf("\n");

	for (in=16;in<=MAX_IMG;in*=2) {
		printf("%03dx%03d: ",in,in);
		for (kn=MIN_KER;kn<=MAX_KER;kn+=1) {
			//n = (in-kn+1)*(in-kn+1)*(2*kn*kn-1);
			n=in*in;
			if (kn<=in) {
				I = MMXMatrixRandom(type,in,in);
				K = gaussKernel(type,kn,kn,kn,kn);
				C = MMXMatrixAlloc(type2,in-kn+1,in-kn+1);
				if (flag==0) {
					initTime();
					MMXConvolve(I,K,C);
					t = getTime();
				}
				else {
					//initTime();
					clocks = get_cpuctr();
					for (i=0;i<30;i++) MMXConvolve(I,K,C);
					clocks = (get_cpuctr() - clocks)/30.0;
					//t = getTime()/30.0;
				}
				free(I);
				free(K);
				free(C);
				//printf("%#07.3f ",t);
				//printf("%#07.2f ",n/(t*1000.0));
				printf("%#07.3f ",(float)clocks/(float)n);
			}
			else {
				printf("------- ");
			}
		}
		printf("\n");
	}
	printf("\n");
	return 0;
}

int main() {
	mmtype type=MMT_F_32;

	type = MMT_F_32;

	printf("Convolution benchmark using MMX/SSE    author S. Rougeaux 99.10.01\n");
	printf("Pentium III 600Mhz (dual), linux kernel 2.2.12, SMP, patched for SSE, Redhat 6.0, glibc 2.1\n");
	printf("Note: single threaded, the second processor is not used during computation\n");
	printf("Result in MOPS (Mega Operations Per Second) based on the formula\n");
	printf("res = (i-k+1)*(i-k+1)*(2*k*k-1) / t\n");
	printf("i: image size\n");
	printf("k: kernel size\n");
	printf("t: convolution time\n");
	printf("\n");
	//bench("[MMX integer 16 bits image and kernel, 32 bits result, one shot, gaussian kernel]",MMT_S_16,0);
	//bench("[SSE float   32 bits image and kernel, 32 bits result, one shot, gaussian kernel]",MMT_F_32,0);
	//bench("[MMX integer 16 bits image and kernel, 32 bits result, average 30 shots, gaussian kernel]",MMT_S_16,1);
	//bench("[SSE float   32 bits image and kernel, 32 bits result, average 30 shots, gaussian kernel]",MMT_F_32,1);

	return 0;
}

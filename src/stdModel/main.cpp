#include <ctime>
#include <conio.h>
#include <stdio.h>

#include <yarp/String.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/property.h>
#include <yarp/os/network.h>
#include <yarp/os/random.h>
#include <yarp/sig/vector.h>
#include <yarp/sig/image.h>
#include <yarp/sig/imagefile.h>

#include "YARPhmax.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


//#define FROMFILE
//using namespace _logpolarParams;

BufferedPort<ImageOf<PixelRgb> > img_in;
BufferedPort<Vector> vec_out;


void cropImages(ImageOf<PixelMono> *src, unsigned char * dst, int newRow, int newCol)
{
	int rmax=src->height();
	int cmax=src->width();

	int rOff=(rmax-newRow)/2;
	int cOff=(cmax-newCol)/2;
	
	for (int r=0; r<newRow; r++)
		for (int c=0; c<newCol; c++)
			dst[newCol*r+c]=src->pixel(c+cOff, r+rOff);
}


void cropImages(ImageOf<PixelMono> *src, ImageOf<PixelMono> *dst, int newRow, int newCol)
{
	int rmax=src->height();
	int cmax=src->width();

	int rOff=(rmax-newRow)/2;
	int cOff=(cmax-newCol)/2;
	
	/*for (int r=0; r<newRow; r++)
		for (int c=0; c<newCol; c++)
			dst.pixel(c, r)=src.pixel(c+cOff, r+rOff);*/
	for (int r=0; r<newRow; r++)
		memcpy(dst->getPixelAddress(0, r),src->getPixelAddress(cOff, r+rOff), sizeof(PixelMono)*newCol);
}


int main (int argc, char *argv[])
{
	Network::init();
	Random::seed(time(NULL));
	
	String basename="/hmax";
		
	//YARPParseParameters::parse(argc, argv, "-name", basename);

	String inName1=basename+"/i:img";
	String outName1=basename+"/o:vec";

	
	img_in.open(inName1.c_str());
	vec_out.open(outName1.c_str());

	Bottle tmpBot;

	ImageOf<PixelMono> *leftImg;
	ImageOf<PixelRgb> *imgCC1;
	ImageOf<PixelMono> *imgMC1;

	const int newCol=100;
	const int newRow=100;
	
	const int numPatch=20;
	
	YARPHMax hmax1;
	hmax1.init(numPatch, newCol, newRow);

	double resp[numPatch];
	Vector example(21);
	
	leftImg = new ImageOf<PixelMono>;
	imgMC1 = new ImageOf<PixelMono>;

	leftImg->resize(newCol, newRow);

	
#ifndef FROMFILE
	img_in.read();
	printf("Cameras connected!\n");
#endif

	if (hmax1.loadPatches("d:\\yarp2\\zgarbage\\hmax\\rand_patches.hmx")!=0) {
		printf("File with saved pathes not found: gathering from images...");
		int rim;
		do {
			Time::delay(2);

			#ifdef FROMFILE
				sig::file::read(imgMC1,"d:\\yarp2\\zgarbage\\hmax\\img.pgm");
			#else
				printf("Reading image 1...");
				imgCC1=img_in.read();
				imgMC1->copy(*imgCC1);
				printf("done!\n");
			#endif

			char *root = ACE_OS::getenv("YARP_ROOT");
			char path[256];
		
			sprintf (path, "%s/zgarbage/", root);

			//printf("Cropping...");
			cropImages(imgMC1, leftImg, newRow, newCol);
			//printf("done!\n");

			//printf("Gathering patches...");
			rim=hmax1.gatherPatch((IplImage*)leftImg->getIplImage(),2);
			//printf("done!\n");
			
		} while (rim!=0);
		printf("done!\n");

		hmax1.savePatches("d:\\yarp2\\zgarbage\\hmax\\rand_patches.hmx");
	}
	hmax1.endGather();
	
	double start = Time::now();
	double cur;
	int frame=0;

	while (1) {
#ifdef FROMFILE
		sig::file::read(imgMC1, "d:\\yarp2\\zgarbage\\hmax\\img.pgm");
#else
		//printf("Reading image 1...");
		imgCC1=img_in.read();
		imgMC1->copy(*imgCC1);
		//printf("done!\n");
#endif

		cropImages(imgMC1, leftImg, newRow, newCol);

		hmax1.apply((IplImage*)leftImg->getIplImage(),resp);

		/*Vector tmp(numPatch);
		for (int i=0; i<numPatch; i++)
			tmp[i]=resp[i];

		vec_out.prepare() = tmp;
		vec_out.write();*/


		if ( _kbhit() ) {
			switch ( _getch() ) {
			case 'o':
				printf("Object.\n");
				{for (int i=0; i<numPatch; i++)
					example[i]=resp[i];}
				example[numPatch]=1;
				vec_out.prepare() = example;
				vec_out.write();
				break;
			case 'b':
				printf("Background.\n");
				{for (int i=0; i<numPatch; i++)
					example[i]=resp[i];}
				example[numPatch]=-1;
				vec_out.prepare() = example;
				vec_out.write();
				break;
			case 'p':
				printf("Predict.\n");
				{for (int i=0; i<numPatch; i++)
					example[i]=resp[i];}
				vec_out.prepare() = example;
				vec_out.write();
				break;
			default:
				printf("Unrecognised keyboard command.\n");
			}
		}

		frame++;
		
		if (frame%100==0) {
			cur = Time::now();
			fprintf(stdout, "\naverage frame time: %f frame #%d acquired\n", (cur-start)/frame, frame);
		}

		//if (frame==100) break;
	}

	img_in.close();
	vec_out.close();
	
	Network::fini();
	return 0;
}

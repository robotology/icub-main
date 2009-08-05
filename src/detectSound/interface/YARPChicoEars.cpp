#include "YARPChicoEars.h"

YARPChicoEars::YARPChicoEars(char *portname, char *netname, char *grabname)
{
	
   // char OutputFileName[]="sound.dat";
  //  outFile = fopen (OutputFileName,"w");


}


//*************************************************************************************************************

int YARPChicoEars::init()
{

	Network::init();

	port.setStrict(false);
    port.open("/sound_recorder");
   
	yarp::os::Network::connect("/sound_grabber","/soundloc:i","tcp");
	printf("connected port\n\n");

	return 0;
}

//*************************************************************************************************************



int YARPChicoEars::grab() {

	return 1;
}


void YARPChicoEars::separateEars(double *l_Ear, double *r_Ear)
{	
	
	
	int i,j;
	double val;

	double *normalizedDoubleSamplesChan0;
    double *normalizedDoubleSamplesChan1;
	


	Sound *input = port.read();

	if (input!=NULL)
	{


		//normalizedDoubleSamplesChan0 = (double*) malloc(sizeof(double) * (input->getSamples()));
		normalizedDoubleSamplesChan0 = (double*) malloc(sizeof(double) * (8192/4));
		normalizedDoubleSamplesChan1 = (double*) malloc(sizeof(double) * (8192/4));

			//divide-se por 32768 para normalizar        
			//for (i=0;i<input->getSamples();i++)

			//printf("\n\nnumber of samples: %d\n\n", input->getSamples());
			for (i=0;i<(8192/4);i++)
			{
				for(j = 0; j < input->getChannels(); j++)
				{
				
					val=(double)input->get(i,j)/32768;
					
					if(j==0) {
						normalizedDoubleSamplesChan0[i]=val;
						//fprintf(outFile, "%1.6f ", val);
					}
					else {
						normalizedDoubleSamplesChan1[i]=val;
						//fprintf(outFile, "%1.6f ", val);
					}
					
				}
			//	fprintf(outFile, "\n");

			}
		memcpy(l_Ear,normalizedDoubleSamplesChan1,sizeof(double)*(8192/4));
		memcpy(r_Ear,normalizedDoubleSamplesChan0,sizeof(double)*(8192/4));

		free(normalizedDoubleSamplesChan0);
		free(normalizedDoubleSamplesChan1);
			
	}
	

}


int YARPChicoEars::close()
{
	

	//-------closing output files ------------------
   // fclose(outFile);
//	printf("\n\nrecording terminated\n");
	//-----------------------------------

	yarp::os::Network::disconnect("/sound_grabber","/sound_recorder");
	printf("\n disconnected port\n");
    Network::fini();

	return 1;
}

YARPChicoEars::~YARPChicoEars()
{


}

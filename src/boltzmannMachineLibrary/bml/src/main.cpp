#include <iCub/MachineBoltzmann.h>

#define SAVE



/**
* The main is called just in the testing phase of the library
* An application can be created in this project in order to test the library
*/


int main(int argc, char *argv[]) {
#ifdef LOAD 	
	MachineBoltzmann *mb=new MachineBoltzmann();
	cout<<"Loading configuration from file"<<endl;
	mb->loadConfiguration();		
#else
	MachineBoltzmann *mb=new MachineBoltzmann(2);
	Layer *layerA=new Layer("LA",2,6);
	mb->addLayer(*layerA);
	Layer *layerB=new Layer("LB",2,6);
	mb->addLayer(*layerB);
	mb->interconnectLayers(*layerA,*layerB);

	int numhid=4*12;
	Matrix data(1,12);
	data(0,0)=0;
	data(0,1)=1;
	data(0,2)=10;
	data(0,3)=11;
	data(0,4)=20;
	data(0,5)=21;
	data(0,6)=30;
	data(0,7)=31;
	data(0,8)=40;
	data(0,9)=41;
	data(0,10)=50;
	data(0,11)=51;

	mb->rbm((Matrix)data,layerA,numhid);
	//mb->migrateLayer(*layer);
	//mb->interconnectLayer(2);
	//mb->evolveClamped(2,1);
	//mb->saveConfiguration();

	/*mb->addClampedUnit(153,1);
	mb->addClampedUnit(13,1);
	mb->addClampedUnit(14,1);
	for(int i=0;i<150;i++)
		mb->evolveFreely(2,1);
	for(int i=0;i<150;i++)
		mb->evolveClamped(2,1);
	for(int i=0;i<1000;i++){
		mb->setProbabilityFreely();
		mb->setProbabilityClamped();
	}
	mb->learn();*/
	//getch();
#endif
	
	return 0;
}
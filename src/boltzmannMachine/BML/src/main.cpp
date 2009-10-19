#include <iCub/MachineBoltzmann.h>
//#include <conio.h>

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
	Layer *layer=new Layer("L2",1,6);
	//mb->addLayer(*layer);
	//mb->migrateLayer(*layer);
	//mb->interconnectLayer(2);
	//mb->evolveClamped(2,1);
	//mb->saveConfiguration();
	mb->addClampedUnit(153,1);
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
	mb->learn();
	//getch();
#endif
	
	return 0;
}
#include "TargetInfo.h"

TargetInfo::TargetInfo(int dofs, int size)
{
	this->nbStates = size+1; 
	this->nbDOFs = dofs;
}

TargetInfo::~TargetInfo(){}

bool TargetInfo::Initialize(Property FromFile)
{
	char drumName[255];
	vector<double> oneDrum;
		  
	//getting idle state info	  
	Bottle& Target= FromFile.findGroup("Idle");
	if(Target.size()!=nbDOFs+1)
	{ 
		printf("WARNING:: Wrong size of the target angles (idle) in config file\n");
		return false;
	}
	else
	{
		printf("Idle: ");
		oneDrum.clear();
		for(int k=0; k<nbDOFs; k++) 
		{		
			oneDrum.push_back(3.14/180.0*Target.get(k+1).asDouble());
			printf("%4.2f ", oneDrum[k]);
		}
		printf("\n");
			
		drumsPos.push_back(oneDrum);
	}
	Target.clear();
	
	//getting target angles for each drums
	for(int i=1; i<nbStates; i++)
	{
		sprintf(drumName, "Drum_%d", i);			  
	    Target= FromFile.findGroup(drumName);
	    if(Target.size()!=nbDOFs+1)
		{ 
			printf("WARNING:: Wrong size of the target angles for %d in config file\n", i);
			return false;
		}
		else
		{
			printf("%s: ", drumName);

			oneDrum.clear();
			for(int k=0; k<nbDOFs; k++) 
			{		
				oneDrum.push_back(3.14/180.0*Target.get(k+1).asDouble());
				printf("%4.2f ", oneDrum[k]);
			}
			printf("\n");
			
			drumsPos.push_back(oneDrum);
		}
		Target.clear();
	}
	
	//getting drumIDs
	Target = FromFile.findGroup("DrumIDs");
	printf("DrumIDs: ");
	if(Target.size()!=nbStates)
	{ 
		printf("WARNING:: Wrong size of drum ids in config file\n");
		return false;
	}
	else
	{
		drumIDs.push_back(-1); //idle state, no ID
		for(int i=0; i<nbStates-1; i++)
		{
			drumIDs.push_back(Target.get(i+1).asInt());
			printf("%d ", drumIDs[i]);
		}
		printf("\n");
	}
	Target.clear();
	
	//getting muOn and muOff
	Target= FromFile.findGroup("muOn");
	printf("muOn: ");
	if(Target.size()!=nbDOFs+1)
	{ 
		printf("WARNING:: Wrong size for muOn in config file\n");
		return false;
	}
	else
	{
		for(int k=0; k<nbDOFs; k++) 
		{		
			muOn.push_back(Target.get(k+1).asInt());
			printf("%d ", muOn[k]);
		}
		printf("\n");			
	}
	Target.clear();
	
	Target= FromFile.findGroup("muOff");
	printf("muOff: ");
	if(Target.size()!=nbDOFs+1)
	{ 
		printf("WARNING:: Wrong size for muOff in config file\n");
		return false;
	}
	else
	{
		for(int k=0; k<nbDOFs; k++) 
		{		
			muOff.push_back(Target.get(k+1).asInt());
			printf("%d ", muOff[k]);
		}
		printf("\n");			
	}
	Target.clear();
	
	
	
	
	return true;	
}

void TargetInfo::UpdateInfo()
{
	Bottle *command = ik_port.read(false);
    if(command!=NULL)
    {   
    	printf("\nCommand is %s", command->toString().c_str());
    	fflush(stdout);
		Bottle *list = command->get(0).asList();
		int drumID = list->get(0).asInt();
		for(int i=0; i<nbStates; i++)
		{
			if(drumIDs[i]==drumID) 
			{
				printf("\n NEW TARGET ANGLES received for drum %d, id %d: ",i, drumID); 
				for(int j=0; j<nbDOFs; j++)
				{
					drumsPos[i][j]=list->get(j+1).asDouble();
					printf("%f ", drumsPos[i][j]);
				}												
			}
		}
		printf("\n");
		command->clear();				
	}

}

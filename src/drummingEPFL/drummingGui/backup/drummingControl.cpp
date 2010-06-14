#include "DrummingControl.h"
#include <iostream>
using namespace std;
using namespace yarp::os;

DrummingControl::DrummingControl()  
{
    for(int i=0; i<NB_PARTS; ++i)
    {
		parts[PARTS[i]].customBeat = 0;
		parts[PARTS[i]].isUsed = false;
		parts[PARTS[i]].currentPhase = 0;
		parts[PARTS[i]].partitionPort = new BufferedPort<Bottle>;
		parts[PARTS[i]].phasePort = new BufferedPort<Bottle>;
    }

    ///init variables
    currentTempo = 0.5; //in Hz

        
    ///we read the partition folder
    //updatePartitionFolder();
    
    //we reset the custom stuff and the partition
	//currentPartition[i].frequency = 0;
 //   for(int i=0; i < NB_PARTS; i++)
 //   {
 //       double pini[11] = {0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0};
 //       vector<double> initPart(pini, pini + 11);
 //       for(int j=0; j<SCORE_SIZE; j++)
 //       {
 //           currentPartition[i].push_back(pini[i]);	
 //       }
 //   }
    
}

vector<string> DrummingControl::Init(void)
{
    vector<string> enabledParts;

    //we open commnunication ports
    bool ok= Network::checkNetwork();
    if(!ok)
    {
        cout << "Problem with the network" << endl;
		return enabledParts;
    }
    Network::init();
    
    interactivePort.open("/interactive/out");
    Network::connect("/interactive/out", "/interactive/in");
    
	for(TParts::iterator it = parts.begin(); it != parts.end(); ++it)
    {
		string partName = it->first;
        string scoreInPort = "/" + partName +  "/score/in";
        string scoreOutPort = "/" + partName +  "/score/out";
		it->second.partitionPort->open(scoreOutPort.c_str());
        bool ok = Network::connect(scoreOutPort.c_str(), scoreInPort.c_str());
        if(!ok)
        {
            cout << "limb " << partName.c_str() << " not used" << endl;
			it->second.isUsed = false;
        }
        else
        {
            enabledParts.push_back(partName);
            it->second.isUsed = true;
            string phaseInPort = "/" + partName +  "/phase/in";
            string phaseOutPort = "/" + partName +  "/phase/out";
			it->second.phasePort->open(phaseOutPort.c_str());
            bool ok2 = Network::connect(phaseOutPort.c_str(),phaseInPort.c_str());
            if(!ok2)
            {
                cout << "Problems connecting to phase port port for part" << partName.c_str() << endl;
            }
        }
    }
    return enabledParts;
}


DrummingControl::~DrummingControl(void)
{
	for(TParts::iterator it = parts.begin(); it != parts.end(); ++it)
    {
        delete it->second.partitionPort;
        delete it->second.phasePort;
    }
}


void DrummingControl::SendPartition(void)
{
	for(TParts::iterator it = parts.begin(); it != parts.end(); ++it)
	{
		if(it->second.isUsed)
		{
			Bottle &partitionBot = it->second.partitionPort->prepare();
		}
	}
    for(int i =0;i<nbparts;i++)
    {
        if(usedPart[i])
        {    
            Bottle &bot = partitionsPort[i].prepare();
            bot.clear();
            cout << "sending score part " << i << ":   ";
            for(int k=0;k<ScoreSize;k++)
            {
                bot.addInt((int)currentPartition[i][k]);
                cout << (int)currentPartition[i][k] << " ";
            }
            cout << endl;
            partitionsPort[i].write();

            Bottle &bot2 = phasePort[i].prepare();
            bot2.clear();
			cout << "sending phase shift information for part " << i <<":    ";
            for(int k=0; k<ScoreSize;k++)
            {
                bot2.addDouble(currentPartition[i+6][k]);
                cout << currentPartition[i+6][k] << " ";	    
            }
            cout << endl;
            phasePort[i].write();
        }	
    }	
    
    Bottle& bot = interactivePort.prepare();
    bot.clear();
    cout << "sending frequency:  ";
    for(int k=0; k<ScoreSize;k++)
        {
            bot.addDouble(currentPartition[5][k]);
            cout << currentPartition[5][k] << " ";	    
        }
    cout << endl;
    interactivePort.write();
}




#include <iostream>
#include <fstream>
#include <sstream>

#include "drummingControl.h"

using namespace std;
using namespace yarp::os;

void DrummingControl::Close()
{
	interactivePort.close();
    for(int i=0;i<nbparts;i++)
    {
		if(usedPart[i])
		{
			partitionsPort[i].close();
			phasePort[i].close();
		}
    }
}

vector<string> DrummingControl::Init()
{
	vector<string> enabledParts;
    nbparts =5;
    string parts[5] = {"left_arm","right_arm","left_leg","right_leg","head"};
    
    ScoreSize=16;
    ///init variables
    this->currentTempo = 0.5; //in Hz
    for(int i=0;i<nbparts;i++)
        this->currentPhase[i] = 0.0;
         
    //we open commnunication ports
    bool ok= Network::checkNetwork();
    if(!ok)
    {
		exit(-1);
    }
    Network::init();
    
    interactivePort.open("/interactive/out");
    Network::connect("/interactive/out","/interactive/in");
    
    for(int i=0;i<nbparts;i++)
    {
        string tmp_in = "/";
        tmp_in += parts[i];
        tmp_in += "/score/in";
        string tmp_out = "/";
        tmp_out += parts[i];
        tmp_out += "/score/out";
        partitionsPort[i].open(tmp_out.c_str());
        bool ok = Network::connect(tmp_out.c_str(),tmp_in.c_str());
        if(!ok)
        {
            cout << "limb " << parts[i].c_str() << " not used" << endl;
            usedPart[i] = false;
        }
        else
        {
			enabledParts.push_back(parts[i]);
            usedPart[i]=true;
            string tmp_in = "/";	
            tmp_in += parts[i];
            tmp_in += "/phase/in";
            string tmp_out = "/";
            tmp_out += parts[i];
            tmp_out += "/phase/out";
            phasePort[i].open(tmp_out.c_str());
            bool ok2= Network::connect(tmp_out.c_str(),tmp_in.c_str());
            if(!ok2)
			{
				cout << "Problems connecting to phase port port for part" << parts[i].c_str() << endl;
			}
        }
    }
    
    //we reset the custom stuff and the partition
    for(int i=0;i<2*nbparts+1;i++)
    {
        double pini[11]={0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0};
        currentPartition[i].clear();
        for(int j=0;j<ScoreSize;j++)
            currentPartition[i].push_back(pini[i]);	
    }
    
    for(int i=0; i<nbparts;i++)
    {
        customBeat[i]=0;
    }
	return enabledParts;
}







void DrummingControl::SendPartitions()
{
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


void DrummingControl::StopPartition()
{
    double pfin[11]={0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0};
    for(int i=0;i<nbparts;i++)
    {	
		currentPartition[i].clear();
		for(int j=0;j<ScoreSize;j++)
		{
			currentPartition[i].push_back(pfin[i]);
		}
    }        
}





void DrummingControl::SendParameters()
{
    double phase[5];
   
    phase[4]=0.0;
    phase[3] = 0.0;
    phase[2] = currentPhase[1];
    phase[1] = currentPhase[2];
    phase[0] = phase[1] + currentPhase[0];
    
    Bottle& bot = interactivePort.prepare();
    bot.clear();
    for(int i=0; i<ScoreSize;i++)
    {
	bot.addDouble(currentTempo);
	currentPartition[5][i]=currentTempo;
    }
    interactivePort.write();
    
    for(int k=0; k<nbparts; k++)
    {
        Bottle& bot = phasePort[k].prepare();
        bot.clear();
        for(int i=0; i<ScoreSize;i++)
		{
			bot.addDouble(phase[k]);
			currentPartition[k+6][i]=phase[k];
		}
        phasePort[k].write();	
    }
}


void DrummingControl::PlayCustom(int i)
{
    currentPartition[i].clear();	
    for(int j=0;j<ScoreSize;j++)
	{
		currentPartition[i].push_back(customBeat[i]);
	}
    SendPartitions();
}

void DrummingControl::PlayAllPartsCustom(void)
{
	for(int i=0; i<nbparts; i++)
    {
		PlayCustom(i);
    } 
}




void DrummingControl::StopCustomPlay(void)
{
	for(int i=0; i<nbparts; i++)
    {
		customBeat[i] = 0;
		PlayCustom(i);
    }
}

void DrummingControl::PlayPartCustom(int partID, int beat)
{
	customBeat[partID] = beat;
    PlayCustom(partID);
}
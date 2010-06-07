#include "drum.h"
#include <iostream>

using namespace std;

drum::drum()
{	
  parts[0]="left_arm";
  parts[1]="right_arm";
  parts[2]="left_leg";
  parts[3]="right_leg";
  parts[4]="head";

  for(int i=0;i<this->nbparts;i++)
    beat[i] =-1; 
}

drum::~drum()
{
  for(int i=0; i<nbparts; i++)
  {
  	if(ok[i])
	{
	  param_port[i].close();
	  check_port[i].close();
	  score_port[i].close();
	}
  }
  clock_port.close();
}


//**************SENDING PARAMETERS TO THE GENERATOR MODULES*********************************************************//

void drum::sendNewParameterSet(int id, double phase, double freq, int i)
{
    Bottle& paramBot = param_port[i].prepare();
    paramBot.clear();  
    paramBot.addDouble(id);//drum id
    paramBot.addDouble(freq); //freq
    paramBot.addDouble(phase);//phase shift
    param_port[i].write();
}

// GETTING RHYTHM AND PHASE SHIFT FROM THE MANAGER MODULE

int drum::getRhythm(Bottle *Rhythm, int beat)
{  
    for (int j=0; j<Rhythm->size(); j++) {
        int indiceScore=(beat+j)%sizeScore;
        rhythmParam[j]= Rhythm->get(j).asDouble();}

    //GETTING OUT OF THE LOOP - NEGATIVE FREQUENCY MAKES THE MODULE CLOSE
    if(rhythmParam[beat]<0)
        {
            return 0;
        }
    else return 1;   
}

//***opening/connecting ports***********//

void drum::openPort(int i, BufferedPort<Bottle> *port, ConstString port_name, int out, int connect)
{
    char temp_in[nbparts][255];
    char temp_out[nbparts][255];
  
    sprintf(temp_out[i], "/%s/%s/out", parts[i].c_str(), port_name.c_str());
    sprintf(temp_in[i], "/%s/%s/in", parts[i].c_str(), port_name.c_str()); 

    if(out==1)port[i].open(temp_out[i]);
    else port[i].open(temp_in[i]);

    if(connect==1)
        {
            ok[i] = Network::connect(temp_out[i], temp_in[i], "tcp");
            if(!ok[i]) 
                {
                    port[i].close();
                    printf("Failed to connect to %s for part %s\n", port_name.c_str(), parts[i].c_str());
                }
        }
}

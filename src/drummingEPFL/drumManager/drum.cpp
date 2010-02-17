#include "drum.h"
#include <iostream>

using namespace std;

drum::drum(){

  parts[0]="left_arm";
  parts[1]="right_arm";
  parts[2]="left_leg";
  parts[3]="right_leg";
  parts[4]="head";

  soundFeedback =0;

  for(int i=0;i<this->nbparts;i++)
    beat[i] =-1;

  m_off=-5.0;//no oscillations
  m_on = 1.0;//oscillations
  
  mu = new double*[this->nbparts];
  //mu_on = new double*[this->nbparts];
  for(int i=0; i<this->nbparts; i++)
    {
      mu[i]=new double[this->max_dofs];
      //mu_on[i]=new double[this->max_dofs];
    }

  for(int i=0; i<this->nbparts; i++)
      for(int j=0; j<this->max_dofs; j++)
	{
	  mu[i][j]=m_off;
	}
  
  g = new double*[this->nbparts];
  MU = new double*[this->nbparts];
  G = new double**[this->nbparts];
  for(int i=0; i<this->nbparts; i++)
    {
      G[i] = new double*[this->max_drums];
      for(int j=0; j<this->max_drums; j++)
	{
	  G[i][j]=new double[this->max_dofs];
	}
    }
  for(int i=0; i<this->nbparts; i++)
    {
      g[i]=new double[this->max_dofs];
      MU[i] = new double[this->max_dofs];
    }

}

drum::~drum()
{
  for(int i=0; i<this->nbparts; i++)
    {
      delete mu[i];
      //delete mu_on[i];
      delete g[i];
      delete MU[i];
    }
  delete mu;
  //delete mu_on;
  delete g;
  delete MU;

  for(int i=0; i<this->max_drums; i++)
    {
      for(int j=0; j<this->max_dofs; j++)
	{
	  delete G[i][j];
	}
      delete G[i];
    }
  delete G;

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

  fclose(feedback_file);
}


//**************SENDING PARAMETERS TO THE GENERATOR MODULES*********************************************************//

void drum::sendNewParameterSet(double *mu, double *g, double phase, double freq, int i, BufferedPort<Bottle> *param_port)
{
    Bottle& paramBot = param_port[i].prepare();
    paramBot.clear();  
    for(int j=0;j<controlled_dofs[i];j++)
        {
            paramBot.addDouble(mu[j]);//mu
            paramBot.addDouble(g[j]);//g
        }
    paramBot.addDouble(freq); //freq
    paramBot.addDouble(phase);
    //ACE_OS::printf("Sending (%f,%f) to part %s\n", freq,phase, parts[i].c_str()); 
    param_port[i].write();
}

// GETTING RHYTHM AND PHASE SHIFT FROM THE MANAGER MODULE

int drum::getRhythm(Bottle *Rhythm, int beat)
{  
    //ACE_OS::printf("Frequencies received: "); 
    for (int j=0; j<Rhythm->size(); j++) {
        int indiceScore=(beat+j)%sizeScore;
        //cout << "indice score " << indiceScore;
        rhythmParam[j]= Rhythm->get(j).asDouble();}
        //ACE_OS::printf(" %f", rhythmParam[j]);}
    //ACE_OS::printf("\n");

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
                    ACE_OS::printf("Failed to connect to %s for part %s\n", port_name.c_str(), parts[i].c_str());
                }
        }
}
 

//********** Sending sound feedback****************3
void drum::sendSoundFeedback(Bottle *Hit, BufferedPort<Bottle> *sound_port)
{
    for(int i=0; i<nbparts; i++)
        {
            int note;
            int drumHit=0;

            if(ok[i])
                {
                    note=Hit->get(2).asInt();
                    //                    if (note>0)
                    //                        fprintf(stderr, "%s Received note %d\n", parts[i].c_str(), note);
                    int velocity= Hit->get(3).asInt();
                    if(velocity>0)
                        {
                            for(int k=0; k<nbDrums[i]; k++) 
                                {
                                    int myNote=notes[i][k];
                                    //              fprintf(stderr, "%s Match with %d\n", parts[i].c_str(), myNote);
                                    if(note==myNote)
                                        {
                                            drumHit=1;
                                        } 
                                }                 
                        }               
                }
			    
            Bottle& soundfeed= sound_port[i].prepare();
            soundfeed.clear();  
            if(drumHit==1) 
                {   
                    soundfeed.addInt(1); //FEEDBACK ON
                    //ACE_OS::printf("Sending %s\n", soundfeed.toString().c_str());
                    fprintf(feedback_file, "%f \n", Time::now());
                    drumHit=0;
                    ACE_OS::printf("FEEDBACK ON FOR PART %s\n", parts[i].c_str());
                }
            else
                {
                    soundfeed.addInt(-1);
                    
                }
            sound_port[i].write(true);
        }
}

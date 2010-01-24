#include "drum.h"
#include <iostream>
#include <yarp/os/Os.h>

using namespace std;

                        
double  mu_on[5][4] =   {{1.0,-5.0,-5.0,1.0}, //left arm 
                        {1.0, -5.0, -5.0, 1.0}, //right arm
                        {1.0, -5.0, -5.0, 1.0}, //left leg
                        {1.0, -5.0, -5.0, 1.0}, //right leg
                        {-5.0, -5.0, 1.0, -5.0}}; //head 
drum::drum()
{
    parts[0]="left_arm";
    parts[1]="right_arm";
    parts[2]="left_leg";
    parts[3]="right_leg";
    parts[4]="head";

    soundFeedback = false;
    visualFeedback = false;
  
    for(int i=0; i<nbparts;i++)
    {
        init[i]=0;
    }
  
    feedback_file=fopen("feedback_time.dat","w"); 

    for(int i=0;i<nbparts;i++)
        beat[i] =-1;
    beat_clock=-1;

    m_off=-5.0;//no oscillations
    m_on = 1.0;//oscillations
    freqHead=0.1;
  
    mu = new double*[nbparts];  
    g = new double*[nbparts];
    MU = new double*[nbparts];
    G = new double**[nbparts];
    phase_shift = new double *[nbparts];
    id = new int *[nbparts];;
    notes = new double *[nbparts];
    Score = new int *[nbparts];
    id_pos_found = new bool *[nbparts];
}

drum::~drum()
{
    delete[] frequency;
    
    for(int i=0; i<this->nbparts; i++)
    {
        delete[] mu[i];
        delete[] g[i];
        delete[] MU[i];
        delete[] phase_shift[i];
        delete[] Score[i];    
        delete[] id_pos_found[i];
    }
    
    delete[] mu;
    delete[] g;
    delete[] MU;
    delete[] phase_shift;
    delete[] Score;
    delete[] id_pos_found;
    
    for(int i=0; i<nbDrums[i]; i++)
    {
        for(int j=0; j<controlled_dofs[i]; j++)
        {
            delete[] G[i][j];
        }
        
        delete[] G[i];
        delete[] id[i];
        delete[] notes[i];
    }    
    
    delete[] G;
    delete[] id;
    delete[] notes;
    
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
    scan_port.close();

    fclose(feedback_file);
}


//**************SENDING PARAMETERS TO THE GENERATOR MODULES****************************************//


void drum::sendNewParameterSet(int i)
{
    Bottle& paramBot = param_port[i].prepare();
    paramBot.clear();  

    //ACE_OS::printf("Parameters sent for part %s ", 
                        //parts[i].c_str()); 
                        
    for(int j=0;j<controlled_dofs[i];j++)
    {
        /*ACE_OS::printf("joint %d: (%4.2f, %4.2f) ", 
                            j, 
                            mu[i][j], 
                            g[i][j]);*/
                            
        paramBot.addDouble(mu[i][j]);//mu
        paramBot.addDouble(g[i][j]);//g
    }
    
    /*ACE_OS::printf("freq %4.2f, phase shift %4.2f\n", 
                            frequency[beat[i]],    
                            phase_shift[i][beat[i]]);*/
                            
    if(i==HEAD)
    {
        paramBot.addDouble(freqHead);
    }
    else
    {
        paramBot.addDouble(frequency[beat[i]]); 
    }
    
    paramBot.addDouble(phase_shift[i][beat[i]]);

    param_port[i].write();
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
void drum::sendSoundFeedback()
{
    Bottle *Hit= midi_port.read(false);
    if(Hit!=NULL) 
    {
        for(int i=0; i<nbparts; i++)
        {
            int note;
            int drumHit=0;

            if(ok[i])
            {
                note=Hit->get(2).asInt();
                int velocity= Hit->get(3).asInt();
                if(velocity>0)
                {
                    for(int k=0; k<nbDrums[i]; k++) 
                    {
                        int myNote=notes[i][k];
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
                soundfeed.addInt(1); 
                fprintf(feedback_file, "%4.2f \n", Time::now());
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
    else
    {
        for(int i=0; i<nbparts; i++)
        {
            Bottle& soundfeed2= sound_port[i].prepare();
            soundfeed2.clear();  
            soundfeed2.addInt(-1);  
            sound_port[i].write();
        }
    }
}

//**************SCANNING POSITION FOR THE HEAD**********

void drum::scanHeadPosition()
{
    Bottle *Head=head_port.read();
    if(Head!=NULL)
    {
        Bottle& HeadPos = *(Head->get(0).asList());
        int nb_pos_scanned=HeadPos.size();
        for(int i=0; i<nb_pos_scanned; i++)
        {            
            Bottle& temp= *(HeadPos.get(i).asList());
            int temp_id = temp.get(0).asInt();
            int id_found=0;

            for(int k=0; k<nbDrums[HEAD]; k++)
            {
                if(temp_id==id[HEAD][k])
                {
                    id_pos_found[HEAD][k]=true;
                    if(temp.size()!= controlled_dofs[HEAD]+1)
                    {
                        ACE_OS::printf("ERROR: receiving unconsistent number of angles (%d) for part %s\n",
                                        temp.size()-1, parts[HEAD].c_str());
                        Network::fini();
                        yarp::os::exit(-1);
                    }
                    id_found++;
                    for(int m=0; m<temp.size()-1; m++)
                    {
                        G[HEAD][k][m]=temp.get(m+1).asDouble();
                        ACE_OS::printf("Received new angles for part %s, drum %d, id %d:  ",
                                                        parts[HEAD].c_str(), k, temp_id);
                        ACE_OS::printf("0: %4.2f, 1: %4.2f, 2: %4.2f, 3: %4.2f\n", 
                                        G[HEAD][k][0],G[HEAD][k][1],G[HEAD][k][2],G[HEAD][k][3]);
                    }
                }
            }
            
            if(id_found==0)
            {
                ACE_OS::printf("WARNING: patch id %d could not be linked to any drum\n", temp_id);
                Network::fini();
                yarp::os::exit(-1);
            }
        }
    }   
}


//***************SCANNING POSITION OF THE DRUMS*****************

void drum::scanDrumPosition()
{
    //ACE_OS::printf("SCANNING\n");
    Bottle *Position= scan_port.read(false);
    if(Position!=NULL)
    {
        int nb_scan_drums=Position->size();
        ACE_OS::printf("receiving information for %d ids\n", nb_scan_drums);
        for(int i=0; i<nb_scan_drums; i++)
        {            
            Bottle& temp= *(Position->get(i).asList());
            int temp_id = temp.get(0).asInt();
            int id_found=0;
            ACE_OS::printf("id %d\n", temp_id);
            for(int j=0; j<(nbparts-1); j++) //all parts but the head
             {
                for(int k=0; k<nbIds[j]; k++)
                {
                    if(temp_id==id[j][k])
                    {
                        id_pos_found[j][k]=true;
                        if(temp.size()!= controlled_dofs[j]+1)
                        {
                            ACE_OS::printf("ERROR: receiving unconsistent number of angles (%d) for part %s\n",
                                                    temp.size()-1, parts[j].c_str());
                            Network::fini();
                            yarp::os::exit(-1);
                        }
                        id_found++;
                        
                        for(int m=0; m<temp.size()-1; m++)
                        {
                            G[j][k][m]=temp.get(m+1).asDouble();
                            if(k==Score[i][beat[i]])
                            {
                                g[j][m]=G[j][k][m];
                            }
                        }
                        sendNewParameterSet(j);
                        
                        ACE_OS::printf("Received new angles for part %s, drum %d, id %d:  ",
                                                        parts[j].c_str(), k, temp_id);
                        ACE_OS::printf("0: %4.2f, 1: %4.2f, 2: %4.2f, 3: %4.2f\n", 
                                                        G[j][k][0],G[j][k][1],G[j][k][2],G[j][k][3]);
                    }
                }
            }
            
            if(id_found==0)
            {
                ACE_OS::printf("WARNING: patch id %d could not be linked to any drum\n", temp_id);
                Network::fini();
                yarp::os::exit(-1);
            }
            
            if(id_found>1)
            {
                ACE_OS::printf("WARNING: patch id %d was linked to more than one drum\n", temp_id);
                Network::fini();
                yarp::os::exit(-1);
            }                    
        }
    }
}

void drum::getConfig()
{
    for(int i=0; i<nbparts; i++)
    { 
        if(ok[i])
        {
            //getting config info 
            bool confFile;
            char partName[255];
            sprintf(partName, "%s/%sConfig.ini", pathToConfig, parts[i].c_str());
            
            Property partConf;
            confFile=partConf.fromConfigFile(partName);
           
            if(!confFile)
            {
                ACE_OS::printf("Config file \"%s\" not found for part %s\n", 
                                partName, parts[i].c_str());
                Network::fini();
                yarp::os::exit(-1);
            }
           
            controlled_dofs[i] = partConf.find("nbDOFs").asInt();
            
            mu[i]=new double[controlled_dofs[i]];
            g[i]=new double[controlled_dofs[i]];
            MU[i] = new double[controlled_dofs[i]];
            for(int j=0; j<controlled_dofs[i]; j++)
            {
                mu[i][j]=m_off;
            }
                        
            ACE_OS::printf( "\nPart %s (%d dofs)\n", 
                            parts[i].c_str(),controlled_dofs[i]);


            //getting target info            
            bool file;
            char drumName[255], temp[255];
            
            sprintf(temp, "%s/%sTargets.ini", 
                    pathToConfig, parts[i].c_str());
            
            Property drum;
            file=drum.fromConfigFile(temp);
            
            if(!file)
            {
                ACE_OS::printf("Target positions file \"%s\" not found for part %s\n", 
                                temp, parts[i].c_str());
                Network::fini();
                yarp::os::exit(-1);
            }
            
            nbDrums[i] = drum.find("NbDrums").asInt();
            ACE_OS::printf("nb drums %d\n",nbDrums[i]);  
            
            G[i] = new double*[nbDrums[i]+1];
            
            id[i] = new int[nbDrums[i]];
            notes[i] = new double[nbDrums[i]];
            
            for(int j=0; j<nbDrums[i]+1; j++)
            {
                G[i][j]= new double[controlled_dofs[i]];
            }
            
            Bottle& Target2 = drum.findGroup("Idle");
            for (int k=0; k<Target2.size()-1; k++) 
            {
                G[i][0][k]=3.14/180.0*Target2.get(k+1).asDouble();
                ACE_OS::printf("%4.2f ", G[i][0][k]);
            }                    
            printf("\n");
            Target2.clear();
                                                                    
            for(int j=1; j<(nbDrums[i]+1); j++)
            {
                sprintf(drumName, "Drum_%d", j);			  
                Bottle& Target= drum.findGroup(drumName);
                 
                for (int k=0; k<Target.size()-1; k++) 
                {
                    G[i][j][k]=3.14/180.0*Target.get(k+1).asDouble();
                    ACE_OS::printf("%4.2f ", G[i][j][k]);
                }
                
                ACE_OS::printf("\n");
                Target.clear();
            }   
                        
            cout << "getting notes... " << endl; 
            ACE_OS::printf("Notes: ");
            Bottle& Target3 = drum.findGroup("Notes");
            for (int k=0; k<Target3.size()-1; k++) 
            {
                printf("%d ", Target3.get(k+1).asInt());
                notes[i][k]=Target3.get(k+1).asInt();
            }
            printf("\n");
            Target3.clear();
            
//            cout << "getting ids... " << endl; 
//            ACE_OS::printf("Ids: ");
//            Bottle& Target4 = drum.findGroup("Ids");
//            nbIds[i]=Target4.size()-1; 
//            id_pos_found[i]=new bool[nbIds[i]];
//            for (int k=0; k<Target4.size()-1; k++) 
//            {               
//                id[i][k]=Target4.get(k+1).asInt();
//                printf("%d ", id[i][k]);

//                if(i==HEAD)
//                {
//                    if(id[HEAD][k]<0)
//                    {
//                        scan = k+1; 
//                        ACE_OS::printf(" SCAN = %d", scan);
//                    }
//                }
//            }
//            printf("\n");
//            Target4.clear();            
        }
    }	
}



//***********OPENING PORTS AND CONNECTING********************

void drum::doConnect()
{
    for(int i=0; i<nbparts; i++)
    {
        ACE_OS::printf("ports for part %s...", parts[i].c_str());
        openPort(i,param_port,"parameters",1,1);// OUT sends parameters to the generator
  
        if(ok[i])
        {
            ACE_OS::printf("Using part %s\n", parts[i].c_str());

            //opening ports and connecting with the DrumGenerator modules
            openPort(i,check_port, "check_motion",0,1);  // IN receives beat 
            openPort(i,sound_port, "sound", 1,1); // OUT sends soundfeedback info to the generator 

            //opening ports for the connection with the guiDemo
            openPort(i,score_port, "score", 0,0); //IN receives score
            openPort(i,phase_port, "phase", 0,0); //IN receives phase
        }
   
        else
        {
            param_port[i].close();
            ACE_OS::printf("Not using %s\n", parts[i].c_str());
        }
    }

    //connecting with the clock
    clock_port.open("/clock/parameters/out");
    bool okClock=Network::connect("/clock/parameters/out","/clock/parameters/in", "tcp");
    if(!okClock)ACE_OS::printf("troubles connecting with the clock\n");
    beat_clock_port.open("/clock/check_motion/in");
    bool okClock2=Network::connect("/clock/check_motion/out","/clock/check_motion/in", "tcp");
    if(!okClock2)ACE_OS::printf("troubles connecting with the clock\n");

    //opening port to get frequency and couplings from the DrumManager
    interactive_port.open("/interactive/in");

    bool check=scan_port.open("/scan/in");
    if(!check)
    {
        ACE_OS::printf("fail to open scan port\n");
    }
    //head_port.open("/headPort/in");

    ACE_OS::printf("connecting ports for ikin");
    fflush(stdout);
    
    bool scan=Network::connect("/drumsarah/out","/scan/in");
    //head = Network::connect("/DrumHeadControl/out","/headPort/in", "udp");
    
    ACE_OS::printf("... done\n");
    fflush(stdout);

    //if(head && scan)
    if(scan)
    {
        ACE_OS::printf( "Visual feedback enabled\n");
        visualFeedback=true;
    }
    else
    {
       ACE_OS::printf(  "No visual feedback \n");
       //(scan %d, head %d)\n"(int) scan, (int) head ); 
    }
    
    //visualFeedback=true;
 
}

bool drum::getRhythm()
{  
    int size;
        
    Bottle *Rhythm = interactive_port.read(false);
    if(Rhythm!=NULL) 
    {
        int size = Rhythm->size();
        frequency = new double [size];
        if(beat_clock==-1) //first time we receive rhythm, get drum beat clock
        {            
            Bottle *time_init=beat_clock_port.read();
            beat_clock=0;
            if(time_init!=NULL) 
            {
                drum_beat_clock = time_init->get(0).asInt();
            }
        }            
        else //beat = current beat - drum beat clock
        {
            Bottle *beat_c= beat_clock_port.read(false);
            if(beat_c!=NULL) 
            {
                beat_clock = beat_c->get(0).asInt()-drum_beat_clock;
            }	  
        }

        ACE_OS::printf("Receiving frequency: ");
        
        for (int j=0; j<size; j++) 
        {
            int indiceScore=(beat_clock+j)%size;
            frequency[indiceScore]= Rhythm->get(j).asDouble();
            ACE_OS::printf("%4.2f ", frequency[indiceScore]);
        }
        ACE_OS::printf("\n");

        if(frequency[beat_clock]<0.0)
        {
            ACE_OS::printf("Closing command received from the gui...\n");
            return 0; 
        }    
    }
            
    return 1;
}

void drum::getScore() //get score parameters from the gui
{    
    for(int i=0; i<nbparts; i++)
    {
        if(ok[i])
        {
            Bottle *newScore = score_port[i].read(false); 
            if(newScore!=NULL) 
            {				                
                if(beat[i]==-1)//getting beat of the generator
                {
                    ACE_OS::printf("Initial score received\n");
                    sizeScore=newScore->size();
        
                    Bottle *time_init=check_port[i].read();

                    beat[i]=0;
                    if(time_init!=NULL)
                    {
                        drum_beat[i] = time_init->get(0).asInt();
                    }
                    
                    phase_shift[i]= new double [sizeScore];
                    Score[i]= new int [sizeScore];
                }
                
                ACE_OS::printf("Score for part %s: ",parts[i].c_str());
                    
                for (int j=0; j<sizeScore; j++) 
                {
                    int indiceScore = (j+beat[i])%sizeScore;
                    Score[i][indiceScore]= newScore->get(j).asInt();
                    ACE_OS::printf("%d ",Score[i][indiceScore], newScore->get(j).asInt());
                }
			      	
                ACE_OS::printf("\n");
            }
	     
            if(beat[i]!=-1)
            {
                Bottle *newPhase = phase_port[i].read(false); 
                if(newPhase!=NULL) 
                {
                    ACE_OS::printf("receiving new information on the phase\n");
                    for(int j=0; j<sizeScore; j++) 
                    {
                        int indiceScore = (j+beat[i])%sizeScore;
                        phase_shift[i][indiceScore]= newPhase->get(j).asDouble();
                    }
            
                    ACE_OS::printf("Phase shifts for part %s: ",parts[i].c_str());
                    for (int j=0; j<sizeScore; j++) 
                    {
                        ACE_OS::printf("%4.2f ", phase_shift[i][j]);
                    } 					                    
                    ACE_OS::printf("... done!\n");
                }
            }
        }
    } 
    
}

void drum::sendScore()
{       
    for(int i=0; i<nbparts; i++)
    {	   
        if(ok[i] && beat[i]!=-1) //if part is active and parameters have already been received
        {
            Bottle *answer = check_port[i].read(false);
            if(answer!=NULL)
            {
                current_beat[i] = answer->get(0).asInt();
                beat[i]=(current_beat[i]-drum_beat[i])%sizeScore;
                ACE_OS::printf("Current beat for part %s is %d\n",
                                parts[i].c_str(),
                                beat[i]);
                    
                for(int k=0; k<controlled_dofs[i]; k++)
                {
                    g[i][k]= G[i][Score[i][beat[i]]][k];
                    
                    if(i!=HEAD) //limbs but head
                    {   
                        if(Score[i][beat[i]]>0) //BEATING
                        {
                            mu[i][k] = mu_on[i][k];
                            init[i] = 1;
                        }
                        else //HOLD ON; PART NOT ACTIVE
                        {
                            mu[i][k] = m_off;
                            init[i]=0;
                        }
                    }
                    else  //head oscillates only during scan
                    {
                        if(Score[i][beat[i]]!=scan)
                        {
                            mu[i][k] = m_off;
                            init[i]=1;
                        }
                        else 
                        {
                            mu[i][k] = mu_on[i][k];
                            init[i]=1;
                        }
                    }
                }
                sendNewParameterSet(i);

            }
                
        }
    }
    
    //sending frequency to the clock	
    if(beat_clock!=-1)
    {
        Bottle *be = beat_clock_port.read(false);
        if(be!=NULL)
        {
            int current_clock= be->get(0).asInt();
            beat_clock=(current_clock-drum_beat_clock)%sizeScore;		      
            ACE_OS::printf("beat %d for clock\n", beat_clock);
            
            Bottle& HeadBot = clock_port.prepare();
            HeadBot.clear();  
            HeadBot.addDouble(frequency[beat_clock]);
            clock_port.write();
        }
    } 		  
}


void drum::getPosition()
{    
    Bottle& paramBot = param_port[HEAD].prepare();
    paramBot.clear();  

    ACE_OS::printf("\nSCANNING: Parameters sent to part HEAD (SCAN=%d)\n", scan);
                                                
    for(int j=0;j<controlled_dofs[HEAD];j++)
    {
        ACE_OS::printf("joint %d: (%4.2f, %4.2f) ", 
                            j, mu_on[HEAD][j], G[HEAD][scan][j]);
        paramBot.addDouble(mu_on[HEAD][j]);//mu
        paramBot.addDouble(G[HEAD][scan][j]);//g
    }
        
    paramBot.addDouble(freqHead); 
    paramBot.addDouble(0.0);

    param_port[HEAD].write();
    
    ACE_OS::printf("... \n");
    

    //while(true)
    //{
        //ACE_OS::printf("... ");
        
        //scanDrumPosition();
        //scanHeadPosition();

        //int check[3]={0,0,0};
        
        //for(int j=0; j<nbparts-1; j+1) //all parts but the head
        //{
            //for(int i=0; i<nbIds[j]; i++)
            //{
                //if(id_pos_found[j][i])
                //{
                    //check[j]++;
                //}
            //}
        //}
        
        //for(int i=0; i<nbIds[HEAD]; i++)
        //{
            //if(id_pos_found[HEAD][i])
            //{
                //check[2]++;
            //}
        //}
        
        //if(check[LEFT_ARM]==nbDrums[LEFT_ARM] && check[RIGHT_ARM]==nbDrums[RIGHT_ARM] && check[2]==nbDrums[HEAD])
        //{
            //ACE_OS::printf(" done\n");
            //break;
        //}
        
    //}
}

void drum::run()
{ 
    bool keep = true;
    
    
    //if(visualFeedback)
    //{
        //getPosition();
    //}
    
    bool init_score = false;
        
    while(true)
    {
        //*******get position of the drums************
        if(visualFeedback)
        {
            scanDrumPosition();
            //scanHeadPosition();
        }  
        
        //***** gets frequency and sends it to the generators
		keep=getRhythm(); 
                                    
        //******gets scores and phase_shifts from the gui*****************
        getScore();
              
        //******sends scores to the generators*********
        sendScore();
        
        if(!keep)
        {
            break;
        }
        
        Time::delay(0.025);
            
    }
}



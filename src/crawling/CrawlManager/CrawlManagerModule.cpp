// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CrawlManagerModule.h"
#include <ace/OS.h>

#define DEBUG 1

char ESC=27;

bool CrawlManagerModule::updateModule()
{
    if(com==9)
        return false;

    cout << "Please specify the task" << endl;
    cout << "(1) Go to init pos" << endl;
    cout << "(2) Crawl" << endl;
    cout << "(3) Faster" << endl;
    cout << "(4) Slower" << endl;
    cout << "(5) Turn right" << endl;
    cout << "(6) Turn left" << endl;
    //cout << "(7) Sit" << endl;
    //cout << "(8) On all fours" << endl; 
    //cout <<"9 Stop the module" << endl;
    
    return true;
}

bool CrawlManagerModule::respond(const Bottle &command, Bottle &reply)
{
  com = command.get(0).asInt();
  cout << "soos " << com << endl;
  
   switch(com)
    {   
        case 1:
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, init_parameters);
            reply.addString("going to init pos");
            break;
            
        case 2:
            crawl_parameters[9][1]=crawl_init_parameters[9][1]; //reset point torso roll
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("crawling");
            break;
                
        case 3:
            om_stance+=0.05;
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("crawling faster");
            break;
            
        case 4:
            om_stance-=0.05;
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("crawling slower");
            break;

            
        case 5: 
        
            turnAngle-=0.1; //this could be turn into an incremental def of the turning angle
            crawl_parameters[9][1]=turnAngle; 
            turnParams = myIK->getTurnParams(crawl_parameters, scaleLeg);
            
            printf("turn Params delta: %f R: %f\n", turnParams[0], turnParams[1]); 
            
            crawl_parameters[0][0]= scaleLeg*(turnParams[1]-myIK->dShoulder); //left shoulder pitch amplitude
            crawl_parameters[2][0]= scaleLeg*(turnParams[1]+myIK->dShoulder); // right shoulder pitch amplitude
            crawl_parameters[4][0]= scaleLeg*(turnParams[1]-myIK->dHip); //left hip pitch amplitude
            crawl_parameters[6][0]= scaleLeg*(turnParams[1]+myIK->dHip); //right hip pitch amplitude
            
            //crawl_parameters[1][2] = crawl_init_parameters[1][2]+turnParams[0]; //left shoulder roll target
            //crawl_parameters[3][2] = crawl_init_parameters[3][2]-turnParams[0]; //right shoulder roll target
            

            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("turning right");
            break;
            
        case 6:
        
            turnAngle+=0.1; //this could be turn into an incremental def of the turning angle
            crawl_parameters[9][1]=turnAngle; 
            turnParams = myIK->getTurnParams(crawl_parameters, scaleLeg);
            
            printf("turn Params angle: %f delta: %f R: %f\n", turnAngle, turnParams[0], turnParams[1]); 
            
            crawl_parameters[0][0]= scaleLeg*(turnParams[1]+myIK->dShoulder); //left shoulder pitch amplitude
            crawl_parameters[2][0]= scaleLeg*(turnParams[1]-myIK->dShoulder); // right shoulder pitch amplitude
            crawl_parameters[4][0]= scaleLeg*(turnParams[1]+myIK->dHip); //left hip pitch amplitude
            crawl_parameters[6][0]= scaleLeg*(turnParams[1]-myIK->dHip); //right hip pitch amplitude
            
            //crawl_parameters[1][2] = crawl_init_parameters[1][2]-turnParams[0]; //left shoulder roll target
            //crawl_parameters[3][2] = crawl_init_parameters[3][2]+turnParams[0]; //right shoulder roll target
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("turning left");
            break;
            
        //case 7:
            //reply.addString("sitting");            
            //for(int k=0;k<nbPosSit;k++)
            //{
                //vector<vector<double> > temp_param;
                //vector<double> temp_ampl, temp_target;
                
                //for(int i=0;i<nbParts;i++)
                //{                    
                    //for(int j=0; j<nbDOFs[i]; j++)
                    //{
                     //temp_ampl.push_back(sit_parameters[i][2*j][k]);
                     //temp_target.push_back(sit_parameters[i][2*j+1][k]);
                    //}
                    //temp_param.push_back(temp_ampl);
                    //temp_param.push_back(temp_target);
                                             
                    //if(connected_part[i]) sendCommand(i, temp_param);  
                    
                    //temp_ampl.clear();
                    //temp_target.clear(); 
                //}         
                //Time::delay(1.5);
            //}
            //break;
            
        //case 8:
            //reply.addString("going on all fours TEST");         
            //for(int k=0;k<nbPosUnsit;k++)
            //{
                //vector<vector<double> > temp_param2;
                //vector<double> temp_ampl2, temp_target2;
                //for(int i=0;i<nbParts;i++)
                //{                    
                    //for(int j=0; j<nbDOFs[i]; j++)
                    //{
                        //int l=2*j+1;
                     //temp_ampl2.push_back(unsit_parameters[i][2*j][k]);
                     //temp_target2.push_back(unsit_parameters[i][2*j+1][k]);
                    //}
                    //temp_param2.push_back(temp_ampl2);
                    //temp_param2.push_back(temp_target2);
      
                    //if(connected_part[i]) sendCommand(i, temp_param2);

                    //temp_ampl2.clear();
                    //temp_target2.clear(); 
                //}         
                //Time::delay(5.0);
            //}
            //break;
            
        //case 9:
            //reply.addString("stopping");
            ////om_stance=-1;
            ////for(int i=0;i<nbParts;i++)
                ////if(connected_part[i]) sendCommand(i, crawl_parameters);
            //interruptModule();
            //close();
            //break;
            
        default:
            break;
            
        }
        
    return true;
       
}    
      
bool CrawlManagerModule::open(Searchable &s)
{
    Property arguments(s.toString());
    Property options;
	
    if(arguments.check("file"))
	{
		options.fromConfigFile(arguments.find("file").asString().c_str());
	}
	else
	{
		options.fromConfigFile("../../Crawling/config/managerConfig.ini");
	}

	cout << "Config : " << options.toString() <<endl;
    Time::turboBoost();

    part_names[0]="left_arm";
    part_names[1]="right_arm";
    part_names[2]="left_leg";
    part_names[3]="right_leg";
    part_names[4]="torso";
    part_names[5]="head";
    
    vector<double> crawl_ampl;
    vector<double> crawl_target;
    vector<double> init_ampl;
    vector<double> init_target;
    
    left_turn=1;
    right_turn=1;
    
    nbPosSit=0;
    nbPosUnsit=0;
    
    turnParams.push_back(0); //delta angle shoulder yaw for turning
    turnParams.push_back(1); //radius of rotation
     
    closedLoop.push_back(-1); //distance left arm right leg x
    closedLoop.push_back(-1); //distance left arm right leg y
    closedLoop.push_back(-1); //distance left arm right leg z
    closedLoop.push_back(-1); //distance right arm left leg x
    closedLoop.push_back(-1); //distance right arm left leg y
    closedLoop.push_back(-1); //distance right arm left leg z

    for(int i=0;i<nbParts;i++)
        {
            scale[i]=0.1; //setting scaling parameters     
                        
            char tmp1[255],tmp2[255];
            sprintf(tmp1,"/%s/parameters/in",part_names[i].c_str());

            /////////////////////////////////////////////////////
            ///check if the part is active and connect///////////
            /////////////////////////////////////////////////////
            Contact query = Network::queryName(tmp1);
            if(query.isValid())
                {
                    cout << "found part " << tmp1 << " ... connecting" << endl;
                    sprintf(tmp2,"/manager/%s/out",part_names[i].c_str());
                    bool ok = parts_port[i].open(tmp2);
                    if(!ok)
                        {
                            ACE_OS::printf("unable to open %s\n",tmp2);
                            return false;
                        }
                    ok = Network::connect(tmp2,tmp1);
                    if(!ok)
                        {
                            ACE_OS::printf("unable to connect %s with %s\n",tmp2,tmp1);
                            return false;
                        }
                    connected_part[i] = true;
                }
            else
				{
                	connected_part[i] = false;
                	printf("%c[1m",ESC); 
					ACE_OS::printf("***ERROR*** PART %s NOT DETECTED \n", part_names[i].c_str());
					printf("%c[0m",ESC); 
				}
				  
                
            ///////we get
            if(options.check(part_names[i].c_str()))
                {
                    Bottle& bot = options.findGroup(part_names[i].c_str());
                    
                    Bottle& bot1=bot.findGroup("nbJoints");
                    nbDOFs[i] =bot1.get(1).asInt();
                    ACE_OS::printf("nbJoints %d\n", nbDOFs[i]);
                    
                    
                    Bottle& bot2 = bot.findGroup("mu_crawl");
                    Bottle& bot3 = bot.findGroup("setPoints_crawl");
                    if(bot2.isNull() || bot3.isNull() || bot2.size()<nbDOFs[i]+1 || bot3.size()<nbDOFs[i]+1)
                        {
                            ACE_OS::printf("please specify crawl config for %s\n",part_names[i].c_str());
                            return false;
                        }
                        
                    for(int j=0;j<nbDOFs[i];j++)
                        {
                            crawl_ampl.push_back(bot2.get(j+1).asDouble());
                            crawl_target.push_back(bot3.get(j+1).asDouble());
                        }

                    crawl_parameters.push_back(crawl_ampl);
                    crawl_parameters.push_back(crawl_target);

                    crawl_init_parameters.push_back(crawl_ampl);
                    crawl_init_parameters.push_back(crawl_target);

                    Bottle& bot4 = bot.findGroup("mu_init");
                    Bottle& bot5 = bot.findGroup("setPoints_init");
                    if(bot4.isNull() || bot5.isNull() || bot4.size()<nbDOFs[i]+1 || bot5.size()<nbDOFs[i]+1)
                        {
                            ACE_OS::printf("please specify init config for %s\n",part_names[i].c_str());
                            return false;
                        }
                    for(int j=0;j<nbDOFs[i];j++)
                        {
                            init_ampl.push_back(bot4.get(j+1).asDouble());
                            init_target.push_back(bot5.get(j+1).asDouble());
                        }
                    init_parameters.push_back(init_ampl); 
                    init_parameters.push_back(init_target); 
                    
                    crawl_ampl.clear();
                    crawl_target.clear();
                    init_ampl.clear();
                    init_target.clear();  
                
                    ACE_OS::printf("crawl parameters for part %s\n", part_names[i].c_str());
                    printf("amplitudes ( ");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_parameters[2*i][j]);
                    printf(")\ntargets (");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_parameters[2*i+1][j]);
                    printf(")\n", om_stance, om_swing);  
                    
                    ACE_OS::printf("init parameters for part %s\n", part_names[i].c_str());
                    printf("amplitudes ( ");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", init_parameters[2*i][j]);
                    printf(")\ntargets (");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", init_parameters[2*i+1][j]);
                    printf(")\n\n", om_stance, om_swing);     
                }
            else
                {
                    ACE_OS::printf("please specify config for %s\n",part_names[i].c_str());
                    return false;
                }
        }
    
                        
    ////we get the frequencies
    if(options.check("omStance"))
        {
			Bottle& tmpBot = options.findGroup("omStance");
			if(tmpBot.size()==2)
	  		{
	    		om_stance = tmpBot.get(1).asDouble();
	  		}
			else
	  		{
	    	ACE_OS::printf("Please specify omStance for manager\n");
	    	return false;
	  		}
      	}
    else
      {
	    ACE_OS::printf("Please specify omStance for manager\n");
	    return false;
      }
    
    if(options.check("omSwing"))
      {
	    Bottle& tmpBot = options.findGroup("omSwing");
	    if(tmpBot.size()==2)
	    {
	        om_swing = tmpBot.get(1).asDouble();
	    }
	    else
	    {
	        ACE_OS::printf("Please specify omSwing for manager\n");
	        return false;
	    }
      }
    else
      {
	    ACE_OS::printf("Please specify omSwing for manager\n");
	    return false;
      }
      
    if(options.check("amplitude"))
      {
	    Bottle& tmpBot = options.findGroup("amplitude");
	    if(tmpBot.size()==2)
	    {
	        scaleLeg = tmpBot.get(1).asDouble();
	        for(int i=0; i<nbParts; i++)
	        {
	            scale[i]=scaleLeg;
	        }
	    }
	    else
	    {
	        ACE_OS::printf("Please specify amplitude for legs\n");
	        return false;
	    }
      }
    else
      {
	    ACE_OS::printf("Please specify amplitude for legs for manager\n");
	    return false;
      }

    //getSequence(sit_parameters, "Sit", nbPosSit);
    //getSequence(unsit_parameters, "Unsit", nbPosUnsit);
    
    myIK = new IKManager;

    return true;

}


bool CrawlManagerModule::getSequence(vector<vector<vector<double> > > parameters, ConstString task, int& nbPos)
{
    ACE_OS::printf("\nSequence for %s...\n", task.c_str());
    vector<vector<vector<double> > > vec;
    
    char partName[255];
    for(int	i=0;i<nbParts;i++)
    {
        sprintf(partName,"%s_%s", task.c_str(), part_names[i].c_str());
        if (opt.check(partName))
        {
            ACE_OS::printf("\n%s\n", part_names[i].c_str());
            Bottle &botPart = opt.findGroup(partName);
            
            char strIndex[2]; // 1 to 99 parts.

            vector<vector<double> > joints;
            
            for(int j=0; j<nbDOFs[i]; j++)
            { 
                sprintf(strIndex,"%d",j);
                Bottle &botJoint = botPart.findGroup(strIndex);
                if(j==0) nbPos=botJoint.size()-2;

                vector<double> positions;
                vector<double> amplitudes;
                
                if(botJoint.isNull())
                {
                    ACE_OS::printf("Please specify the positions for %s \n", task.c_str());
                    return false;
                }

                for(int k=0; k<nbPos; k++)
                {
                    amplitudes.push_back(-1); //no oscillations
                    positions.push_back(botJoint.get(k+2).asDouble());
                    ACE_OS::printf("%f ", positions[k]);
                }
                
                ACE_OS::printf("\n");
                
                joints.push_back(amplitudes);
                joints.push_back(positions);                                
            }
        
            vec.push_back(joints); 
        }  
        
        else 
        {
            ACE_OS::printf("WARNING: No sequence found for task %s\n\n", task.c_str());
            return false;
        }
    }    
    parameters.swap(vec);  
    
    return true;
}

void CrawlManagerModule::sendCommand(int i, vector<vector<double> > params)
{
    Bottle& paramBot = parts_port[i].prepare();
    paramBot.clear();
    for(int j=0;j<nbDOFs[i];j++)
    {
        paramBot.addDouble(params[2*i][j]); 
        paramBot.addDouble(params[2*i+1][j]);                               
    }
    paramBot.addDouble(om_stance);
    paramBot.addDouble(om_swing);
    parts_port[i].write();
                            
    #if DEBUG
    printf("SENDING COMMAND TO PART %s:\n", part_names[i].c_str());
    printf("amplitudes ( ");
    for(int j=0; j<nbDOFs[i]; j++) 
        printf("%f ", params[2*i][j]);

    printf(")\ntargets (");
    for(int j=0; j<nbDOFs[i]; j++) 
        printf("%f ", params[2*i+1][j]);
        
    printf(")\n om_stance %f, om_swing %f\n\n", om_stance, om_swing);  
    #endif                         
}

double CrawlManagerModule::getPeriod()
{
    return 2.0;
}
 
bool CrawlManagerModule::close()
{
    fprintf(stderr, "crawl manager module closing...");
    return true;
}

bool CrawlManagerModule::interruptModule()
{
    for(int i=0;i<6;i++){
        if(connected_part[i]==true){
            parts_port[i].interrupt();}}
    fprintf(stderr, "interrupting module...");
    return true;
}



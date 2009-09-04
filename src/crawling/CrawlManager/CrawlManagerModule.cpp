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
    //cout <<"(9) Stop the module" << endl;
    
    return true;
}

bool CrawlManagerModule::respond(const Bottle &command, Bottle &reply)
{
  com = command.get(0).asInt();
  cout << "soos " << com << endl;
  
   switch(com)
    {   
        case 1:
            
            if(STATE==CRAWL)
            {
                int side=0;
                while(side==0) side=getSwingingArm();
                
                if(side==LEFT_ARM)
                {
                    for(int i=0;i<nbParts;i++)
                        if(connected_part[i]) sendCommand(i, crawl_left_parameters);
                    Time::delay(1.0);
                }
                
                if(side==RIGHT_ARM)
                {
                    for(int i=0;i<nbParts;i++)
                        if(connected_part[i]) sendCommand(i, crawl_right_parameters);
                    Time::delay(1.0);
                }
            }
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, init_parameters);
            
            reply.addString("going to init pos");
            STATE = INIT_POS;
            
            break;
            
        case 2:                

            //if(STATE!=CRAWL)
            //{
                //for(int i=0;i<nbParts;i++)
                    //if(connected_part[i]) sendCommand(i, crawl_right_parameters);
                //Time::delay(1.0);
            //}
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            
            reply.addString("crawling");
            STATE = CRAWL;
            
            break;
                
        case 3:
            
            if(STATE!=CRAWL)
            {
                for(int i=0;i<nbParts;i++)
                    if(connected_part[i]) sendCommand(i, crawl_right_parameters);
                Time::delay(0.2);
            }
            
            om_stance+=0.05;
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            
            reply.addString("crawling faster");
            STATE = CRAWL;
            
            break;
            
        case 4:
            
            if(STATE!=CRAWL)
            {
                for(int i=0;i<nbParts;i++)
                    if(connected_part[i]) sendCommand(i, crawl_right_parameters);
                Time::delay(0.2);
            }
            om_stance-=0.05;
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            
            reply.addString("crawling slower");
            STATE = CRAWL;
            
            break;
            
        case 5: 
            
            turnAngle-=0.1; 
            crawl_parameters[9][1]=turnAngle; 

            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            
            reply.addString("turning right");
            
            break;
            
        case 6:
        
            turnAngle+=0.1;
            crawl_parameters[9][1]=turnAngle; 
            
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_parameters);
            reply.addString("turning left");
            
            break;
                        
        default:
            break;
            
        }
        
    return true;
       
}    
      
bool CrawlManagerModule::open(Searchable &s)
{
    Property arguments(s.toString());
    
    if(arguments.check("file"))
	{
		options.fromConfigFile(arguments.find("file").asString().c_str());
	}
	else
	{
		options.fromConfigFile("../../Crawling/config/managerConfig.ini");
	}
    
    options.toString();
    
    Time::turboBoost();

    part_names[0]="left_arm";
    part_names[1]="right_arm";
    part_names[2]="left_leg";
    part_names[3]="right_leg";
    part_names[4]="torso";
    part_names[5]="head";
    
    STATE = NOT_SET;
    
    vector<double> crawl_ampl;
    vector<double> crawl_target;
    vector<double> init_ampl;
    vector<double> init_target;

    turnAngle=0;
    nbPosSit=0;
    nbPosUnsit=0;

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
                    
                    sprintf(tmp1, "/manager/%s/status/in", part_names[i].c_str());
                    sprintf(tmp2, "/%s/status_for_manager/out", part_names[i].c_str());
                    ok = check_port[i].open(tmp1);
                    if(!ok)
                    {
                        ACE_OS::printf("unable to open %s\n",tmp1);
                        return false;
                    }
                    ok = Network::connect(tmp2,tmp1);
                    if(!ok)
                    { 
                        ACE_OS::printf("unable to connect %s with %s\n",tmp2,tmp1);
                        return false;
                    }
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
                    
                    crawl_ampl.clear();
                    crawl_target.clear();
                    
                    Bottle& bot6 = bot.findGroup("mu_left_crawl");
                    Bottle& bot7 = bot.findGroup("setPoints_left_crawl");
                    if(bot6.isNull() || bot7.isNull() || bot6.size()<nbDOFs[i]+1 || bot7.size()<nbDOFs[i]+1)
                    {
                        ACE_OS::printf("please specify init crawl config for %s\n",part_names[i].c_str());
                        return false;
                    }
                        
                    for(int j=0;j<nbDOFs[i];j++)
                    {
                        crawl_ampl.push_back(bot6.get(j+1).asDouble());
                        crawl_target.push_back(bot7.get(j+1).asDouble());
                    }

                    crawl_left_parameters.push_back(crawl_ampl);
                    crawl_left_parameters.push_back(crawl_target);
                    
                    crawl_ampl.clear();
                    crawl_target.clear();
                    
                    Bottle& bot8 = bot.findGroup("mu_right_crawl");
                    Bottle& bot9 = bot.findGroup("setPoints_right_crawl");
                    if(bot8.isNull() || bot9.isNull() || bot8.size()<nbDOFs[i]+1 || bot9.size()<nbDOFs[i]+1)
                    {
                        ACE_OS::printf("please specify init crawl config for %s\n",part_names[i].c_str());
                        return false;
                    }
                        
                    for(int j=0;j<nbDOFs[i];j++)
                    {
                        crawl_ampl.push_back(bot8.get(j+1).asDouble());
                        crawl_target.push_back(bot9.get(j+1).asDouble());
                    }

                    crawl_right_parameters.push_back(crawl_ampl);
                    crawl_right_parameters.push_back(crawl_target);
                                                            
                    crawl_ampl.clear();
                    crawl_target.clear();

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
                    
                    ACE_OS::printf("crawl left parameters for part %s\n", part_names[i].c_str());
                    printf("amplitudes ( ");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_left_parameters[2*i][j]);
                    printf(")\ntargets (");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_left_parameters[2*i+1][j]);
                    printf(")\n", om_stance, om_swing);   
                    
                    ACE_OS::printf("crawl right parameters for part %s\n", part_names[i].c_str());
                    printf("amplitudes ( ");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_right_parameters[2*i][j]);
                    printf(")\ntargets (");
                    for(int j=0; j<nbDOFs[i]; j++) 
                        printf("%f ", crawl_right_parameters[2*i+1][j]);
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
        if (options.check(partName))
        {
            ACE_OS::printf("\n%s\n", part_names[i].c_str());
            Bottle &botPart = options.findGroup(partName);
            
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

int CrawlManagerModule::getSwingingArm()
{    
    int side=0;
    double y_speed[4];
    bool swing[4]={false,false,false,false};
    
    for(int i=0; i<4; i++)
    {
        Bottle *command = check_port[i].read(false);
        if(command!=NULL)
        {
            ACE_OS::printf("getting info from the generators\n");
            y_speed[i] = command->get(0).asDouble();
            y_speed[i]>0.0 ? swing[i] = false : swing[i] = true;
        }
        else ACE_OS::printf("no info\n");
        //Time::delay(0.1);
    }
    
    ACE_OS::printf("y_speed %f %f %f %f\n", 
                            y_speed[0], y_speed[1], y_speed[2], y_speed[3]);
    
    if(swing[0] || swing[3] || !swing[1] ||!swing[2]) side = LEFT_ARM;
    if(!swing[0] || !swing[3] || swing[1] ||swing[2]) side = RIGHT_ARM;

    return side;
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
    paramBot.addDouble(turnAngle);
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
    for(int i=0;i<6;i++)
        if(connected_part[i]==true) parts_port[i].close();
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



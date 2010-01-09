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
    cout <<"(9) EMERGENCY STOP" << endl;
    
    return true;
}

///This function switches between behaviors: 

bool CrawlManagerModule::respond(const Bottle &command, Bottle &reply)
{
  com = command.get(0).asInt();
  cout << "soos " << com << endl;
  
   switch(com)
    {   
        case 1: /// - on all fours
        
			InitPosition();
            reply.addString("going to init pos");
            
            break;
            
        case 2:  ///- crawl (straight)
        
			Crawl();
            reply.addString("crawling");
            
            break;
                
        case 3: ///- crawl faster
            
            Crawl(turnAngle,STANCE_INDENT);
            reply.addString("crawling faster");
            
            break;
            
        case 4:///- crawl slower
            
            Crawl(turnAngle, -STANCE_INDENT);
            reply.addString("crawling slower");
            
            break;
            
        case 5: ///- turn to the right
            
			//Crawl(-MAX_TURN_ANGLE);
			Crawl(turnAngle-TURN_INDENT, 0);
            reply.addString("turning right");
            
            break;
            
        case 6:///- turn to the left

			//Crawl(MAX_TURN_ANGLE);
            Crawl(turnAngle+TURN_INDENT,0);
            reply.addString("turning left");
            
            break;


		case SEB_TURN_COMMAND: ///- turn to avoid obstacles 
        {
            double angle = command.get(1).asDouble();
            cout << "angle : " << angle << endl;

			Crawl(angle);
            
			reply.addString("turning\n");
			STATE = CRAWL;

            break;
        }

		case HEAD_ROT_COMMAND :
		{
			double pitchAngle = command.get(1).asDouble();
			double yawAngle = command.get(2).asDouble();

			HeadControl(pitchAngle, yawAngle);
			
			break;
		}
        
		case REACH_COMMAND: ///- reach for a mark on the ground
		{
			Reach(command.get(1).asList());	

            reply.addString("reaching\n");
			STATE = REACH;

            break;
		}                
		//=========END added by Seb=========

		case 9: /// - emergency stop  (will stop the module)
                
            crawl_left_parameters[9][1]=turnAngle; 
            
            //if the robot is not crawling, we go to an intermediate position
            //otherwise, we do nothing
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
            
            reply.addString("EMERGENCY STOP");
            
            crawl_left_parameters[9][1]=0.0;
            
            break;

        default:
            break;
            
        }
        
    return true;
       
}    
      
bool CrawlManagerModule::open(Searchable &s)
{
    Property arguments(s.toString());
    
    //we get the config file
    if(arguments.check("file"))
	{
		options.fromConfigFile(arguments.find("file").asString().c_str());
	}
    else
    {
    	char *cubPath;
    	cubPath = getenv("ICUB_DIR");
    	if(cubPath == NULL) {
    		ACE_OS::printf("generatorThread::init>> ERROR getting the environment variable ICUB_DIR, exiting\n");
    		return false;
    	}
    	yarp::String cubPathStr(cubPath);
    	options.fromConfigFile((cubPathStr + "/app/Crawling/config/managerConfig.ini").c_str());
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
    turnAngle=0;
    
    vector<double> crawl_ampl;
    vector<double> crawl_target;
    vector<double> init_ampl;
    vector<double> init_target;

    //we connect to the ports
    for(int i=0;i<nbParts;i++)
    {                      
        char tmp1[255],tmp2[255];
        sprintf(tmp1,"/%s/parameters/in",part_names[i].c_str());

        /////////////////////////////////////////////////////
        // check if the part is active and connect///////////
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
                    
            connected_part[i] = true; //the part i is connected
            
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
			  
            
        ///////we get the parameters from the config files
        if(options.check(part_names[i].c_str()))
        {
            Bottle& bot = options.findGroup(part_names[i].c_str());
            
            Bottle& bot1=bot.findGroup("nbJoints");
            nbDOFs[i] =bot1.get(1).asInt();
            ACE_OS::printf("nbJoints %d\n", nbDOFs[i]);

			om_swing.push_back(bot.find("omSwing").asDouble());
			om_stance.push_back(bot.find("omStance").asDouble());
            
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
                ACE_OS::printf("please specify left crawl config for %s\n",part_names[i].c_str());
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
                ACE_OS::printf("please specify right crawl config for %s\n",part_names[i].c_str());
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
            printf(")\n", om_stance[i], om_swing[i]); 
            
            ACE_OS::printf("crawl left parameters for part %s\n", part_names[i].c_str());
            printf("amplitudes ( ");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", crawl_left_parameters[2*i][j]);
            printf(")\ntargets (");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", crawl_left_parameters[2*i+1][j]);
            printf(")\n", om_stance[i], om_swing[i]); 
            
            ACE_OS::printf("crawl right parameters for part %s\n", part_names[i].c_str());
            printf("amplitudes ( ");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", crawl_right_parameters[2*i][j]);
            printf(")\ntargets (");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", crawl_right_parameters[2*i+1][j]);
            printf(")\n", om_stance[i], om_swing[i]); 
            
            ACE_OS::printf("init parameters for part %s\n", part_names[i].c_str());
            printf("amplitudes ( ");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", init_parameters[2*i][j]);
            printf(")\ntargets (");
            for(int j=0; j<nbDOFs[i]; j++) 
                printf("%f ", init_parameters[2*i+1][j]);
            printf(")\n", om_stance[i], om_swing[i]); 
        }
        else
        {
			ACE_OS::printf("please specify config for %s\n",part_names[i].c_str());
			return false;
        }
    }
    
	//=========START added by Seb=========
	commandPort.open(COMMAND_PORT_NAME);
    if(!(attach(commandPort,true)))
    {
        cout << "WARNING : port not attached to respond function" << endl;
    }
	//=========END added by Seb=========

      
    return true;

}

///This function returns which arm is in the swinging phase according to the cpgs info 
///(1= left arm, -1= right arm, 0= no info or not applicable)
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
    }
    
    ACE_OS::printf("y_speed %f %f %f %f\n", 
                            y_speed[0], y_speed[1], y_speed[2], y_speed[3]);
    
    if(swing[0] || swing[3] || !swing[1] ||!swing[2]) side = LEFT_ARM;
    if(!swing[0] || !swing[3] || swing[1] ||swing[2]) side = RIGHT_ARM;

    return side; 
}

///This functions sends the command parameters to the different generators
void CrawlManagerModule::sendCommand(int i, vector<vector<double> > params)
{
    Bottle& paramBot = parts_port[i].prepare();
    paramBot.clear();
    for(int j=0;j<nbDOFs[i];j++)
    {
        paramBot.addDouble(params[2*i][j]); 
        paramBot.addDouble(params[2*i+1][j]);                               
    }
    paramBot.addDouble(om_stance[i]);
    paramBot.addDouble(om_swing[i]);
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
        
    printf(")\n om_stance %f, om_swing %f, turnAngle %f\n\n", om_stance[i], om_swing[i], turnAngle);  
    #endif                         
}

double CrawlManagerModule::getPeriod()
{
    return 2.0;
}
 
bool CrawlManagerModule::close()
{
	ACE_OS::printf( "crawl manager module closing...");
    for(int i=0;i<6;i++)
        if(connected_part[i]==true) 
		{
			parts_port[i].close();
			check_port[i].close();
		}
	commandPort.close();
    return true;
}

void CrawlManagerModule::InitPosition(void)
{
    //if the robot crawls, we need to check if it safe before going on all fours
    if(STATE==CRAWL)
    {
        //if it turns (turnAngle!=0), it should go back to the initial position while crawling
        if(turnAngle < -0.001)
        {
            while(turnAngle < -0.001)
            {
                turnAngle+=TURN_INDENT;
                crawl_parameters[9][1]=turnAngle;
                for(int i=0;i<nbParts;i++)
                    if(connected_part[i]) sendCommand(i, crawl_parameters);
                Time::delay(1.0);
            }
        }
        
        if(turnAngle > 0.001)
        {
            while(turnAngle > 0.001)
            {
                turnAngle-=TURN_INDENT;
                crawl_parameters[9][1]=turnAngle;
                for(int i=0;i<nbParts;i++)
                    if(connected_part[i]) sendCommand(i, crawl_parameters);
                Time::delay(1.0);
            }
        }
        
        //the robot first goes to an intermediate position before going on all fours
        //the swinging arm is lifted and then put on the ground                    
        int side=0;
        while(side==0) {
        	side=getSwingingArm();
        }
        
		cout << "side : " << side << endl;

        if(side==LEFT_ARM)
        {
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_left_parameters);
            Time::delay(5.0);
        }
        
        if(side==RIGHT_ARM)
        {
            for(int i=0;i<nbParts;i++)
                if(connected_part[i]) sendCommand(i, crawl_right_parameters);
            Time::delay(5.0);
        }
    }
    
    //we can now send the init position
    //the stance frequency om_stance is reinitialized            
    for(int i=0;i<nbParts;i++)
	{
        if(connected_part[i])
		{
			sendCommand(i, init_parameters);
			om_stance[i] = om_swing[i];
		}
	}
    
	STATE = INIT_POS;
}


void CrawlManagerModule::Crawl(double desiredTurnAngle, double stanceIncrement)
{
	if(desiredTurnAngle > MAX_TURN_ANGLE)
	{
		desiredTurnAngle = MAX_TURN_ANGLE;
        ACE_OS::printf("Already at MAX TURN ANGLE %d\n", MAX_TURN_ANGLE);  
	}
	else if(desiredTurnAngle < -MAX_TURN_ANGLE)
	{
		desiredTurnAngle = -MAX_TURN_ANGLE;
        ACE_OS::printf("Already at MIN TURN ANGLE %d\n", -MAX_TURN_ANGLE);  

	}

	//We have the robot turn incrementally to the desired turn angle.
    if(STATE==CRAWL && desiredTurnAngle != turnAngle)
	{
		if(turnAngle < desiredTurnAngle)
		{
			while(turnAngle <  desiredTurnAngle-TURN_INDENT/2)
			{
				turnAngle+=TURN_INDENT;
				crawl_parameters[9][1]=turnAngle;
				for(int i=0;i<nbParts;i++)
				{
					if(part_names[i] == "head")
					{
						continue;
					}
					if(connected_part[i]) sendCommand(i, crawl_parameters);
				}
				Time::delay(1.0);
			}
		}
    
		if(turnAngle > desiredTurnAngle)
		{
			while(turnAngle > desiredTurnAngle+TURN_INDENT/2)
			{
				turnAngle-=TURN_INDENT;
				crawl_parameters[9][1]=turnAngle;
				for(int i=0;i<nbParts;i++)
				{
					if(part_names[i] == "head")
					{
						continue;
					}
					if(connected_part[i]) sendCommand(i, crawl_parameters);
				}
				Time::delay(1.0);
			}
		}
	}

    //if the robot is not crawling, we go first to an intermediate position
    if(STATE!=CRAWL)
    {
        for(int i=0;i<nbParts;i++)
            if(connected_part[i])
			{
				sendCommand(i, crawl_right_parameters);
			}
        turnAngle=0;
        Time::delay(5.0);
    }
    
    //we can now send the normal crawling parameters
    for(int i=0;i<nbParts;i++)
        if(connected_part[i]) 
		{
            if(part_names[i]!="head") om_stance[i]+=stanceIncrement;
            
			sendCommand(i, crawl_parameters);
		}
    
    STATE = CRAWL;
}

void CrawlManagerModule::Reach(Bottle *reachingCommand)
{
	ACE_OS::printf("command : %s\n",reachingCommand->toString().c_str());
	ConstString reachingPart = reachingCommand->get(0).asString();
	ACE_OS::printf( "REACHING WITH PART %s\n", reachingPart.c_str());

	if(STATE!=CRAWL)
    {
		if(reachingPart == "left_arm")
		{
			for(int i=0;i<nbParts;i++)
			{
				if(part_names[i] == "head")
				{
					continue;
				}
				if(connected_part[i]) sendCommand(i, crawl_left_parameters);
			}
		}
		else if(reachingPart == "right_arm")
		{
			for(int i=0;i<nbParts;i++)
			{
				if(part_names[i] == "head")
				{
					continue;
				}
				if(connected_part[i]) sendCommand(i, crawl_right_parameters);
			}
		}
        Time::delay(2.0);
    }

	vector<vector<double> > reach_parameters;
	for(int i=0;i<nbParts;i++)
	{
		if(part_names[i] == "head")
		{
			continue;
		}
		if(connected_part[i])
		{
			ConstString currentPart = part_names[i];
			if(part_names[i] == ConstString(reachingPart.c_str())/* == currentPart*/)
			{
				vector<double> amplitude;
				vector<double> offset;
				for(int j=0; j<nbDOFs[i]; ++j)
				{
					amplitude.push_back(-1);
					offset.push_back(reachingCommand->get(j+1).asDouble());
				}
				reach_parameters.push_back(amplitude);
				reach_parameters.push_back(offset);
			}
			else
			{
				vector<double> amplitude;
				vector<double> offset;
				reach_parameters.push_back(init_parameters[2*i]);
				reach_parameters.push_back(init_parameters[2*i+1]);
			}
			sendCommand(i, reach_parameters);
		}
	}
}

void CrawlManagerModule::HeadControl(double pitchAngle, double yawAngle)
{
	vector<vector<double> > head_parameters;
	int headIndex;
	for(int i=0;i<nbParts;i++)
	{
		if(connected_part[i])
		{
			ConstString currentPart = part_names[i];
			if(part_names[i] == "head"/* == currentPart*/)
			{
				headIndex = i;
				vector<double> amplitude;
				vector<double> offset;
				amplitude.push_back(-1);
				offset.push_back(pitchAngle);

				amplitude.push_back(-1);
				offset.push_back(0);

				amplitude.push_back(-1);
				offset.push_back(yawAngle);

				for(int j=3; j<nbDOFs[i]; ++j)
				{
					amplitude.push_back(-1);
					offset.push_back(0.0);
				}
				head_parameters.push_back(amplitude);
				head_parameters.push_back(offset);
			}
			else
			{
				vector<double> amplitude;
				vector<double> offset;
				head_parameters.push_back(init_parameters[2*i]);
				head_parameters.push_back(init_parameters[2*i+1]);
			}
		}
	}
	sendCommand(headIndex, head_parameters);
}

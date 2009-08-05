#include <stdio.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <ace/OS.h>

#include <iCub/iha/debug.h>
#include <iCub/iha/mem_util.h> // from ControlBoardInterfaces.inl

#include <iCub/iha/Actions.h>
#include <iCub/iha/ActionSelect.h>


using namespace std;
using namespace yarp::os;
using namespace iCub::iha;


void iCub::iha::ActionSelect::init(Searchable& config) {
	wheelResolution = 10000;

	neighbour_radius = config.check("neighbour_radius",Value(1.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"neighbour_radius %f\n",neighbour_radius);

	Temperature = config.check("temperature",Value(4.0)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"temperature %f\n",Temperature);

	temp_dec = config.check("temp_dec",Value(0.002)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"temp_dec %f\n",temp_dec);

	// Read horizons (here as well as in Module)
	Bottle& horizonsConfig =  config.findGroup("HORIZONS");
	
	int h=0;
	for (int i=1;i<horizonsConfig.size();i++) {
		horizons[h] = horizonsConfig.get(i).asInt();
		h++;
	}
	numHorizons=h;
}

/**
 * Read the action list from the given file
 * into the Actions class
 *
 * Populates action_commands array and NUM_ACTIONS
 */
void ActionSelect::getActions(ConstString filename, ConstString seqdir) {
    //------------------------------------------------------
	// create the action defs object and read the actions
	// from the config file
	Property actiondefs_props;
	actiondefs_props.fromConfigFile(filename.c_str());
	
	if (!iCubActions.open(actiondefs_props, seqdir)) {
		fprintf(stderr,"Error in action definitions\n");
		exit(-1);
	}

	action_commands = iCubActions.getFullActionCommandList();
	NUM_ACTIONS = action_commands.size();
	IhaDebug::pmesg(DBGL_DEBUG2,"NUM_ACTIONS defined : %d\n",NUM_ACTIONS);
}


int iCub::iha::ActionSelect::chooseFromDistribution(vector<double> distribution) {
	// rand() return an integer between 0 and MAXINT
	// convert to an int between 0 and wheelResolution
	int r = rand() % wheelResolution;

	int accumProb = 0;
	int chosen = -1;

	if (IhaDebug::getLevel()>=DBGL_DEBUG2) {
		IhaDebug::pmesg(DBGL_DEBUG2,"chooseFromDistribution: r=%d\n",r);
		IhaDebug::pmesg(DBGL_DEBUG2,"Dist:");
		for (int a=0 ; a<distribution.size() && chosen<0 ; a++) {
			IhaDebug::pmesg(DBGL_DEBUG2," %d:%f=%d",a,distribution[a],(int) (distribution[a] * (double)wheelResolution));
		}
		IhaDebug::pmesg(DBGL_DEBUG2,"\n");
	}
	for (int a=0 ; a<distribution.size() && chosen<0 ; a++) {
		accumProb += (int) (distribution[a] * (double)wheelResolution);
		if (r < accumProb) {
			chosen = a;
		}
	} 
	IhaDebug::pmesg(DBGL_DEBUG2,"Chose: %d\n",chosen);
	return chosen;
}


// uses parameter variables:
// numHorizons, horizons[]
//
// populates:
// actfreq_distributions, elist, distances, masses, values
//
bool iCub::iha::ActionSelect::parseNeighbourList(Bottle* neighbourlist[]) 
{
	elist.clear();
	distances.clear();
	actfreq_distributions.clear();
	masses.clear();
	values.clear();

	for (int hor=0; hor < numHorizons; hor++) 
	{
		if (neighbourlist[hor]==NULL) {
			IhaDebug::pmesg(DBGL_STATUS2,"No neighbour list received for horizon %d\n",horizons[hor]);
			continue;
		}
		IhaDebug::pmesg(DBGL_DEBUG3,"RCV:%s\n",neighbourlist[hor]->toString().c_str());

		// check this is a proper line (invalid lines have an expid of -1)
		//   item 0 is horizon
		//   item 1 is expid
		//   data starts at n-2
		if (neighbourlist[hor]->get(1).asInt() >= 0 && neighbourlist[hor]->size()>2) 
		{
			int expid = neighbourlist[hor]->get(1).asInt();

			IhaDebug::pmesg(DBGL_DEBUG2,"Distances from exp %d h=%d :\n",expid,horizons[hor]);
			// loop through neighbours
			for (int n=2; n < neighbourlist[hor]->size(); n++) {
				// get the distance and experience id to which this distance relates
				
				Bottle* itemn = neighbourlist[hor]->get(n).asList();
				int    exp  = itemn->get(0).asInt();	// item 0 expid
				double dist = itemn->get(1).asDouble();	// item 1 distance

				// save the distance in the map if it is within the radius we
				// are interested in
				if (dist <= neighbour_radius) {
					pair<int,int> he = pair<int,int>(horizons[hor],exp);
					elist.insert(pair<double,pair<int,int> >(dist, he));
					distances.insert(pair<pair<int,int>,double >(he, dist));
				

					double val  = itemn->get(2).asDouble();   // item 3 value
				
					// store the value for this hor/exp
					values[pair<int,int>(horizons[hor],exp)] = val;

					IhaDebug::pmesg(DBGL_DEBUG2,"exp: %d dist: %f val: %f actfreq: ",exp,dist,val);

					Bottle* actfreql = itemn->get(3).asList();// item 4 action frequency distribution
					//
					// zero the distribution
					for (int a=0;a<NUM_ACTIONS;a++) {
						actfreq_distributions[he].push_back(0);
					}
					// store the action frequency distribution
					int totalaf=0;
					for (int afl=0;afl<actfreql->size();afl++) {
						int ac = actfreql->get(afl).asList()->get(0).asInt();
						int af = actfreql->get(afl).asList()->get(1).asInt();
						IhaDebug::pmesg(DBGL_DEBUG2,"%d:%d ",ac,af);
						actfreq_distributions[he][ac]=(double)af;
						totalaf+=af;;
					}
					IhaDebug::pmesg(DBGL_DEBUG2,"\n");
					// turn into probabilities by dividing by number of actions
					// in the frequecy distribution
					for (int i=0;i< actfreq_distributions[he].size();i++) {
						actfreq_distributions[he][i] /= (double)totalaf;
					}
					// the total of the action frequencies = mass of the experience
					masses[he] = (double)totalaf;

				}

			} // loop thru neighbours
		} // valid line

	} // for

	return true;
}

// uses:
//   numHorizons, horizons, 
//   elist, values, masses, actfreq_distributions, 
//   use_behaviour_sets, next_behaviour_set
// modifies:
//
// returns:
//   action number
int iCub::iha::ActionSelect::selectAction() 
{
	// we have a list of experiences ordered by distance.
	// Now create a probability map based on distance and value. This
	// map will be used to decide which experience to use for the 
	// next action selection.

	// Gravitation-like model 
	//      value_i
	// Vi = --------  * mass
	//        d^2   
	//
	// Also: adjust for horizon size
	// Vi *= log(h) / log(Hmax)
	// or Vi *= sqrt(h)/sqrt(Hmax)
	//
	map<pair<int,int>, double> probs;
	double sumprobs=0, sumvalues=0, summasses=0;
	double logHmax = log(horizons[numHorizons-1]);
	double sqrtHmax = sqrt(horizons[numHorizons-1]);

	IhaDebug::pmesg(DBGL_DEBUG2,"Probability List:\n");

	for (multimap<double,pair<int,int> >::iterator it=elist.begin();it!=elist.end();it++) {
		double dist = it->first;
		int horiz = it->second.first;
		int exp = it->second.second;

		double value = values[ pair<int,int>(horiz,exp) ];
		double mass = masses[ pair<int,int>(horiz,exp) ];

		// adjust for horizon length
		//if (numHorizons>1) value *= horiz/horizons[numHorizons-1];
		//if (numHorizons>1) value *= log(horiz)/logHmax;
		if (numHorizons>1) value *= sqrt(horiz)/sqrtHmax;

		double p;
		if (dist>0) {
			p = value * mass / (dist * dist);
		} else {
			//p = value * mass / 0.00001;
			p = 0; // don't choose experiences with dist=0
		}
		probs[pair<int,int>(horiz,exp)] = p;

		sumprobs+=p;
		sumvalues+=value;
		summasses+=mass;

		/*pmesg(DBGL_DEBUG2,"Hor:%d Exp:%d Dist:%f Value:%f -> Vi=%f AF:%4.2f %4.2f %4.2f %4.2f\n",
				horiz,exp,dist,value,p,
				actfreq_distributions[pair<int,int>(horiz,exp)][0],
				actfreq_distributions[pair<int,int>(horiz,exp)][1],
				actfreq_distributions[pair<int,int>(horiz,exp)][2],
				actfreq_distributions[pair<int,int>(horiz,exp)][3]
			 );*/
		IhaDebug::pmesg(DBGL_DEBUG2,"Hor:%d Exp:%d Dist:%f Value:%f -> Vi=%f\n", horiz,exp,dist,value,p);
	}
	
	// add chance of random by placing a single point of value = sum(values)
	// mass should balance out the other masses ...
	// at a distance of R_max / Temperature
	// CHANGE 21/3/08 - Use value = 1.0
	sumvalues=1.0;
	double p = sumvalues * summasses / ((neighbour_radius/Temperature)*(neighbour_radius/Temperature));

	probs[ pair<int,int>(0,0)] = p;
	sumprobs+=p;
	IhaDebug::pmesg(DBGL_DEBUG2,"Vi(Random)=%f\n, sum(Vi)=%f",p,sumprobs);

	// Weighting of experiences
	//
	//        V_i
	// Wi = ------- 
	//      sum(V_i)
	//


	// sorted list of probabilities
	multimap<double, pair<int,int> > sprobs;
	for (map<pair<int,int>, double>::iterator it=probs.begin();it!=probs.end();it++) {
		sprobs.insert(pair< double, pair<int,int> >(it->second, it->first));
	}

	// roulette wheel selection of one of the experiences
	int r = rand() % wheelResolution;
	IhaDebug::pmesg(DBGL_STATUS2,"Roulette Num : %f\n",((double)r)*100.0/((double)wheelResolution));
	//
	double accumProb=0;

	select_chosen=false; 
	select_random=false;

	pair<int,int> chosenexp;
	for (map<double,pair<int,int> >::reverse_iterator spit=sprobs.rbegin();select_chosen==false && spit!=sprobs.rend();spit++) {
		int h=spit->second.first;
		int e=spit->second.second;
		double p=spit->first;

		accumProb += ( p / sumprobs ) * (double) wheelResolution;
		if (r < accumProb) {
			select_chosen=true;
			if (h==0 && e==0) {
				select_random=true;
			} else {
				chosenexp = pair<int,int>(h, e);
			}
		}
	}

	// now choose an action from the action probabilities for this experience
	
	int chosenAction;

	// 1st case: we have a chosen experience, and we are not required to pick a random action
	if (select_chosen && !select_random) {
		
		// Debug: print out the action probability distribution
		IhaDebug::pmesg(DBGL_DEBUG1,"Chosen experience: %d hor %d\n",chosenexp.second,chosenexp.first);
		if (IhaDebug::getLevel()>=DBGL_DEBUG2) {
			IhaDebug::pmesg(DBGL_DEBUG2,"Action Probabilities for chosen experience:\n");
			for (int i=0;i<NUM_ACTIONS;i++) {
				IhaDebug::pmesg(DBGL_DEBUG2,"%d:%f ",i,actfreq_distributions[chosenexp][i]);
			}
			IhaDebug::pmesg(DBGL_DEBUG2,"\n");
		}

		if (use_behaviour_sets)
		{
			// create a subset of the distribution using only actions in the 
			// next behaviour set
			vector<double> bset_distribution;
			double total_prob=0;
			// loop through the behaviour set
			IhaDebug::pmesg(DBGL_DEBUG2,"Building distribution subset\n");
			for (int a=0; a<next_behaviour_set.size(); a++) {
				// for each action in the set, add the frequency to a new distribution
				int na=next_behaviour_set[a];
				IhaDebug::pmesg(DBGL_DEBUG2,"Adding: Set item %d=Act %d  :  Freq %f\n",a,na,actfreq_distributions[chosenexp][na]);
				bset_distribution.push_back(actfreq_distributions[chosenexp][na]);
				total_prob += actfreq_distributions[chosenexp][na];
			}
			IhaDebug::pmesg(DBGL_DEBUG2,"Done, total freq = %f\n",total_prob);
			if (total_prob==0) {
				// no action from the subset has a frequency
				// so create a even distribution for random choice
				for (int a=0;a<bset_distribution.size();a++) {
					bset_distribution[a]=1.0;
				}
				total_prob=bset_distribution.size();
				IhaDebug::pmesg(DBGL_STATUS2,"No action frequencies in subset, use even distribution\n");
			}
			// normalise to a total of 1.0
			for (int i=0; i<bset_distribution.size(); i++) {
				bset_distribution[i] /= total_prob;
			}
			
			// More Debug
			if (IhaDebug::getLevel()>=DBGL_DEBUG2) {
				IhaDebug::pmesg(DBGL_DEBUG2,"Subset of Action Probabilities:\n");
				for (int i=0;i<bset_distribution.size();i++) {
					IhaDebug::pmesg(DBGL_DEBUG2,"Act %d: %f ",next_behaviour_set[i],bset_distribution[i]);
				}
				IhaDebug::pmesg(DBGL_DEBUG2,"\n");
			}

			// choose a number from the subset
			int cnum = chooseFromDistribution(bset_distribution);
			if (cnum<0) {
				// if we don't choose an action from the distribution there is something wrong
				fprintf(stderr,"Error: choosing action from distribution failed\n");
				exit(1);
			} else {
				// cnum is an index into the behaviour set, so get action number
				// from the set
				chosenAction=next_behaviour_set[cnum];
			}
		}
		else
		{
			int cnum = chooseFromDistribution(actfreq_distributions[chosenexp]);
			if (cnum<0) {
				// if we don't choose an action from the distribution there is something wrong
				fprintf(stderr,"Error: choosing action from distribution failed\n");
			} else {
				chosenAction=cnum;
			}
		}


		IhaDebug::pmesg(DBGL_STATUS1,"Choosing action %d experience %d horizon %d\n",(int)chosenAction,chosenexp.second,chosenexp.first);

	} 
	else 
	// 2nd case: no chosen experience so choose a random one 
	{
		chosenAction=selectRandomAction(next_behaviour_set);
	}

	//---------------------------------------------------------
	// debug printing of experience/action probabilities
	if (IhaDebug::getLevel()>=DBGL_STATUS2) {
		// sorted
		IhaDebug::pmesg(DBGL_STATUS2,"Weighting: \n");
		for (multimap<double, pair<int,int> >::reverse_iterator it=sprobs.rbegin();it!=sprobs.rend();it++) {
			string colstr_on="";
			string colstr_off="";
			if (it->second.second==0 && it->second.first==0) {
				colstr_on += "\E[31m"; // RED
				colstr_off += "\E[37m";
				IhaDebug::pmesg(DBGL_STATUS2,"%s==== Random ====  Wi %7.6f%%  Mass %3.0f  Val %4.1f%s                     ",
						colstr_on.c_str(),
						100 * it->first / sumprobs,
						summasses,
						sumvalues,
						colstr_off.c_str());
				if (select_random) {
					IhaDebug::pmesg(DBGL_STATUS2," ==> random Act: %d\n",chosenAction);
				} else {
					IhaDebug::pmesg(DBGL_STATUS2,"\n");
				}
			} else {
				//if (values[it->second]>VAL_MID) {
					colstr_on += "\E[32m"; // GREEN
					colstr_off += "\E[37m";
				//}
				IhaDebug::pmesg(DBGL_STATUS2,"%sExp %5d Hor %3d  Wi %7.6f%% D %7.6f  Mass %3.0f  Val %4.1f  AF:",
					colstr_on.c_str(),
					it->second.second,
					it->second.first,
					100 * it->first / sumprobs,
					distances[it->second],
					masses[it->second],
					values[it->second]
					 );
				for (int a=0;a<NUM_ACTIONS;a++) {
					IhaDebug::pmesg(DBGL_STATUS2," %3.0f",actfreq_distributions[it->second][a]*100);
				}

				if (select_chosen 
					&& !select_random 
					&& chosenexp.first==it->second.first 
					&& chosenexp.second==it->second.second) 
				{
					IhaDebug::pmesg(DBGL_STATUS2,"%s ==> chosen Act: %d\n",colstr_off.c_str(), chosenAction);
				} else {
					IhaDebug::pmesg(DBGL_STATUS2,"%s\n",colstr_off.c_str());
				}
			}
		}
	}
	
    // Cooling - ie reduction in chance of Random over time
    // Adaptive temperature control instead is in the action control
    // main procedures as it is tied in with reward
    Temperature -= temp_dec;
    if (Temperature<0) Temperature=0;

	//---------------------------------------------------------
	// finally, return the action
	//
	string rand_txt;
	if (select_chosen && ! select_random) {
		rand_txt = "chosen";
	} else {
		rand_txt = "random";
	}

	//IhaDebug::pmesg(DBGL_STATUS1,"current action %d returning %s action %d\n",current_action, rand_txt.c_str(), chosenAction);
	IhaDebug::pmesg(DBGL_STATUS1,"ActionSelect returning %s action %d\n", rand_txt.c_str(), chosenAction);

	return chosenAction;
}

void ActionSelect::updateNextBehaviourSet(int act) {
	if (iCubActions.getActionNextBehaviourSetNo(act) >= 0) {
		next_behaviour_set=iCubActions.getActionNextBehaviourSet(act);
	}
}

int ActionSelect::selectRandomAction(vector<int> next_behaviour_set) 
{
	int chosenAction;
	if (use_behaviour_sets) 
	{
		chosenAction = chooseRandomFromSet(next_behaviour_set);
		IhaDebug::pmesg(DBGL_STATUS1,"Choosing random action %d from set \n",(int)chosenAction);
		IhaDebug::pmesg(DBGL_DEBUG1,"Behaviour Set : ");
		for (int i=0;i<next_behaviour_set.size();i++) {
			IhaDebug::pmesg(DBGL_DEBUG1,"%d ",next_behaviour_set[i]);
		}
		IhaDebug::pmesg(DBGL_DEBUG1,"\n");
	}
	else
	{
		// choose a random action from all available
		chosenAction  = chooseRandom();
		IhaDebug::pmesg(DBGL_STATUS1,"Choosing random action %d\n",(int)chosenAction);
	}
	 return chosenAction;

}

#include <string>
#include <map>

#include <iCub/iha/Actions.h>
#include <iCub/iha/Sequence.h>
#include <iCub/iha/Expression.h>

#include <iCub/iha/debug.h>

using namespace std;
using namespace iCub::iha;


iCub::iha::Actions::~Actions() 
{
	delete[] action_names;
	delete[] action_commands;
	delete[] seq_fname;
	delete[] joint_names;
	delete[] joint_motors;
}

//bool iCub::iha::Actions::open(yarp::os::Property& config, yarp::os::ConstString seq_dir) 
bool iCub::iha::Actions::open(yarp::os::Property& config) 
{
	
  yarp::os::ConstString seq_dir = config.check("SequenceDir",yarp::os::Value("conf/sequences")).asString();

	num_joints = config.find("NUM_JOINTS").asInt();
	IhaDebug::pmesg(DBGL_INFO,"%d joints configured\n",num_joints);
	joint_names = new string[num_joints];
	joint_motors = new int[num_joints];

	yarp::os::Bottle& jointNamesGroup = config.findGroup("JOINT_NAMES");
	if (jointNamesGroup.isNull()) {
		ACE_OS::fprintf(stderr,"Error: JOINT_NAMES group not found\n");
		return 0;
	}
	yarp::os::Bottle& jointMotorsGroup = config.findGroup("JOINT_MOTORS");
	if (jointMotorsGroup.isNull()) {
		ACE_OS::fprintf(stderr,"Error: JOINT_MOTORS group not found\n");
		return 0;
	}

	for (int j=0;j<num_joints;j++) {
		joint_names[j]=jointNamesGroup.get(j+1).asString();
		joint_motors[j]=jointMotorsGroup.get(j+1).asInt();
		joint_name_motor_map.insert(pair<string,int>(joint_names[j],joint_motors[j]));
		IhaDebug::pmesg(DBGL_DEBUG2,"Joint : %s -> %d\n",joint_names[j].c_str(),joint_motors[j]);
	}

	num_actions = 0;

	char actname[10];

	if (!config.check("NUM_ACTIONS")) {
		return false;
	}
	num_actions = config.find("NUM_ACTIONS").asInt();

	IhaDebug::pmesg(DBGL_INFO,"%d actions configured\n",num_actions);

	action_names = new string[num_actions];
	action_commands = new string[num_actions];
	seq_fname = new string[num_actions];
	nextBSet = new int[num_actions];

	IhaDebug::pmesg(DBGL_DEBUG2,"arrays allocated\n");

	for (int a=0;a<num_actions;a++) {
		IhaDebug::pmesg(DBGL_DEBUG2,"action #%d:\n",a);

		ACE_OS::sprintf(actname,"ACTION%02d",a);

		yarp::os::Bottle &aGroup = config.findGroup(actname);
		if (aGroup.isNull()) {
			IhaDebug::pmesg(DBGL_INFO,"error - action %d missing from config\n",a);
			return false;
		}

		string name (aGroup.find("Name").asString().c_str());
		action_names[a] = name;
		IhaDebug::pmesg(DBGL_STATUS2,"action: %s\n",action_names[a].c_str());

		action_commands[a] = string(aGroup.find("Command").asString().c_str());
		action_command_map[action_commands[a]]=a;

		nextBSet[a] = aGroup.find("NextSet").asInt();

		Sequence seq;
		seq.name = action_names[a];
		Expression expr;

		seq_fname[a] = string(seq_dir.c_str());
		seq_fname[a].append("/");
		if ( aGroup.check("ExpressionFile") ) {
			seq_fname[a].append(aGroup.find("ExpressionFile").asString().c_str());
			expr.read(seq_fname[a]);
			IhaDebug::pmesg(DBGL_INFO,"Read Expression: %s\n",seq_fname[a].c_str());
			expr.print();
		} else {
			seq_fname[a].append(aGroup.find("SequenceFile").asString().c_str());
			seq.read(seq_fname[a]);
			IhaDebug::pmesg(DBGL_INFO,"Read Sequence: %s\n",seq_fname[a].c_str());
			seq.print();
		}


		sequences.push_back(seq);
		expressions.push_back(expr);
	}

	IhaDebug::pmesg(DBGL_STATUS2,"Reading behaviours\n");
	yarp::os::Bottle &bsetsGroup = config.findGroup("BEHAVIOURSETS");
	if (bsetsGroup.isNull()) {
		IhaDebug::pmesg(DBGL_INFO,"No behaviour sets defined - default single set containing all actions created\n");
		num_behavioursets=1;
		vector<int> bset;
		for (int a=0;a<num_actions;a++) {
			bset.push_back(a);
		}
		bsets.push_back(bset);
	}
	num_behavioursets = bsetsGroup.check("NUM",yarp::os::Value(0)).asInt();
	if (num_behavioursets<1) {
		IhaDebug::pmesg(DBGL_INFO,"error num_behavioursets invalid\n");
		return false;
	}
	bset_default = bsetsGroup.check("DEFAULT",yarp::os::Value(0)).asInt();

	for (int s=0;s<num_behavioursets;s++) {
		char setname[20];
		ACE_OS::sprintf(setname,"SET%d",s);

		yarp::os::Bottle &bsetGroup = bsetsGroup.findGroup(setname);
		if (bsetGroup.isNull()) {
			IhaDebug::pmesg(DBGL_INFO,"error - set %d missing from config\n",s);
			return false;
		}
		vector<int> bset;
		int groupindex=1;
		IhaDebug::pmesg(DBGL_DEBUG2,"BSet Group %d :");
		while ( groupindex < bsetGroup.size() ) {
			int act=bsetGroup.get(groupindex).asInt();
			bset.push_back(act);
			IhaDebug::pmesg(DBGL_DEBUG2," %d",act);
			groupindex++;
		}
		bsets.push_back(bset);
		IhaDebug::pmesg(DBGL_DEBUG2,"\n");
	}
	
	return true;
}


int iCub::iha::Actions::getActionIndex(std::string name) {
  int act = -1;
  for(int i = 0; i < num_actions; i++) {
    if(action_names[i].compare(name) == 0) {
      act = i;
      break;
    }
  }
  return act;
}


vector<std::string> iCub::iha::Actions::getActionCommandList() 
{
	return getActionCommandList(bset_default);
}

vector<std::string> iCub::iha::Actions::getActionCommandList(int behaviour_set) 
{
	vector<std::string> actlist;
	if (behaviour_set<0 || behaviour_set>=num_behavioursets) {
		IhaDebug::pmesg(DBGL_INFO,"Error: incorrect behaviour set %d\n",behaviour_set);
		return actlist;
	}
	for (vector<int>::iterator it=bsets[behaviour_set].begin();it!=bsets[behaviour_set].end();it++) {
		actlist.push_back(action_commands[*it]);
	}
	return actlist;
}

vector<std::string> iCub::iha::Actions::getFullActionCommandList() 
{
	vector<std::string> actlist;
	for (int a=0;a<num_actions;a++) {
		actlist.push_back(action_commands[a]);
	}
	return actlist;
}

std::string iCub::iha::Actions::getSequenceString(int act) 
{
	std::string seq;
	IhaDebug::pmesg(DBGL_DEBUG2,"getSequenceString : %d\n",act);

	vector<SequenceLine>::iterator it = sequences[act].sequence.begin();


	while (it!=sequences[act].sequence.end() ){
		if (it->type == SEQTYPE_BLOCK) {
			seq.append("BLK\n");
		}
		if (it->type == SEQTYPE_JOINT_POSITION) {
			stringstream ss;
			ss << "POS " 
			   << joint_name_motor_map[it->joint_name]
			   << " " 
			   << it->value
			   << " " 
			   << it->speed
			   << "\n" ;
			seq.append(ss.str());
		}
		it++;
	}
	IhaDebug::pmesg(DBGL_DEBUG2,"SequenceString :\n%s\n",seq.c_str());
	return seq;
}

std::string iCub::iha::Actions::getExpressionString(int act) {
	std::string expr_str;
	IhaDebug::pmesg(DBGL_DEBUG2,"getExpressionString : %d\n",act);

	//vector<Expression>::iterator it = expressions[act].begin();


	//while (it!=sequences[act].sequence.end() ){
	
	return expr_str;
}

vector<int> iCub::iha::Actions::getBehaviourSet(int bsetno)
{
	return bsets[bsetno];
}
int iCub::iha::Actions::getActionNextBehaviourSetNo(int act) 
{
	return nextBSet[act];
}
vector<int> iCub::iha::Actions::getActionNextBehaviourSet(int act)
{
	int next_bset_no = getActionNextBehaviourSetNo(act);

	// -1 can be interpreted as "do default"
	// or (my intention) keep same behaviour set
	// calling object will need to check for -1 : we don't keep state here
	if (next_bset_no<0) {
		return getBehaviourSet(0); // default behaviour
	}
	return getBehaviourSet(next_bset_no);
}

void iCub::iha::Actions::debugPrintBehaviourSet(int dbgl, int bsetno) {
	IhaDebug::pmesg(dbgl,"Behaviour Set [%d] : ",bsetno);
	for (int i=0;i<bsets[bsetno].size();i++) {
		IhaDebug::pmesg(dbgl,"%d ",bsets[bsetno][i]);
	}
	IhaDebug::pmesg(dbgl,"\n");
}

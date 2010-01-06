#include "DrumIKinSolverModule.h"

DrumIKinSolverModule::DrumIKinSolverModule() 
{
	slv=NULL;
}


bool DrumIKinSolverModule::configure(ResourceFinder &rf)
{                
    string part, slvName;

    if (rf.check("part"))
        part=rf.find("part").asString();
    else
    {
        cout<<"Error: part option is not specified"<<endl;
        return false;
    }

    Bottle &group=rf.findGroup(part.c_str());

    if (group.isNull())
    {
        cout<<"Error: unable to locate "<<part<<" definition"<<endl;
        return false;
    }

    if (group.check("name"))
        slvName=group.find("name").asString();            
    else
    {
        cout<<"Error: name option is missing"<<endl;
        return false;
    }

    if (part=="left_arm" || part=="right_arm")
        slv=new ArmDrumStickSolver(slvName);
    else
    {
        cout<<"Error: "<<part<<" is invalid"<<endl;
        return false;
    }

    return slv->open(group);
}


bool DrumIKinSolverModule::close()
{
	if (slv!=NULL)
		delete slv;
    return true;
}

double DrumIKinSolverModule::getPeriod()    
{ 
	return 0.0;  
}

bool DrumIKinSolverModule::updateModule() 
{ 
	return true; 
}


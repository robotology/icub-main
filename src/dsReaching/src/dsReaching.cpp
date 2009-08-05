// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
#include "iCub/dsReaching.h"
#include "iCub/mathlib.h"

dsReaching::dsReaching():Module(){
  paused = false;
  targetTimer.SetDelay(100);
  commandTimer.SetDelay(100);
  outputTimer.SetDelay(50);
}




bool dsReaching::open(Searchable& config){
  Value& v = config.find("name");
  if(v.isNull()){
    setName("/dsReaching");
  }
  else{
    setName(v.asString().c_str());
  }
  if(!targetPort.open(getName("target:i"))){
    return false;
  }
  if(!commandPort.open(getName("command:i"))){
    return false;
  }

#ifdef PORT_CONTROL
  if(!outputPort.open(getName("position:o"))){
    return false;
  }
#else
  initMotorControl();
#endif
 
  targetTimer.Start();
  commandTimer.Start();
  outputTimer.Start();
  return true;
}

#ifndef PORT_CONTROL
bool dsReaching::initMotorControl(){
  double speed[NB_JOINTS];
  int n;
  Property options;
  options.put("local",getName("/icub/right_arm/control"));
  options.put("remote","/icub/right_arm");
  armDriver = new PolyDriver(options);
  if (!armDriver->isValid()) {
    cout<<"Device not available.  Here are the known devices:\n";
    printf("%s", Drivers::factory().toString().c_str());
    Network::fini();
    return false;
  }
  else{
    armDriver->view(posCtl);
    if(!posCtl){
      cout<<"Error getting IPositionControl interface.\n";
      Network::fini();
      return false;
    }

   posCtl->getAxes(&n);
   for(int i=0;in;i++){
     speed[i] = 100;
   }
   posCtl->setRefSpeeds(speed);
 }
  return true;
}
#endif


bool dsReaching::close(){
  targetPort.close();
  commandPort.close();
#ifdef PORT_CONTROL
  outputPort.close();
#else
  armDriver->close();
  delete armDriver;
#endif
  return true;
}


bool dsReaching::readPorts(){
  if(commandTimer.IsDone()){
    commandTimer.Start();   
    if(!readCommandPort()){ // exit signal
      return false;
    }
  }
  if(targetTimer.IsDone()){
    targetTimer.Start();
    readTargetPort();
  }
  return true;
}


void dsReaching::readTargetPort(){
  Bottle *b;
  int i;
  CVector3_t target;
  b = targetPort.read(false);
  if(b){
    for(i=0;i<3;i++){
      target[i] = (float) b->get(i).asDouble();
    }
    cout<<"desired target: "<< 
    reach.SetLocalTarget(target);
  }
}

bool dsReaching::readCommandPort(){
  float angles[NB_JOINTS];
  int i;
  Bottle *b;
  b = commandPort.read(false);
  if(b){
    ConstString cmd = b->get(0).asString();
    if(cmd=="Pause"){
      paused = true;
    }
    else{
      if(cmd=="Resume"){
	paused = false;
      }
      else{
	if(cmd=="Reset"){
	  for(i=0;i<NB_JOINTS;i++){
	    angles[i] = (float) b->get(i).asDouble();
	  }
	  reach.Icub2Rob(angles);
	  reach.SetActualRobPosition(angles);
	}
	else{
	  if(cmd=="Stop"){
	    return false;
	  }
	  else{
	    cout<<"can't understand command "<<cmd.c_str()<<endl;
	  }
	}
      }
    }
  }
  return true;
}
    


void dsReaching::postOutput(){
  float angles[NB_JOINTS];
  int i;
  reach.GetAngle(angles);
  reach.Rob2Icub(angles);
  //  coutvec4(angles);
#ifdef PORT_CONTROL
  Bottle& b =outputPort.prepare();
  b.clear();  
for(i=0;i<NB_JOINTS;i++){
    b.addDouble((double)angles[i]);
  }
  outputPort.write(true);
#else
  double da[NB_JOINTS];
  for(i=0;i<NB_JOINTS;i++){
    da[i] = (double)angles[i];
  }
  posCtl->positionMove(da);
#endif					  
}
 
bool dsReaching::updateModule(){
  if(!readPorts()){
    return false;
  }
  if(!paused){
    reach.ReachingStep();
    if(outputTimer.IsDone()){
      outputTimer.Start();
      postOutput();   
    }
  }
  return true;
}







int main(int argc,char *argv[]){
  dsReaching dsmod;
  dsmod.openFromCommand(argc, argv,true);
  dsmod.runModule();
}


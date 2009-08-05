#pragma warning(disable: 4995)
#pragma warning(disable: 4996)
#include "visionServer.h"
#include "string.h"
#include <iostream>

using namespace std;

VisionServer::VisionServer(){
#ifdef WITH_YARP
  Network::init();
  registered =0;
  strcpy(portname,"/vision");
#endif
}

VisionServer::~VisionServer(){
#ifdef WITH_YARP 
	Network::fini();
	#endif
}


VisionServer::VisionServer(const char *name){
#ifdef WITH_YARP  
	Network::init();
	#endif
  strcpy(portname,name);
  registered =0;
}

int VisionServer::SetPortName(const char *name){
  if(!registered){
    strcpy(portname,name);
    return 1;
  }
  else{
    return 0;
  }
}

bool VisionServer::Start(){
  cout <<"opening connection"<<endl;
  registered = 1;
  #ifdef WITH_YARP
  return open(portname);
#else
  return false;
#endif
}

void VisionServer::Stop(){
#ifdef WITH_YARP
	if(registered){
	  close();
    registered = 0;
  }
	#endif
}

void  VisionServer::SendPosition(double x, double y, double z){
  //  cout<<"sending "<<x<<" "<<y<<" "<<z<<endl; 
#ifdef WITH_YARP 
	Bottle& b = prepare();
  b.clear();
  b.addDouble(x);
  b.addDouble(y);
  b.addDouble(z);

  //  cout <<"sending "<<b.toString().c_str()<<endl;
  write(true);
#endif
}

void  VisionServer::SendPosition(const double *pos, const int *valid, int nb){
  #ifdef WITH_YARP
	int i,j;
  int nbval=0;
  for(i=0;i<nb;i++){
    nbval+= valid[i];
  }
  if(nbval){
    Bottle& b = prepare();
    b.clear();
    for(j=0;j<nb;j++){
      if(valid[j]){
	b.addInt(j);
	for(i=0;i<3;i++){
	  b.addDouble(pos[3*j+i]);
	}
      }
    }
    write(true);
  }
  #endif
}

/**
 * @return the number of 3d positions read.
 */
int VisionServer::ReadPosition(float *position,int nb_max,int ok[]){
  #ifdef WITH_YARP
	int si,nb_el,j,i,k;
  Bottle *b = read(false);
  if(b){
    int si = b->size();
    int nb_el = si/4;
    if(nb_el>nb_max){
      cout<<"not enough memory allocated for reading positions"<<endl;
      return 0;
    }
    if(si==3){ //only one object tracked
      for(i=0;i<3;i++){
	position[i] =(float) b->get(i).asDouble();
      }
      return 1;
    }
    else{  // many objects are tracked
      for(j=0;j<nb_max;j++){
	ok[j]=0;
      }
      for(j=0;j<nb_el;j++){
	k  = b->get(j*4).asInt(); 
	ok[k] = 1;
	for(int i=0;i<3;i++){
	  position[k*3+i] =(float) b->get(j*4+i+1).asDouble();
	}
      }
      return nb_el;
    }
  }
  else{
    return 0;
  }
#else
 return 0;
  #endif
}

int VisionServer::ReadPosition(double *position){
  #ifdef WITH_YARP
	Bottle *b = read(false);
  if(b){
    for(int i=0;i<3;i++){
      position[i] =b->get(i).asDouble();
    }
    return 1;
  }
  else{
    return 0;
  }
#else
	return 0;
#endif
}

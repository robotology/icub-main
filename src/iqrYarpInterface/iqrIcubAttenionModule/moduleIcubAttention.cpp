#include "moduleIcubAttention.hpp"

#include <unistd.h>
#include <sstream>
#include <iomanip>

#include <Common/Helper/iqrUtils.h>

#include <yarp/os/Network.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


MAKE_MODULE_DLL_INTERFACE(iqrcommon::moduleYarped,"iCub IQR Multimodal Attention Module")

iqrcommon::moduleYarped::moduleYarped() : ClsThreadModule() {
    tid = 0;
  
     bot_up_saliency_left_faces= addOutputToGroup("Bottom Up Att: Faces Left", "Bottom Up Att: Faces Left");
     bot_up_saliency_right_faces= addOutputToGroup("Bottom Up Att: Faces Right", "Bottom Up Att: Faces Right");
     bot_up_saliency_left_torsos= addOutputToGroup("Bottom Up Att: Torsos Left", "Bottom Up Att: Torsos Left");
     bot_up_saliency_right_torsos= addOutputToGroup("Bottom Up Att: Torsos Right", "Bottom Up Att: Torsos Right");
     bot_up_saliency_left_moves= addOutputToGroup("Bottom Up Att: Moves Left", "Bottom Up Att: Moves Left");
     bot_up_saliency_right_moves= addOutputToGroup("Bottom Up Att: Moves Right", "Bottom Up Att: Moves Right");
     
     attentionCmds = addInputFromGroup("Attention Commands", "Attention Commands");	

     
}

iqrcommon::moduleYarped::~moduleYarped(){ 
}

void iqrcommon::moduleYarped::init(){
    Network::init();
    face_left_in.open("/iqr/attention/faces/left/in");
    face_right_in.open("/iqr/attention/faces/right/in");
    torso_left_in.open("/iqr/attention/torsos/left/in");
    torso_right_in.open("/iqr/attention/torsos/right/in");
    move_left_in.open("/iqr/attention/movements/left/in");
    move_right_in.open("/iqr/attention/movements/right/in");
    
    
    attention_out.open("/iqr/attention/commands/out");

    firstTime = true;
    attentionCount = 0;
}


void iqrcommon::moduleYarped::update(){
   
    qmutexThread->lock();

    std::vector<Point> bupPointsLeft; // bottom up saliency points in left image
    std::vector<Point> bupPointsRight; // bottom up saliency points in right image

   
    //----------------------- left cam faces ---------------------------------------------
    
    StateArray &stateArrayIn = bot_up_saliency_left_faces->getStateArray();
    for (int i = 0; i < 32 * 24; i++){
  	stateArrayIn[0][i] = 0.0;
    }
    
    Vector* faces_left = face_left_in.read(false);
    if (faces_left != NULL){
	int sizeLeft = faces_left->size();
     
	for (int i=0; i<sizeLeft; i = i+2){
	    int index_x = (int)((double)(*faces_left)[i] / 10.0);
	    int index_y = (int)((double)(*faces_left)[i + 1] / 10.0);

	
	    Point p; 
	    p.x = (float)(*faces_left)[i];
	    p.y = (float)(*faces_left)[i + 1];
	    bupPointsLeft.push_back(p);

	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);

	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}
	    
	    stateArrayIn[0][index]= 1.0;
	}
     
    }

    //----------------------- left cam movements ---------------------------------------------
    
    StateArray &stateArrayInMl = bot_up_saliency_left_moves->getStateArray();
    for (int i = 0; i < 32 * 24; i++){
  	stateArrayInMl[0][i] = 0.0;
    }
    
    Vector* moves_left = move_left_in.read(false);
    if (moves_left != NULL){
	int sizeLeftm = moves_left->size();
     
	for (int i=0; i<sizeLeftm; i = i+2){
	    int index_x = (int)((double)(*moves_left)[i] / 10.0);
	    int index_y = (int)((double)(*moves_left)[i + 1] / 10.0);

	
	    Point p; 
	    p.x = (float)(*moves_left)[i];
	    p.y = (float)(*moves_left)[i + 1];
	    bupPointsLeft.push_back(p);

	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);

	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}
	    
	    stateArrayInMl[0][index]= 1.0;
	}
     
    }

    
     
     //----------------------- left cam torsos ---------------------------------------------
    //qmutexThread->lock();
    StateArray &stateArrayInTl = bot_up_saliency_left_torsos->getStateArray();
    for (int i = 0; i < 32 * 24; i++){
  	stateArrayInTl[0][i] = 0.0;
    }
  	
    
    
    Vector* torsos_left = torso_left_in.read(false);
    if (torsos_left != NULL ){
	int sizeLeftt = torsos_left->size();
	//printf("sizeLeft = %d\n", sizeLeft);

	for (int i=0; i<sizeLeftt; i = i+2){
	    int index_x = (int)((double)(*torsos_left)[i] / 10.0);
	    int index_y = (int)((double)(*torsos_left)[i + 1] / 10.0);
	    
	    Point p; 
	    p.x = (float)(*torsos_left)[i];
	    p.y = (float)(*torsos_left)[i + 1];
	    bupPointsLeft.push_back(p);
	    
	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);
	    
	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}
	    
	    
	    stateArrayInTl[0][index] = 1.0;
	    
	}
    
    }
     

    //------------------------------ right cam torsos  -----------------------------------------
    
    StateArray &stateArrayInRT = bot_up_saliency_right_torsos->getStateArray();
    
    // zero the state array
    for (int i = 0; i < 32*24; i++){
	stateArrayInRT[0][i] = 0.0;
     }
    
    
    Vector* torsos_right = torso_right_in.read(false);
    if (torsos_right != NULL){
	int sizeRightT = torsos_right->size();
	//printf("sizeLeft = %d\n", sizeLeft);

	for (int i=0; i<sizeRightT; i = i+2){
	    int index_x = (int)((double)(*torsos_right)[i] / 10.0);
	    int index_y = (int)((double)(*torsos_right)[i + 1] / 10.0);
	
	    Point p; 
	    p.x = (float)(*torsos_right)[i];
	    p.y = (float)(*torsos_right)[i + 1];
	    bupPointsRight.push_back(p);

	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);

	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}

	
	    stateArrayInRT[0][index] = 1.0;
	
	}
    
    }
   
    //------------------------------ right cam faces  -----------------------------------------
    
    StateArray &stateArrayInR = bot_up_saliency_right_faces->getStateArray();
    for (int i = 0; i < 32*24; i++){
	stateArrayInR[0][i] = 0.0;
     }
    	
    
    
    Vector* faces_right = face_right_in.read(false);
    if (faces_right != NULL){
	int sizeRight = faces_right->size();

	for (int i=0; i<sizeRight; i = i+2){
	    int index_x = (int)((double)(*faces_right)[i] / 10.0);
	    int index_y = (int)((double)(*faces_right)[i + 1] / 10.0);


	    Point p; 
	    p.x = (float)(*faces_right)[i];
	    p.y = (float)(*faces_right)[i + 1];
	    bupPointsRight.push_back(p);


	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);

	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}
	    
	
	    stateArrayInR[0][index] = 1.0;
	
	}
    }


    //------------------------------ right cam moves  -----------------------------------------
    
    StateArray &stateArrayInRm = bot_up_saliency_right_moves->getStateArray();
    for (int i = 0; i < 32*24; i++){
	stateArrayInRm[0][i] = 0.0;
     }
    	
    
    
    Vector* moves_right = move_right_in.read(false);
    if (moves_right != NULL){
	int sizeRightm = moves_right->size();

	for (int i=0; i<sizeRightm; i = i+2){
	    int index_x = (int)((double)(*moves_right)[i] / 10.0);
	    int index_y = (int)((double)(*moves_right)[i + 1] / 10.0);


	    Point p; 
	    p.x = (float)(*moves_right)[i];
	    p.y = (float)(*moves_right)[i + 1];
	    bupPointsRight.push_back(p);


	    int index = index_y * 32 + index_x;	// we assume here the group is 32*24!!
	    //printf("i: %d, index_x %d, index_y %d, index %d \n:  ", i, index_x, index_y, index);

	    if (index < 0){index = 0;}
	    if (index > 32*24){index = 32*24;}
	    
	
	    stateArrayInRm[0][index] = 1.0;
	
	}
    }








    //-------------------------------- read out from the attention command group and send it out ( connect this to iKinGazeCtrl)
    
    StateArray &stateArrayOutAtt = attentionCmds->getTarget()->getStateArray();
    float xl,xr,yl,yr;
    bool donel = false;
    bool doner = false;
    int i,j;

    //printf("#left points: %d, #right points: %d\n", bupPointsLeft.size(), bupPointsRight.size());
    float bestAct = 0.0;
    int index = -1;
    int tmpindex;

    for (int ii =0; ii < 32; ii++){
	
	for (int jj =0; jj < 24; jj++){
	       tmpindex = jj * 32 + ii;
	    if ((stateArrayOutAtt[0][tmpindex]) > bestAct){
		index = tmpindex;
		bestAct = (stateArrayOutAtt[0][index]);
		i = ii;
		j = jj;
	    }
	}

    }
	    if (bestAct >= 0.5){
		stateArrayOutAtt[0][index] = stateArrayOutAtt[0][index] / 2.0; // inhibition of return
		//printf("attention at: %d, %d, %d\n", i,j,index);
		float attX = i * 10.0;
		float attY = j * 10.0;
		
		// pre requisite: this cell will light up only if there were corresponsind left and right bottom up input
		// find the closest point in the left field
		float bestDist = 99999999999.0;
		
		for (int ii = 0; ii < bupPointsLeft.size(); ii++){
		    float curX = bupPointsLeft.at(ii).x;
		    float curY = bupPointsLeft.at(ii).y;
		    //printf("left: curX, curY = %f, %f\n", curX, curY);
		    float curDist = sqrt((curX - attX)*(curX - attX)+(curY - attY)*(curY - attY));
		    if (curDist < 100.0){
			xl = curX;
			yl = curY;
			donel = true; 

		    }
		    if (donel) break;
		}
		// find the closest point in the right field
		for (int ii = 0; ii < bupPointsRight.size(); ii++){
		    float curX = bupPointsRight.at(ii).x;
		    float curY = bupPointsRight.at(ii).y;
		    float curDist = sqrt((curX - attX)*(curX - attX)+(curY - attY)*(curY - attY));
		    if (curDist < 100.0){
			xr = curX;
			yr = curY;
			doner = true; 
		    }
		    if (doner) break;
		}
		
	   }


    
	    if (donel && doner){firstTime = false;attentionCount = 0;}
	    
    Vector& attOut = attention_out.prepare();
        
        attOut.resize(4);
        attOut[0] = xl;
        attOut[1] = yl;
        attOut[2] = xr;
        attOut[3] = yr;
	//std::cout << "sending out: " << xl << ", " << yl << ", " << xr << ", " << yr << std::endl;

	if ((!firstTime) && (attentionCount < 10)){
	    attention_out.write();
	    attentionCount++;
	}
	//  }
    

    bupPointsLeft.clear();
    bupPointsRight.clear();

    qmutexThread->unlock();

    usleep(1e5);

}

void iqrcommon::moduleYarped::cleanup(){
    Network::fini();
}

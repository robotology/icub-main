// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * author Micha Hersch
 * email:   micha.hersch@robotcub.org
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

#include <ace/OS_NS_stdio.h>
#include "BodySchemaLogger.h"
//#include <ace/OS.h>

bool BodySchemaLogger::open(Searchable& s){

    char fname[80];
    ios_base::openmode opt;
  

//     //loading tree
//     Value& struc = s.find("structure");
//     if(struc.isNull()){
//         ACE_OS::printf("no robot structure file given\n");
//         return false;
//   }
    
//   body = new KinematicTree(struc.asString().c_str());

    body_data_size = 200;

    Value& head_s = s.find("head_size");
    if(head_s.isNull()){
         ACE_OS::printf("No size for head proprioception specified\n");
         return false;
    }
    head_size = head_s.asInt();
    
    Value& arm_s = s.find("arm_size");
    if(arm_s.isNull()){
        ACE_OS::printf("No size for arm proprioception specified\n");
        return false;
    }
    arm_size = arm_s.asInt();
 
    
    body_data = new float[body_data_size];    
    head_position = new float[head_size];
    arm_position = new float[arm_size];


    Value& dir = s.find("directory");
    if(dir.isNull()){
          ACE_OS::printf("No directory specified\n");
          return false;
    }

    if(s.check("binary")){
        binary = true;
        opt = ios_base::out | ios_base::binary;
    }
    else{
        binary = false;
        opt = ios_base::out;
    }

    cout<<"opening files "<<endl;
    //opening log files
    sprintf(fname,"%s/%s",dir.asString().c_str(),"body_schema_log");
    body_schema_file.open(fname,opt);
    if(body_schema_file.fail()){    
        ACE_OS::printf("Cannot open file %s\n",fname);
          return false;
    }

    sprintf(fname,"%s/%s",dir.asString().c_str(),"head_log");
    head_file.open(fname,opt);
    if(head_file.fail()){    
        ACE_OS::printf("Cannot open file %s\n",fname);
          return false;
    }

    sprintf(fname,"%s/%s",dir.asString().c_str(),"arm_log");
    arm_file.open(fname,opt);
    if(arm_file.fail()){    
        ACE_OS::printf("Cannot open file %s\n",fname);
          return false;
    }

    sprintf(fname,"%s/%s",dir.asString().c_str(),"vision_log");
    vision_file.open(fname,opt);
    if(vision_file.fail()){    
        ACE_OS::printf("Cannot open file %s\n",fname);
          return false;
    }

    sprintf(fname,"%s/%s",dir.asString().c_str(),"visual_rotation_log");
    visual_rotation_file.open(fname,opt);
    if(visual_rotation_file.fail()){    
        ACE_OS::printf("Cannot open file %s\n",fname);
          return false;
    }

    cout<<"opening ports "<<endl;
    //opening ports

    if(! body_schema_port.open(getName("body_schema:i").c_str())){
        ACE_OS::printf("Cannot open port %s\n",getName("body_schema:i").c_str());
      return false;
  }
    if(! head_port.open(getName("proprio_head:i").c_str())){
       ACE_OS::printf("Cannot open port %s\n",getName("proprio_head:i").c_str());
      return false;
  }

    if(! arm_port.open(getName("proprio_arm:i").c_str())){
        ACE_OS::printf("Cannot open port %s\n",getName("proprio_arm:i").c_str());
        return false;
    }

   if(! vision_port.open(getName("vision:i").c_str())){
       ACE_OS::printf("Cannot open port %s\n",getName("vision:i").c_str());
      return false;
   }

   if(! visual_rotation_port.open(getName("visual_rotation:i").c_str())){
       ACE_OS::printf("Cannot open port %s\n",getName("visual_rotation:i").c_str());
      return false;
   }

  //Opening the communication ports
   if(!cmd.open(getName("cmd:i").c_str())){
       ACE_OS::printf("Cannot open port %s\n",getName("cmd:i").c_str());
       return false;
   }

   return true;
}

bool BodySchemaLogger::close(){
    //closing files
    body_schema_file.close();
    head_file.close();
    arm_file.close();
    vision_file.close();
    visual_rotation_file.close();

    //closing ports
    body_schema_port.close();
    head_port.close();
    arm_port.close();
    vision_port.close();
    visual_rotation_port.close();

    //releasing memory
    delete[] body_data;
    delete[] head_position;
    delete[] arm_position;
    
    return true;
}


void BodySchemaLogger::putInFile(ofstream& out,const float *data, int size){
    if(binary){
        out.write((char *)data, size*sizeof(float));
    }
    else{
        for(int i=0;i<size;i++){
            out<<data[i]<<" ";
        }
        out<<endl;
    }
}


bool BodySchemaLogger::updateModule(){    
    Stamp stamp;
    if(cmd.ShouldQuit()){return false;}
    if(body_schema_port.ReadBodySchema(body_data,body_data_size)){
        if(body_schema_port.GetSize()<=body_data_size){
            putInFile(body_schema_file,body_data,body_schema_port.GetSize());
        }
        else{
            body_data_size =body_schema_port.GetSize();
            delete [] body_data;
            body_data = new float[body_data_size];
        } 
    }
    if(head_port.ReadPosition(head_position+1,head_size-1)){
        head_port.getEnvelope(stamp);
        head_position[0] = (float) stamp.getTime();
        putInFile(head_file,head_position,head_size);
    }

    if(arm_port.ReadPosition(arm_position+1,arm_size-1)){
        arm_port.getEnvelope(stamp);
        arm_position[0] = (float) stamp.getTime();
        putInFile(arm_file,arm_position,arm_size);
    }
    
    if(vision_port.ReadPosition(vision+1)){
        vision_port.getEnvelope(stamp);
        vision[0] = (float) stamp.getTime();
        putInFile(vision_file,vision,4);
    }

    double t0,t1;
    if(visual_rotation_port.ReadRotation(visual_rotation+2,&t0,&t1 )){
        visual_rotation[0] = (float)t0;
        visual_rotation[1] = (float)t1;        
        putInFile(visual_rotation_file,visual_rotation,5);
    }

    
    return true;
}

int main(int argc, char *argv[]){
    BodySchemaLogger module;
    module.runModule(argc,argv);
    return 0;
}

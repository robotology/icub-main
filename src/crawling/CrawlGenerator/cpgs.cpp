/*  * Copyright (C) 2008 sarah dégallier BIRG, EPFL, Lausanne
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sarah.degallier@robotcub.org
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

#include "cpgs.h"
#include <iostream>
#include <ace/OS.h>
using namespace std;



cpgs::cpgs(int nbDOFs, int nbLIMBs){

  this->nbDOFs = nbDOFs;
  this->nbLIMBs = nbLIMBs;

  cpgs_size = 4*nbDOFs + 1 + 2*nbLIMBs;
  controlled_param = 2*nbDOFs;

  //we initialize coupling strength and phase angle and set everything to 0
  epsilon = new double*[nbDOFs];
  theta = new double*[nbDOFs];
  theta_init = new double*[nbDOFs];

    for(int i =0;i<nbDOFs;i++)
    {
        epsilon[i] = new double[nbDOFs];
        theta[i] = new double[nbDOFs];
        theta_init[i]= new double[nbDOFs];

        for(int j=0;j<nbDOFs;j++)
	    {
            epsilon[i][j] = 0.0;
		    theta[i][j] = 0.0;
		    theta_init[i][j] =0.0;
	    }
    }
    
  next_external_coupling = new double [nbLIMBs];
  external_coupling=new double [nbLIMBs];  
  
  for(int i=0;i>nbLIMBs;i++)
  {
    next_external_coupling[i] = 0.0;
    external_coupling[i]= 0.0; 
      
  }
    
    
  //open parameters
  m = new double [nbDOFs];//amplitudes
  g = new double [nbDOFs];//target angles
 
  //radii
  r = new double [nbDOFs+nbLIMBs];
  dydt = new double [cpgs_size];
  for(int i=0; i<cpgs_size; i++)
    {
      dydt[i]=0.0;
    }

  //*********open parameters**********

  parameters = new double[controlled_param];
  //next_parameters = new double[controlled_param];
  
  for(int i=0; i< nbDOFs; i++)
    {
      parameters[2*i]=-1.0; // amplitude of oscillations
      parameters[2*i+1]=0.0; //target of the discrete movement
    }

  //********fixed parameters********* 

  //equation
  a = 2.0; //rate of convergence of the rhythmic system
  b = 5.0;//rate of convergence of the discrete system
  m_off = -5.0; //value to turn of the oscillations 
  m_on = 1.0; // value to turn on the oscillations
  b_go = 0.5; //rate of conv. of the go command (for the discrete system)
  u_go = 4.0; //max value of the go command
  dt = 0.001; //integration step in seconds
  c = 100.0;//parameter of for the swing/stance switching

  //frequency and amplitude
  om_stance=0.2; //in Hz
  //next_om_stance = 0.25;
  om_swing = om_stance;
  //next_om_swing = om_stance;

  ampl = new double[nbDOFs];
  for(int i=0;i<nbDOFs;i++)
    ampl[i]=0.1;
    
  //turning angle (torso roll joint, used to compute amplitudes)  
  turnAngle = 0;
  
  //feedback auxiliary variables  
  feedbackable=0; 
  feedback_on=0; 
  contact[0]=0.0;
  contact[1]=0.0;
}



cpgs::~cpgs()

{
  delete[] parameters;

  for(int i=0;i<nbDOFs;i++)
    {
      delete[] epsilon[i];
      delete[] theta[i];
      delete[] theta_init[i];
    }

  delete[] epsilon;
  delete[] theta;
  delete[] theta_init;
  delete[] m;
  delete[] g;
  delete[] dydt;
  delete[] r;
  delete[] ampl;
  
  delete[] next_external_coupling;
  delete[] external_coupling;  
}

void cpgs::integrate_step(double *y, double *at_states)
{

  //************ getting open parameters******************
  
   for(int i=0; i<nbDOFs;i++)
   {
      m[i]= parameters[2*i];
      g[i]= parameters[2*i+1]/ampl[i];
   }

    if(m[0]>0.0)
    {
        if(partName=="left_arm" || partName=="right_arm")
        {
            if(y[3]<0.0)
            {
                g[3] -= 1.5*y[3];
                g[1] -= y[3];
            }
        
      //g[3] += (40.0*exp(-4.0*(y[3]+1.0)*(y[3]+1.0)))/180*3.14/ampl[3];
       //g[1] += (15.0*exp(-4.0*(y[3]+1.0)*(y[3]+1.0)))/180*3.14/ampl[1];
        }

    if(partName=="left_leg" || partName=="right_leg")
        {
            if(y[3]<0.0)
            {
                g[1] -= 2.5*y[3];//(30.0*exp(-4.0*(y[3]+1.0)*(y[3]+1.0)))/180*3.14/ampl[1];
            }
        }
    }




  //***********CPGS - EQUATIONS***************************************
   for(int i=0;i<nbDOFs;i++)
     {
       r[i]=(y[2+i*4]-y[i*4])*(y[2+i*4]-y[i*4])+y[3+i*4]*y[3+i*4]; 
     }
   for(int i=0;i<nbLIMBs;i++)
     r[nbDOFs+i]=y[4*nbDOFs+2*i]*y[4*nbDOFs+2*i] + y[4*nbDOFs+2*i+1]*y[4*nbDOFs+2*i+1];

   ///go Command
   dydt[cpgs_size-1]=b_go*(u_go-y[cpgs_size-1]);


  //***********JOINTS***********************************
   double omega = 2*3.14*(om_swing/(1+exp(-c*y[3])) + om_stance/(1+exp(c*y[3])));

  ////internal dynamics
  for(int i =0;i<nbDOFs;i++)
    {
      ///discrete system
      dydt[i*4] = y[cpgs_size-1]*y[cpgs_size-1]*y[cpgs_size-1]*y[cpgs_size-1]*y[i*4+1];
      dydt[i*4+1] = u_go*u_go*u_go*u_go*b * (b/4.0 * (g[i] - y[i*4]) - y[i*4+1]);

	   //rhythmic one
      dydt[i*4+2] = a * (m[i]-r[i]) * (y[i*4+2]-y[i*4]) - omega * y[i*4+3];
      dydt[i*4+3] = a * (m[i]-r[i]) * y[i*4+3] + omega* (y[i*4+2]-y[i*4]);
   }

  ///internal couplings
  for(int i = 0;i<nbDOFs;i++)
    for(int j=0;j<nbDOFs;j++)
    {	    
        dydt[i*4+2] += epsilon[i][j]*(cos(theta[i][j])*(y[4*j+2]-y[4*j]) - sin(theta[i][j])*y[4*j+3]);
	    dydt[i*4+3] +=  epsilon[i][j]*(sin(theta[i][j])*(y[4*j+2]-y[4*j]) + cos(theta[i][j])*y[4*j+3]);
    }

  ///external coupling
  for(int i=0;i<nbLIMBs;i++)
    dydt[3] += external_coupling[i]*y[4*nbDOFs+2*i+1];

  //*********Other limbs**********/////////////
  for(int i=0;i<nbLIMBs;i++)
    {
      int index = 4*nbDOFs + 2*i;
      omega = 2*3.14*(om_stance/(1+exp(c*y[index+1])) + om_swing/(1+exp(-c*y[index+1])));
      dydt[index] = a * (m[0]-r[nbDOFs+i]) * y[index] - omega*y[index+1];
      dydt[index+1] = a * (m[0]-r[nbDOFs+i]) * y[index+1] + omega*y[index];
    }
    
 //*****************feedback*****************////
   //if(feedback_on)
    //{
        //double gain = 200.0;

        ////switch faster
        //if(y[3] < 0.0 && (y[2]-y[0]) > 0.95 && contact[0] > 20.0) //modified! still swing and contact
            //dydt[3] += gain;
	    
        //if(y[3] > 0.0 && (y[2]-y[0]) < -0.95 && contact[0] < 20.0)//modified! still stance no contact
        //{ 
            //dydt[3] -= gain;
            //if(partName=="left_arm" || partName=="right_arm")
            //{
                //g[3] += y[3];
                //g[1] -= y[3];
            //}
            //if(partName=="left_leg" || partName=="right_leg")
            //{
                //g[1] += 1.5*y[3];//(30.0*exp(-4.0*(y[3]+1.0)*(y[3]+1.0)))/180*3.14/ampl[1];
            //}
        //}
            
        ////switch slower
        //if(y[3] < 0.0 && (y[2]-y[0]) > 0.95 && contact[0] < 20.0) //switching to stance but no contact
        //{
            //dydt[3] -= omega*(y[2]-y[0]);
            //for(int i=0;i<nbLIMBs;i++)
                //dydt[3] -= external_coupling[i]*y[4*nbDOFs+2*i+1];
        //}
            
        //if(y[3] > 0.0 && y[2] < -0.95 && contact[1] < 0.8*contact[0]) //switching to swing but still contact
        //{
            //dydt[3] -= omega*(y[2]-y[0]);
            //for(int i=0;i<nbLIMBs;i++)
                //dydt[3] -= external_coupling[i]*y[4*nbDOFs+2*i+1];
        //}
    //}

  

  
  //********INTEGRATION****************************************
  for(int i=0; i<cpgs_size; i++)
    y[i]=y[i]+dydt[i]*dt;
  

  //***** SETTING TARGET POSITION
  for(int i=0;i<nbDOFs;i++)
    at_states[i]= ampl[i]*180.0/M_PI*y[4*i+2];
}


void cpgs::printInternalVariables()
{

  ACE_OS::printf("nbDOFs %d, cpgs_size %d, controlled param %d\n",
		 nbDOFs,cpgs_size,controlled_param);
  ACE_OS::printf("a %f, b %f, m_off %f, m_on %f, b_go %f, u_go %f, dt %f\n",
		 a,b,m_off,m_on,b_go,u_go,dt);
  ACE_OS::printf("omStance %f, omSwing %f\n",om_stance,om_swing);

 for(int i=0;i<nbDOFs;i++)
   {
     ACE_OS::printf("for DOF %d, mu=%f and g=%f - ampl=%f\n",i,parameters[2*i],parameters[2*i+1],ampl[i]);
     ACE_OS::printf("coupling strength");

     for(int j=0;j<nbDOFs;j++)
       ACE_OS::printf(" - %f",epsilon[i][j]); 

     ACE_OS::printf("\n");
     ACE_OS::printf("phase diff");

     for(int j=0;j<nbDOFs;j++)
       ACE_OS::printf(" - %f",theta[i][j]); 

     ACE_OS::printf("\n");
   }

   ACE_OS::printf("next external coupling");

     for(int j=0;j<nbLIMBs;j++)
       ACE_OS::printf(" - %f",next_external_coupling[j]); 

     ACE_OS::printf("\n");
     
    ACE_OS::printf("external coupling");

     for(int j=0;j<nbLIMBs;j++)
       ACE_OS::printf(" - %f", external_coupling[j]); 

     ACE_OS::printf("\n");

}


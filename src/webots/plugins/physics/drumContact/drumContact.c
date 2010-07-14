/*
 * File:         flying_mybot.c
 * Date:         August 15th, 2006
 * Description:  An example of use of a custom ODE physics
 *               plugin.
 * Author:       Simon Blanchoud
 * Modifications:
 *
 * Copyright (c) 2006 Cyberbotics - www.cyberbotics.com
 */

#include <stdlib.h>
#include <plugins/physics.h>
#include <stdio.h>


#define NB_INFO_BUFFER 3
#define DEFAULT_VELOCITY 1


/* The general ODE variables, always useful. */
static dWorldID world;
static dSpaceID space;
static dJointGroupID contact_joint_group;

/* The Geoms used in our plugin. */
static dGeomID left_stick_geom;
static dGeomID right_stick_geom;
static dGeomID drum_l_geom;
static dGeomID drum_ml_geom;
static dGeomID drum_mr_geom;
static dGeomID drum_r_geom;
static dGeomID ground_geom;

// buffer d'envoi a superviseur
static float buffer[2*NB_INFO_BUFFER];

// variables de detection de collision
static int left_etat = 0;
static int left_collide = 0;

static int right_etat = 0;
static int right_collide = 0;


/*
 * This function is called every time ODE is started, this is a good place to
 * initialize the variables and to create the objects we want. It is also
 * useful to store the general ODE variables.
 */
void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j)
{
    world = w;
    space = s;
    contact_joint_group = j;

    dWebotsConsolePrintf("Init: "
                         "dWorldID = %p - dSpaceID = %p - "
                         "dContactJointGroupID = %p\n", w, s, j);

    /*
     * Here we get all the geoms associated with DEFs in Webots. 
     * A Geom corresponds to the boundingObject node of the object specified
     * by the DEF name.
     * That is why you can retreive here only DEFs of nodes which contains a
     * boundingObject node.
     */
 
    left_stick_geom = dWebotsGetGeomFromDEF("LEFT_STICK");
    right_stick_geom = dWebotsGetGeomFromDEF("RIGHT_STICK");
 
    drum_l_geom = dWebotsGetGeomFromDEF("left_drum");
    drum_ml_geom = dWebotsGetGeomFromDEF("middle_left_drum");
    drum_mr_geom = dWebotsGetGeomFromDEF("middle_right_drum");
    drum_r_geom = dWebotsGetGeomFromDEF("right_drum");
 
    ground_geom = dWebotsGetGeomFromDEF("GROUND");
}


void webots_physics_step()
{

/*
SYNTAXE MESSAGE

Bras | Instrument | Velocity


Bras : LEFT (0) | RIGHT (1)
Instrument : NO_CONTACT (0) | SMALL_DRUM (1) | CENTRAL_DRUM (2) | CYMBAL (3)
Velocity : not used yet, always 0

*/


/*
4 CAS POSSIBLE POUR CHAQUE MAIN

(1)
etat == 0
collide == 0
-------------------> pas de collision en cours, pas de collision ce step
------------------------> pas de modification
------------------------> pas de message

(2)
etat == 0
collide != 0
-------------------> pas de collision en cours, mais nouvelle collision
------------------------> etat = collide (n° instrument)
------------------------> message

(3)
etat != 0
collide == 0
-------------------> collision en cours, mais fin collision
------------------------> etat = collide (0)
------------------------> message

(4)
etat != 0
collide != 0
-------------------> collision en cours et collision ce step
------------------------> test integrité : si etat != collide => exit(EXIT_FAILURE)
------------------------> pas de message
*/


  int i;
  int message = 0;

  /*
  on met le buffer message a 0
  */
  for(i=0; i<2*NB_INFO_BUFFER; i++)
  {
    buffer[i] = 0;
  }


  /* LEFT HAND */
  if((left_etat == 0) && (left_collide == 0))
  {
    /* on ne fait rien */   
  }
  else if((left_etat == 0) && (left_collide != 0))
  {
    // nouvelle collision
    buffer[0] = 1;
    
    // NOUVEL instrument
    buffer[1] = left_collide;
    
    // velocity
    buffer[2] = DEFAULT_VELOCITY;
    
    // un message doit etre envoyé
    message = 1;
    
    // on stock le nouvel etat
    left_etat = left_collide;
    
  }
  else if((left_etat != 0) && (left_collide == 0))
  {
    // fin collision
    buffer[0] = 2;
    
    // ANCIEN instrument
    buffer[1] = left_etat;
    
    // velocity
    buffer[2] = DEFAULT_VELOCITY;

    // on stock le nouvel etat
    left_etat = left_collide;
    
  }
  else if((left_etat != 0) && (left_collide != 0))
  {
    
    if(left_etat != left_collide){
      exit(EXIT_FAILURE);
    }
  }

  if((right_etat == 0) && (right_collide == 0))
  {
    /* on ne fait rien */
    
  }
  else if((right_etat == 0) && (right_collide != 0))
  {
    
    // nouvelle collision
    buffer[3] = 1;
    
    // NOUVEL instrument
    buffer[4] = right_collide;
    
    // velocity
    buffer[5] = DEFAULT_VELOCITY;
    
    // un message doit etre envoyé
    message = 1;
    
    // on stock le nouvel etat
    right_etat = right_collide;
    
  }
  else if((right_etat != 0) && (right_collide == 0))
  {
    
    // fin collision
    buffer[3] = 2;
    
    // ANCIEN instrument
    buffer[4] = right_etat;
    
    // velocity
    buffer[5] = DEFAULT_VELOCITY;

    
    // on stock le nouvel etat
    right_etat = right_collide;
  }
  else if((right_etat != 0) && (right_collide != 0))
  {    
    if(right_etat != right_collide)
    {
      exit(EXIT_FAILURE);
    }
  }

  /* on se prepare pour le step prochain */
  left_collide = 0;
  right_collide = 0;

  if(message)
  {
    dWebotsSend(0, (void*) buffer, 2*NB_INFO_BUFFER*sizeof(float));
    //printf("Sending message\n");
  }
  
}

int webots_physics_collide(dGeomID g1, dGeomID g2)
{
    // on commence par tester les collisions de la main gauche
    
    
    if( (g1 == left_stick_geom && g2 == drum_l_geom) 
        || (g1 == left_stick_geom && g2 == drum_ml_geom) 
        || (g2 == left_stick_geom && g1 == drum_l_geom) 
        || (g2 == left_stick_geom && g1 == drum_ml_geom) )  

    {
        //printf("collide >> Collisiton detected on the left\n");
        left_collide = 1;
    }

    if( (g1 == right_stick_geom && g2 == drum_r_geom) 
        || (g1 == right_stick_geom && g2 == drum_mr_geom) 
        || (g2 == right_stick_geom && g1 == drum_r_geom) 
        || (g2 == right_stick_geom && g1 == drum_mr_geom) )  

    {
        //printf("collide >> Collisiton detected on the right\n");
        right_collide = 1;
    }


    return 0; //collision handled by Webots
    
}

/* 
 * This function is called every time the World is destroyed or restarted. It
 * is mandatory to destroy here everything you have created. If you do not do
 * it, the stability of Webots cannot be guaranteed.
 */
void webots_physics_cleanup()
{
    printf("Clean up\n");
}

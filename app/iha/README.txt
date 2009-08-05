==========================================
Interaction History Architecture
==========================================

GENERAL NOTES ON USAGE
======================
  1.  To start and stop the system you only really need to use the "numbered scripts"
      e.g. 10_xxx, 20_xxx
      Their number gives the ideal order they would be executed in when starting and
      subsequently stopping the system

  2.  To run different processes on different machines, change the processor map in 
      config_machines.sh

  3.  The recommended way to stop individual processes is to use the modular scripts
      e.g. facedetector.sh stop
	  In extreme cases there is also usually a kill option. 

  4.  There are two config scripts containing variables. The first is the machine 
      mapping script config_machines.sh, the other, config_ports.sh, holds all the port 
	  names so things can be connected easily in different scripts.

SCRIPT STRUCTURE
================

start_stop_utils.sh
-------------------
  This script provides functions to start and stop individual modules or processes 
  on the local or remote machines. 

  You can choose to start locally or remotely, with logging or without and also you can 
  choose to start the process in its own spawned xterm

Init scripts
------------
  These scripts are named for the processes the look after, e.g. viewers.sh
They always take a parameter e.g. [start|stop|kill|connect]


High-level scripts
------------------
  These just simplify the process of starting and stopping the system as you 
can just start the scripts in order of number (and stop in order) without having to know
or remember what should be started first.  They consist of lists of init scripts.


======================================================================================
Credits
-------
The structure of these scripts is based on ones by Jonas Reuch and Lorenzo Natale
and have been modified extensively (especially for remoting) by Assif Mirza

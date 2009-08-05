iCub Head

These are the steps to do for starting up the icubHead

1. Connect the head to a power supply (12V and 10A). Leave switched off the power supply.
2. Connect the laptop and the Ethernet cable of the head in a switch. 
3. Switch on the laptop. UserName: icub Password: icub
4. Once the laptop is running, switch on the power supply of the head and wait about 1 minute for the bootstrap of the pc104.
5. In /usr/local/src/robot/iCub/app/default/scripts/ run ./icub-cluster.sh start. It start the yarp server, etc.
6. In /usr/local/src/robot/iCub/app/default/scripts/ ./cameras start
7. Open an ssh connection to the pc104. There is an icon in the desktop bar with a penguin or: ssh icub@pc104 
8. Run: iCubInterface --  config /usr/local/src/robot/iCub/app/iCubAbudhabi01/conf/icubHead.ini
9. Open a shell on the laptop and in the folder /usr/local/src/robot/iCub/app/iCubAbudhabi01/conf/  run robotMotorgui  
10. 



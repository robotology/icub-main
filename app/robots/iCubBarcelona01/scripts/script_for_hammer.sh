
#/bin/bash
iCub_SIM &
sleep 10
iKinGazeCtrl --from configSim.ini &
sleep 10
objectsPropertiesCollector --name OPC --empty &
simCartesianControl &
iKinCartesianSolver --context simCartesianControl/conf --part right_arm &
HammerGUI --from hammergui.ini.sim &
sleep 10
HAMMER4iCub --from hammer4icub.ini.sim &
sleep 10
HammerResponseExample 

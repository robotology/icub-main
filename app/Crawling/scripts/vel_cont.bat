echo off

set ROBOT=icub

start  velocityControl.exe --period 50 --part right_arm --robot %ROBOT%
start  velocityControl.exe --period 50 --part left_arm --robot %ROBOT%
start  velocityControl.exe --period 50 --part right_leg --robot %ROBOT%
start  velocityControl.exe --period 50 --part left_leg --robot %ROBOT%
start  velocityControl.exe --period 50 --part head --robot %ROBOT%
start  velocityControl.exe --period 50 --part torso --robot %ROBOT%


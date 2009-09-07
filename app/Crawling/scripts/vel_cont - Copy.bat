echo off

set ROBOT=icub

start /B velocityControl.exe --period 50 --part right_arm --robot %ROBOT%
start /B velocityControl.exe --period 50 --part left_arm --robot %ROBOT%
start /B velocityControl.exe --period 50 --part right_leg --robot %ROBOT%
start /B velocityControl.exe --period 50 --part left_leg --robot %ROBOT%
start /B velocityControl.exe --period 50 --part head --robot %ROBOT%
start /B velocityControl.exe --period 50 --part torso --robot %ROBOT%


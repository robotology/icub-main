start  "LeftImage" cmd /c yarpdev --device static_grabber --filename scene1.row3.col5.ppm --name /leftimage
start  "RightImage" cmd /c yarpdev --device static_grabber --filename scene1.row3.col1.ppm --name /rightimage

start yarpview /leftview
start yarpview /rightview



yarp wait /leftimage
yarp wait /leftview
yarp connect /leftimage /leftview shmem

yarp wait /rightimage
yarp wait /rightview
yarp connect /rightimage /rightview shmem

start "LogPolarGlobalDisparity" cmd /c debug\logpolarglobaldisparity --file logpolarglobaldisparity.ini

yarp wait /disparity/left:i
yarp wait /disparity/right:i
yarp connect /leftimage /disparity/left:i
yarp connect /rightimage /disparity/right:i

start yarpview /blended
yarp wait /blended
yarp wait /disparity/img:o
yarp connect /disparity/img:o /blended

rem start "Disparities" cmd /c yarp read /r
rem yarp wait /r
rem yarp wait /disparity/disp:o
rem yarp connect /disparity/disp:o /r


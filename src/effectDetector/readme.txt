This filter uses the OpenCV camShift algorithm for tracking an object on the image in the YARP iCub Software Architecture.

Input/Output ports
1 - /effectDetector/init //receives a bottle containing information on initial ROI, histogram and bounds, sends 0 when initialization fails, 1 when it succeeds.

Input ports
1 - /effectDetector/rawSegmImg:i //image on which the segmentation was performed
2 - /effectDetector/rawCurrImg:i //flow of input images

The outputs of the filter are:
1 - /effectDetector/effect:o  //stream of (u,v) positions of the tracked object


Parameters:
none so far.
one should be the value of the threshold.


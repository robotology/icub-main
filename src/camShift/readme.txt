This filter implements the OpenCV camshift algorithm for the YARP iCub Software Architecture.

The inputs of the filter are 
1 - /camshift/img/i (a color image)		
2 - /camshift/roi/i (a input region of interest, in a bottle  x,y,w,h) 

The outputs of the filter are:
1 - /camshift/obj/o  (array with parameters of the containing ellipse 
    centerx,centery, width, height, orientation)
2 - /camshift/roi/o  (search window)
3 - /camshift/img/o  (backprojected image)


Parameters:

--file <campars.ini> // file with internal camera parameters
--angle              // to output azimuth-elevation [degrees]

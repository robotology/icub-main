yarp connect /icub/cam/right /camcalib/image mcast
yarp connect /camcalib/image /camshiftplus/img/i
yarp connect /camcalib/image /artracker/debimage
yarp connect /artracker/debimage /yarpview/i:img
yarp connect  /camshiftplus/all/o /demoAff/objectinfo
yarp connect  /artracker/debout /demoAff/marks 
yarp connect  /demoAff/synccamshift /camshiftplus/roi/i

yarp disconnect /icub/cam/right /camcalib/image mcast
yarp disconnect /camcalib/image /camshiftplus/img/i
yarp disconnect /camcalib/image /artracker/debimage
yarp disconnect /artracker/debimage /yarpview/i:img
yarp disconnect  /camshiftplus/all/o /demoAff/objectinfo
yarp disconnect  /artracker/debout /demoAff/marks 
yarp disconnect  /demoAff/synccamshift /camshiftplus/roi/i

# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# fingersTuner.thrift

struct Value { }
(
  yarp.name = "yarp::os::Value"
  yarp.includefile="yarp/os/Value.h"
)

/**
* fingersTuner_IDL
*
* IDL Interface to \ref fingersTuner services.
*/
service fingersTuner_IDL
{
  /**
  * Synchronize PID values with values stored
  * in the configuration file.
  * @param part specifies the part name as per
  * configuration file.
  * @param val accounts for a single joint if
  * the corresponding integer is given or a set
  * of joints if the corresponding alias is provided
  * as defined within the configuration file.
  * @return true/false on success/failure.
  */
  bool sync(1:string part, 2:Value val);

  /**
  * Tune PID of a joint or a set of joints.
  * @param part specifies the part name as per
  * configuration file.
  * @param val accounts for a single joint if
  * the corresponding integer is given or a set
  * of joints if the corresponding alias is provided
  * as defined within the configuration file.
  * @return true/false on success/failure.
  */
  bool tune(1:string part, 2:Value val);
  
  /**
  * Save the PID parameters on configuration file.
  * @return true/false on success/failure.
  */
  bool save();  

  /**
  * Quit the module.
  * @return true/false on success/failure.
  */
  bool quit();  
}

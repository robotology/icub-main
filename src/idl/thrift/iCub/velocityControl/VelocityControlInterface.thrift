/// 
namespace yarp iCub


/**
*  VelocityControlInterface
*
*/

/// @cond
service VelocityControlInterface
{
 /**
 * Suspend the controller (command zero velocity)
 *  @return true if command was accepted
 */
  bool susp();
 /**
 * Start (and resume after being suspended) the controller
 *  @return true if command was accepted
 */
  bool run();
 /**
 * Quit the module (exit)
 *  @return true if command was accepted
 */
  bool quit();
 /**
 * move joint j to p (degrees)
 *  @return true if command was accepted
 */
 bool set_(1:i32 j, 2:double p);
 /**
 * Set maximum speed for joint j to v (deg/sec)
 *  @return true if command was accepted
 */
 bool svel(1:i32 j, 2:double v);
 /**
 * Set gain for joint j to k
 *  @return true if command was accepted
 */
 bool gain(1:i32 j, 2:double k);
}
/// @endcond

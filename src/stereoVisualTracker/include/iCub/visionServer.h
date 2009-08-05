#ifndef __VISIONSERVER_H__
#define __VISIONSERVER_H__
#endif



#ifdef WITH_YARP
#include "yarp/os/all.h"

using namespace yarp::os;
#endif

/**
 * @brief the yarp communication interface for sending and recieving
 * stereovision data. The connection between two VisionServer is done
 * offline with yarp connect.
 */

#ifdef WITH_YARP
class VisionServer:public BufferedPort<Bottle>{
#else
class VisionServer{
#endif

 protected:
  //  bool isConnected; 
  int registered;
  char portname[100];


 public: 
  /** 
   * Constructor
   */ 
  VisionServer();
  /** 
   * Constructor
   * @param portname the name of the yarp port to open 
  */ 
 VisionServer(const char *portname);
  /** 
   * Destructor
   */ 
  ~VisionServer();

  /**
   * @brief opens the port
   */
  bool Start();

  /**
   * @brief closes the port
   */
  void Stop();

  /**
   * Sends a position over the network
   */
  void SendPosition(double x,double y,double z);

  /**
   * Sends many positions over the network
   * @param pos an array of positions, first
   * three position correspond to the first object, and so on
   * @param nb the size of the array divided by three
   * @param valid an array of size <nb_max> indicating which positions are valid
  */
  void SendPosition(const double *pos, const int *valid, int nb);
   /**
   * Reads a position from the network
   */
 int ReadPosition(float *position);
   /**
   * Reads a position from the network
   */
  int ReadPosition(double *position);
    /**
   * Reads many positions over the network
   * @param position an array of positions, first
   * three position correspond to the first object, and so on
   * @param nb_max the size of the array divided by three
   * @param ok an array of size <nb_max> indicating which positions are valid
    */
int ReadPosition(float *position,int nb_max,int ok[]);

/**
 * @brief Sets the port name. Can only be called before the port is opened
 */
  int SetPortName(const char *portname);
};

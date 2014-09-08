// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_depth2kin_IDL
#define YARP_THRIFT_GENERATOR_depth2kin_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <PointReq.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

class depth2kin_IDL;


/**
 * depth2kin_IDL
 * IDL Interface to \ref depth2kin services.
 */
class depth2kin_IDL : public yarp::os::Wire {
public:
  depth2kin_IDL() { yarp().setOwner(*this); }
/**
 * Return the number of available experts.
 * @return the number of available experts.
 */
  virtual int32_t getNumExperts();
/**
 * Clear the list of currently available experts.
 * @return true/false on success/failure.
 */
  virtual bool clearExperts();
/**
 * Reload the list of experts stored within the
 * configuration file.
 * @return true/false on success/failure.
 */
  virtual bool load();
/**
 * Save the current list of experts into the
 * configuration file.
 * @return true/false on success/failure.
 */
  virtual bool save();
/**
 * Store on file the log of system response computed
 * out of the explored set of input-output pairs.
 * @param type can be "experts" or "calibrator", accounting either
 * for the response of mixture of available experts or the output
 * of the current calibrator, respectively.
 * @return true/false on success/failure. It returns false also if
 * "calibrator" is selected and calibration has not been performed yet.
 * @note Each row of the file will contain the following data: \n
 * \f$ d_x d_y d_z k_x k_y k_z r_x r_y r_z e, \f$ where \f$ d \f$
 * is the depth point, \f$ k \f$ is the kinematic point, \f$ r \f$
 * is the system response and \f$ e=|k-r| \f$ is the error.
 */
  virtual bool log(const std::string& type);
/**
 * Start the exploration phase.
 * @return true/false on success/failure.
 */
  virtual bool explore();
/**
 * Yield an asynchronous stop of the exploration phase.
 * @return true/false on success/failure.
 */
  virtual bool stop();
/**
 * Set the maximum allowed distance between the depth point and
 * kinematic prediction to enable data collection.
 * @param max_dist the value in meters.
 * @return true/false on success/failure.
 */
  virtual bool setMaxDist(const double max_dist);
/**
 * Return the maximum allowed distance between depth point and
 * kinematic prediction to enable data collection.
 * @return the distance.
 */
  virtual double getMaxDist();
/**
 * Set the side of the squared window used to filter data
 * collection in the image plane.
 * @param side the length of the window side.
 * @return true/false on success/failure.
 */
  virtual bool setRoi(const int32_t side);
/**
 * Return the side of the squared window used to filter data
 * collection in the image plane.
 * @return the window side.
 */
  virtual int32_t getRoi();
/**
 * Set the vergence angle used to keep the gaze fixed.
 * @param block_eyes the value in degrees of the vergence. It must
 * be equal or greater than the minimum vergence angle allowed
 * by the gaze controller.
 * @return true/false on success/failure.
 */
  virtual bool setBlockEyes(const double block_eyes);
/**
 * Return the current angle to keep the vergence at.
 * @return the vergence angle in degrees.
 */
  virtual double getBlockEyes();
/**
 * Tell the gaze to immediately steer the eyes to the stored
 * vergence angle and stay still.
 * @return true/false on success/failure.
 */
  virtual bool blockEyes();
/**
 * Remove the block on the eyes.
 * @return true/false on success/failure.
 */
  virtual bool clearEyes();
/**
 * Select the arm to deal with.
 * @param arm is "left" or "right".
 * @return true/false on success/failure.
 */
  virtual bool setArm(const std::string& arm);
/**
 * Return the current arm.
 * @return "left" or "right".
 */
  virtual std::string getArm();
/**
 * Set up the calibrator type.
 * @param type can be one of the following: \n
 * "se3", "se3+scale", "affine", "lssvm".
 * @param extrapolation specifies whether the calibrator will be
 * used for extrapolating data ("true") or not ("false"); if "auto"
 * is provided, then automatic choice is taken depending on the type.
 * @return true/false on success/failure.
 */
  virtual bool setCalibrationType(const std::string& type, const std::string& extrapolation = "auto");
/**
 * Return the current calibration type.
 * @return the calibration type.
 */
  virtual std::string getCalibrationType();
/**
 * Ask the current calibrator to carry out the calibration.
 * @param rm_outliers if true outliers removal is performed.
 * @return a property containing the output in terms of
 * calibration errors for each subsystem: "calibrator", "aligner".
 */
  virtual yarp::os::Property calibrate(const bool rm_outliers = 1);
/**
 * Push the current calibrator in the list of experts.
 * @return true/false on success/failure.
 * @note the calibrator needs to have been calibrated at least once.
 */
  virtual bool pushCalibrator();
/**
 * Enable/disable the use of experts for touch test.
 * @param switch is "on"/"off" to use/not-use the experts.
 * @return true/false on success/failure.
 */
  virtual bool setTouchWithExperts(const std::string& sw);
/**
 * Return the current status of the switch for experts usage
 * during touch test.
 * @return "on"/"off" if experts are used/not-used.
 */
  virtual std::string getTouchWithExperts();
/**
 * Yield a <i>touch</i> action with the finger on a depth point.
 * @param u the u-coordinate of the depth point in the image plane.
 * @param v the v-coordinate of the depth point in the image plane.
 * @return true/false on success/failure.
 */
  virtual bool touch(const int32_t u, const int32_t v);
/**
 * Retrieve the compensated kinematic point corresponding to the input
 * depth point.
 * @param arm accounts for "left" or "right" list of experts.
 * @param x the x-coordinate of the depth point.
 * @param y the y-coordinate of the depth point.
 * @param z the z-coordinate of the depth point.
 * @return the requested point in \ref PointReq format.
 */
  virtual PointReq getPoint(const std::string& arm, const double x, const double y, const double z);
/**
 * Set on/off an experiment.
 * @param exp the experiment ("depth2kin" or "aligneyes") to switch on/off.
 * @param v is "on" or "off".
 * @return true/false on success/failure.
 */
  virtual bool setExperiment(const std::string& exp, const std::string& v);
/**
 * Return the current status of the experiment.
 * @param exp the experiment ("depth2kin" or "aligneyes")
 * @return "on"/"off".
 */
  virtual std::string getExperiment(const std::string& exp);
/**
 * Retrieve the current extrinsics camera parameters.
 * @param eye is "left" or "right" camera eye.
 * @return a 6x1 vector containing the translational and the
 * rotational (in roll-pith-yaw convention) parts of the
 * extrinsics matrix.
 */
  virtual yarp::sig::Vector getExtrinsics(const std::string& eye);
/**
 * Reset the extrinsics matrix to default eye matrix.
 * @param eye is "left" or "right" camera eye.
 * @return true/false on success/failure.
 */
  virtual bool resetExtrinsics(const std::string& eye);
/**
 * Set up the wait timeout used during exploration between
 * two consecutive data points.
 * @param wait the timeout in seconds.
 * @return true/false on success/failure.
 */
  virtual bool setExplorationWait(const double wait);
/**
 * Return the current wait timeout used during exploration
 * between two consecutive data points.
 * @return the wait timeout in seconds.
 */
  virtual double getExplorationWait();
/**
 * Set up the internally coded exploration space composed by
 * two co-centered ellipses, one orthogonal to other, and defined
 * by means of the center and the two semi-axes.
 * @param cx the center x-coordinate.
 * @param cy the center y-coordinate.
 * @param cz the center z-coordiante.
 * @param a the major semi-axis length.
 * @param b the minor semi-axis length.
 * @return true/false on success/failure.
 */
  virtual bool setExplorationSpace(const double cx, const double cy, const double cz, const double a, const double b);
/**
 * Set up the exploration space in terms of differences with respect
 * to the internally coded couple of ellipses.
 * @param dcx the center delta x-coordinate.
 * @param dcy the center delta y-coordinate.
 * @param dcz the center delta z-coordiante.
 * @param da the major semi-axis delta length.
 * @param db the minor semi-axis delta length.
 * @return true/false on success/failure.
 */
  virtual bool setExplorationSpaceDelta(const double dcx = 0, const double dcy = 0, const double dcz = 0, const double da = 0, const double db = 0);
/**
 * Return some progress about the ongoing exploration.
 * @return a property that looks like
 * ("status" ["idle"|"ongoing"]) ("total_points" <int>) ("remaining_points" <int>)
 * ("calibrator_points" <int>) ("aligner_points" <int>)
 */
  virtual yarp::os::Property getExplorationData();
/**
 * Clean up the internal list of explored points pairs.
 * @return true/false on success/failure.
 */
  virtual bool clearExplorationData();
/**
 * Make the robot reach a predefined posture.
 * @param type can be one of the following: \n
 * "home", "look_hands".
 * @return true/false on success/failure.
 */
  virtual bool posture(const std::string& type);
/**
 * Put the robot in a suitable predefined posture
 * and then execute depth calibration.
 * @return true/false on success/failure.
 */
  virtual bool calibrateDepth();
/**
 * Quit the module.
 * @return true/false on success/failure.
 */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif


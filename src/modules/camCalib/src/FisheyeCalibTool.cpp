// Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
// All rights reserved.
//
// This software may be modified and distributed under the terms of the
// BSD-3-Clause license. See the accompanying LICENSE file for details.

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
 
#include <algorithm>
#include <cmath>
 
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
 
#include <iCub/FisheyeCalibTool.h>
 
using yarp::os::Bottle;
using yarp::os::Searchable;
using yarp::sig::ImageOf;
using yarp::sig::PixelRgb;
 
FisheyeCalibTool::FisheyeCalibTool()
    : previousImageSize(-1, -1)
    , groupName()
    , needInitialization(true)
    , balance(0.0)
    , fovScale(1.0)
    , drawEpipolars(false)
    , epipolarLineStep(48)
{
}
 
bool FisheyeCalibTool::open(Searchable& config)
{
    return configure(config);
}
 
bool FisheyeCalibTool::close()
{
    leftMap1.release();
    leftMap2.release();
    rightMap1.release();
    rightMap2.release();
    return true;
}
 
bool FisheyeCalibTool::loadCameraCalibration(const Searchable& group,
                                               CameraCalibration& camera,
                                               const std::string& groupName)
{
    const char* required[] = {"w", "h", "fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"};
    for (const char* key : required)
    {
        if (!group.check(key))
        {
            yError() << "Stereo rectification: missing" << key << "in" << groupName;
            return false;
        }
    }
 
    camera.calibrationSize = cv::Size(group.find("w").asInt32(), group.find("h").asInt32());
    if (camera.calibrationSize.width <= 0 || camera.calibrationSize.height <= 0)
    {
        yError() << "Stereo rectification: invalid calibration size in" << groupName;
        return false;
    }
 
    camera.K = cv::Mat::eye(3, 3, CV_64F);
    camera.K.at<double>(0, 0) = group.find("fx").asFloat64();
    camera.K.at<double>(1, 1) = group.find("fy").asFloat64();
    camera.K.at<double>(0, 2) = group.find("cx").asFloat64();
    camera.K.at<double>(1, 2) = group.find("cy").asFloat64();
 
    camera.D = cv::Mat::zeros(4, 1, CV_64F);
    camera.D.at<double>(0, 0) = group.find("k1").asFloat64();
    camera.D.at<double>(1, 0) = group.find("k2").asFloat64();
    camera.D.at<double>(2, 0) = group.find("k3").asFloat64();
    camera.D.at<double>(3, 0) = group.find("k4").asFloat64();
    return true;
}
 
bool FisheyeCalibTool::loadStereoCalibration(const Searchable& config)
{
    if (!config.check("STEREO_DISPARITY"))
    {
        yError() << "Stereo rectification: [STEREO_DISPARITY] group not found";
        return false;
    }
 
    const Bottle& stereo = config.findGroup("STEREO_DISPARITY");
    if (!stereo.check("HN"))
    {
        yError() << "Stereo rectification: HN is missing from [STEREO_DISPARITY]";
        return false;
    }
 
    const Bottle* homogeneous = stereo.find("HN").asList();
    if (homogeneous == nullptr || homogeneous->size() != 16)
    {
        yError() << "Stereo rectification: HN must contain a 4x4 homogeneous transform";
        return false;
    }
 
    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            rotation.at<double>(row, col) = homogeneous->get(row * 4 + col).asFloat64();
        }
        translation.at<double>(row, 0) = homogeneous->get(row * 4 + 3).asFloat64();
    }
 
    if (cv::norm(translation) <= 1e-12)
    {
        yError() << "Stereo rectification: translation from HN is zero";
        return false;
    }
    return true;
}
 
bool FisheyeCalibTool::configure(Searchable& config)
{
    groupName = config.check("group", yarp::os::Value("")).asString();
    if (groupName != "CAMERA_CALIBRATION_LEFT" && groupName != "CAMERA_CALIBRATION_RIGHT")
    {
        yError() << "Stereo rectification requires --group CAMERA_CALIBRATION_LEFT or CAMERA_CALIBRATION_RIGHT";
        return false;
    }
    if (!config.check("CAMERA_CALIBRATION_LEFT") || !config.check("CAMERA_CALIBRATION_RIGHT"))
    {
        yError() << "Stereo rectification requires both camera calibration groups";
        return false;
    }
 
    const Bottle& leftGroup = config.findGroup("CAMERA_CALIBRATION_LEFT");
    const Bottle& rightGroup = config.findGroup("CAMERA_CALIBRATION_RIGHT");
    if (!loadCameraCalibration(leftGroup, leftCamera, "CAMERA_CALIBRATION_LEFT")
        || !loadCameraCalibration(rightGroup, rightCamera, "CAMERA_CALIBRATION_RIGHT")
        || !loadStereoCalibration(config))
    {
        return false;
    }

    const Bottle& selectedGroup = (groupName == "CAMERA_CALIBRATION_LEFT") ? leftGroup : rightGroup;
    const yarp::os::Value defaultBalance = config.check("rectifyAlpha", yarp::os::Value(0.0));
    balance = config.check("balance", selectedGroup.check("balance", defaultBalance)).asFloat64();
    balance = std::max(0.0, std::min(1.0, balance));
    fovScale = config.check("fovScale", selectedGroup.check("fovScale", yarp::os::Value(1.0))).asFloat64();
    if (fovScale <= 0.0)
    {
        yError() << "Fisheye rectification: fovScale must be greater than zero";
        return false;
    }
    drawEpipolars = config.check("drawEpipolars", yarp::os::Value(false)).asBool();
    epipolarLineStep = config.check("epipolarLineStep", yarp::os::Value(48)).asInt32();
    epipolarLineStep = epipolarLineStep > 0 ? epipolarLineStep : 1;
    previousImageSize = cv::Size(-1, -1);
    needInitialization = true;
    yDebug() << "FisheyeCalibTool configured with balance:" << balance << "fovScale:" << fovScale
             << "drawEpipolars:" << drawEpipolars << "epipolarLineStep:" << epipolarLineStep;
    return true;
}
 
cv::Mat FisheyeCalibTool::scaledCameraMatrix(const CameraCalibration& camera,
                                               const cv::Size& imageSize) const
{
    const double scaleX = static_cast<double>(imageSize.width) / camera.calibrationSize.width;
    const double scaleY = static_cast<double>(imageSize.height) / camera.calibrationSize.height;
    cv::Mat scaled = camera.K.clone();
    scaled.at<double>(0, 0) *= scaleX;
    scaled.at<double>(0, 2) *= scaleX;
    scaled.at<double>(1, 1) *= scaleY;
    scaled.at<double>(1, 2) *= scaleY;
    return scaled;
}
 
bool FisheyeCalibTool::initializeMaps(const cv::Size& imageSize)
{
    const cv::Mat leftK = scaledCameraMatrix(leftCamera, imageSize);
    const cv::Mat rightK = scaledCameraMatrix(rightCamera, imageSize);
    cv::Mat leftR;
    cv::Mat rightR;
    cv::Mat leftP;
    cv::Mat rightP;
    cv::Mat disparityToDepth;
    cv::fisheye::stereoRectify(leftK, leftCamera.D, rightK, rightCamera.D, imageSize,
                               rotation, translation, leftR, rightR, leftP, rightP,
                               disparityToDepth, cv::fisheye::CALIB_ZERO_DISPARITY,
                               imageSize, balance, fovScale);
 
    if (leftR.empty() || rightR.empty() || leftP.empty() || rightP.empty())
    {
        yError() << "Fisheye rectification: stereoRectify returned empty matrices";
        return false;
    }
 
    const double leftFocal = leftP.at<double>(0, 0);
    const double rightFocal = rightP.at<double>(0, 0);
    if (!std::isfinite(leftFocal) || !std::isfinite(rightFocal)
        || leftFocal < 1.0 || rightFocal < 1.0)
    {
        yWarning() << "Fisheye rectification: stereoRectify returned an invalid focal length"
                   << leftFocal << rightFocal
                   << "- using a projection matrix derived from the original intrinsics";
        cv::Mat commonProjection = cv::Mat::eye(3, 3, CV_64F);
        commonProjection.at<double>(0, 0) = 0.5 * (leftK.at<double>(0, 0) + rightK.at<double>(0, 0));
        commonProjection.at<double>(1, 1) = 0.5 * (leftK.at<double>(1, 1) + rightK.at<double>(1, 1));
        commonProjection.at<double>(0, 2) = 0.5 * (leftK.at<double>(0, 2) + rightK.at<double>(0, 2));
        commonProjection.at<double>(1, 2) = 0.5 * (leftK.at<double>(1, 2) + rightK.at<double>(1, 2));
        leftP = commonProjection;
        rightP = commonProjection;
    }

    if(groupName == "CAMERA_CALIBRATION_LEFT")
    {
        cv::fisheye::initUndistortRectifyMap(leftK, leftCamera.D, leftR, leftP,
                                        imageSize, CV_16SC2, leftMap1, leftMap2);
    }
    else if(groupName == "CAMERA_CALIBRATION_RIGHT")
    {
        cv::fisheye::initUndistortRectifyMap(rightK, rightCamera.D, rightR, rightP,
                                            imageSize, CV_16SC2, rightMap1, rightMap2);    
    }

    previousImageSize = imageSize;
    needInitialization = false;
    return (!leftMap1.empty() && !leftMap2.empty()) || (!rightMap1.empty() && !rightMap2.empty());
}
 
void FisheyeCalibTool::drawEpipolarLines(cv::Mat& image) const
{
    const cv::Scalar lineColor(0, 255, 0);
    for (int y = 0; y < image.rows; y += epipolarLineStep)
    {
        cv::line(image, cv::Point(0, y), cv::Point(image.cols - 1, y), lineColor, 1, cv::LINE_AA);
    }
}
 
void FisheyeCalibTool::apply(const ImageOf<PixelRgb>& in, ImageOf<PixelRgb>& out)
{
    const cv::Size imageSize(in.width(), in.height());
    if (needInitialization || imageSize != previousImageSize)
    {
        if (!initializeMaps(imageSize))
        {
            yError() << "Fisheye calibration: unable to initialize maps";
            out = in;
            return;
        }
    }

    out.resize(in.width(), in.height());
    cv::Mat input(in.height(), in.width(), CV_8UC3, in.getRawImage(), in.getRowSize());
    cv::Mat output(out.height(), out.width(), CV_8UC3, out.getRawImage(), out.getRowSize());
    const cv::Mat& activeMap1 = (groupName == "CAMERA_CALIBRATION_LEFT") ? leftMap1 : rightMap1;
    const cv::Mat& activeMap2 = (groupName == "CAMERA_CALIBRATION_LEFT") ? leftMap2 : rightMap2;
    cv::remap(input, output, activeMap1, activeMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (drawEpipolars)
    {
        drawEpipolarLines(output);
    }
}
 

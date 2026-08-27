// Copyright (C) 2026 Istituto Italiano di Tecnologia (IIT)
// All rights reserved.
//
// This software may be modified and distributed under the terms of the
// BSD-3-Clause license. See the accompanying LICENSE file for details.

#ifndef __FISHEYECALIBTOOL__
#define __FISHEYECALIBTOOL__
 
#include <string>
 
#include <opencv2/core.hpp>
 
#include <yarp/os/Searchable.h>
#include <yarp/sig/Image.h>
 
#include <iCub/ICalibTool.h>
 
class FisheyeCalibTool : public ICalibTool
{
private:
    struct CameraCalibration
    {
        cv::Mat K;
        cv::Mat D;
        cv::Size calibrationSize;
    };
 
    CameraCalibration leftCamera;
    CameraCalibration rightCamera;
    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat leftMap1;
    cv::Mat leftMap2;
    cv::Mat rightMap1;
    cv::Mat rightMap2;
    cv::Size previousImageSize;
    std::string groupName;
    bool needInitialization;
    double balance;
    double fovScale;
    bool drawEpipolars;
    int epipolarLineStep;
 
    bool loadCameraCalibration(const yarp::os::Searchable& group,
                               CameraCalibration& camera,
                               const std::string& groupName);
    bool loadStereoCalibration(const yarp::os::Searchable& config);
    bool initializeMaps(const cv::Size& imageSize);
    cv::Mat scaledCameraMatrix(const CameraCalibration& camera,
                               const cv::Size& imageSize) const;
    void drawEpipolarLines(cv::Mat& image) const;
 
public:
    FisheyeCalibTool();
    ~FisheyeCalibTool() = default;
 
    bool open(yarp::os::Searchable& config) override;
    bool close() override;
    bool configure(yarp::os::Searchable& config) override;
 
    void apply(const yarp::sig::ImageOf<yarp::sig::PixelRgb>& in,
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& out) override;
};
 
#endif
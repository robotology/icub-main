#ifndef ICUB_STEREOCALIB_CALIBRATION_TYPES_H
#define ICUB_STEREOCALIB_CALIBRATION_TYPES_H

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace stereo_calib
{

    enum class CameraSide
    {
        Left, 
        Right
    };

    enum class CalibrationMode
    {
        MonocularLeft,  // validate obs -> calibrate left -> populate result.leftCamera -> no call to stereo calib nor rectification
        MonocularRight, // validate obs -> calibrate right -> populate result.rightCamera -> no call to stereo calib nor rectification
        MonocularBoth,  // validate obs -> calibrate left and right -> estimate intrinsics -> no estimation of R and T or rectification
        StereoFull      // validate obs -> calibrate left and right -> stereo calib with fixed intrinsics -> stereo rectification -> evaluate rectification quality -> populate result with all fields
    };

    struct ChessboardConfiguration
    {
        int cornersX{0};
        int cornersY{0};

        // Physical side length of one chessboard square
        double squareSizeMeters{0.0};

        cv::Size patternSize() const
        {
            return cv::Size(cornersX, cornersY);
        }
        
        std::size_t cornersCount() const
        {
            if(!isValid())
            {
                return 0;
            }
            return static_cast<std::size_t>(cornersX)*static_cast<std::size_t>(cornersY);
        }

        bool isValid() const
        {
            return (cornersX > 1 && cornersY > 1 && squareSizeMeters > 0.0);
        }

        std::vector<cv::Point3f> createObjectPoints() const
        {
            std::vector<cv::Point3f> points;

            if(!isValid())
            {
                return points;
            }

            points.reserve(cornersCount());

            for (int r = 0; r < cornersY; ++r)
            {
                for(int c = 0; c < cornersX; ++c)
                {
                    points.emplace_back(
                        static_cast<float>(c * squareSizeMeters),
                        static_cast<float>(r * squareSizeMeters),
                        0.0F
                    );
                }
            }
            
            return points;
        }
    };

    struct StereoObservation
    {
        cv::Size imageSize;

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> leftImagePoints;
        std::vector<cv::Point2f> rightImagePoints;

        double leftTimestampSeconds {0.0};
        double rightTimestampSeconds {0.0};
        double timestampDeltaSeconds {0.0};

        int64_t leftSequenceNumber {-1};
        int64_t rightSequenceNumber {-1};

        bool isValid() const
        {
            if(imageSize.width <= 0 ||
                imageSize.height <= 0)
            {
                return false;
            }

            if(objectPoints.empty())
                return false;
            
            return (objectPoints.size() == leftImagePoints.size() &&
                objectPoints.size() == rightImagePoints.size() &&
                timestampDeltaSeconds >=0.0);
        }
    };
    
    struct FisheyeCalibrationOptions
    {
        CalibrationMode calibrationMode{CalibrationMode::StereoFull};

        cv::Size imageSize{1920, 1080};

        int monocularFlags{
            cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
            cv::fisheye::CALIB_CHECK_COND |
            cv::fisheye::CALIB_FIX_SKEW
        };

        int stereoFlags{
            cv::fisheye::CALIB_FIX_INTRINSIC |
            cv::fisheye::CALIB_CHECK_COND |
            cv::fisheye::CALIB_FIX_SKEW
        };

        cv::TermCriteria criteria{
            cv::TermCriteria::COUNT |
            cv::TermCriteria::EPS,
            100,
            1e-5
        };

        double rectificationBalance {0.0};
        double rectificationFovScale{1.0};

        bool zeroDisparity{true};

        bool isValid() const
        {
            return (imageSize.width > 0 &&
                imageSize.height > 0 &&
                std::isfinite(rectificationBalance) &&
                rectificationBalance >= 0.0 &&
                rectificationBalance <= 1.0 &&
                std::isfinite(rectificationFovScale) &&
                rectificationFovScale > 0.0 &&
                (!(criteria.type & cv::TermCriteria::COUNT) || criteria.maxCount > 0) &&
                (!(criteria.type & cv::TermCriteria::EPS) ||
                 (std::isfinite(criteria.epsilon) && criteria.epsilon > 0.0)));
        }
    };

    struct CameraCalibrationResult
    {
        cv::Size imageSize;

        // 3x3 intrinsic camera matrix
        cv::Mat K;

        // Four fisheye coefficients: k1, k2, k3, k4.
        // Standardize internally on 4x1 CV_64F matrix
        cv::Mat D;

        // Board pose for each accepted observation
        std::vector<cv::Mat> rotationVectors;
        std::vector<cv::Mat> translationVectors;

        // Optional quality value calculated for each view
        std::vector<double> perViewRms;

        double rms{-1.0};

        bool isValid() const
        {
            return (imageSize.width > 0 &&
                imageSize.height > 0 &&
                K.rows == 3 &&
                K.cols == 3 &&
                D.total() == 4 &&
                rms >= 0.0);
        }
    };

    struct StereoCalibrationResult
    {
        // Transform convention:
        //
        // X_right = R * X_left + T
        //
        // R: 3x3 rotation matrix
        // T: 3x1 translation vector.
        cv::Mat R;
        cv::Mat T;

        // Optional quality value calculated for each stereo observation
        std::vector<double> perPairRms;

        double rms{-1.0};

        bool isValid() const
        {
            return (R.rows == 3 &&
                R.cols == 3 &&
                T.total() == 3 &&
                rms >= 0.0);
        }
    };

    struct RectificationResult
    {
        cv::Size outputImageSize;

        // Rectification rotations.
        cv::Mat R1;
        cv::Mat R2;

        // Rectified projection matrices
        cv::Mat P1;
        cv::Mat P2;

        // Disparity-to-depth mapping matrix
        cv::Mat Q;

        double balance{0.0};
        double fovScale{1.0};

        bool zeroDisparity{true};

        bool isValid() const
        {
            return (outputImageSize.width > 0 &&
                outputImageSize.height > 0 &&
                R1.rows == 3 &&
                R1.cols == 3 &&
                R2.rows == 3 &&
                R2.cols == 3 &&
                P1.rows == 3 &&
                P1.cols == 4 &&
                P2.rows == 3 &&
                P2.cols == 4 &&
                Q.rows == 4 &&
                Q.cols == 4);
        }
    };

    struct CalibrationQualityMetrics
    {
        std::size_t synchronizedPairs{0};
        std::size_t acceptedObservations{0};
        std::size_t rejectedDetections{0};

        double meanTimestampDeltaMs {0.0};
        double maxTimestampDeltaMs {0.0};

        double meanVerticalRectificationErrorPx {-1.0};
        double medianVerticalRectificationErrorPx {-1.0};
        double rmsVerticalRectificationErrorPx {-1.0};
        double p95VerticalRectificationErrorPx {-1.0};
        double maxVerticalRectificationErrorPx {-1.0};
    };

    struct CalibrationResult
    {
        CalibrationMode mode{CalibrationMode::StereoFull};

        CameraCalibrationResult leftCamera;
        CameraCalibrationResult rightCamera;

        StereoCalibrationResult stereo;
        RectificationResult rectification;

        CalibrationQualityMetrics quality;

        bool isValid() const
        {
            switch (mode)
            {
                case CalibrationMode::MonocularLeft:
                    return leftCamera.isValid();
                case CalibrationMode::MonocularRight:
                    return rightCamera.isValid();
                case CalibrationMode::MonocularBoth:
                    return (leftCamera.isValid() && rightCamera.isValid());
                case CalibrationMode::StereoFull:
                    return (leftCamera.isValid() &&
                        rightCamera.isValid() &&
                        stereo.isValid() &&
                        rectification.isValid());
            }
            return false;
        }
    };

} // namespace stereo_calib

#endif

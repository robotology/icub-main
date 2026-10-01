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
        MonocularLeft,  // validate obs -> calibrate left -> populate result.leftCamera -> no call to stereo calib
        MonocularRight, // validate obs -> calibrate right -> populate result.rightCamera -> no call to stereo calib
        MonocularBoth,  // validate obs -> calibrate left and right -> estimate intrinsics -> no estimation of R and T
        StereoFull,      // validate obs -> calibrate left and right -> stereo calib with fixed intrinsics -> evaluate stereo quality -> populate result with all fields
        Invalid = 255
    };

    enum class CameraModel
    {
        Pinhole,
        Fisheye,
        Invalid = 255
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

        // Relative filenames of the raw images saved for this accepted
        // observation.  They are intentionally part of the observation so
        // that the point data and the dataset images can be joined without
        // relying on directory iteration.
        std::string leftImageFilename;
        std::string rightImageFilename;

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

    struct CommonCalibrationOptions
    {
        CalibrationMode calibrationMode{CalibrationMode::StereoFull};
        cv::Size imageSize{1920, 1080};
    };
    
    struct PinholeCalibrationOptions
    {
        CommonCalibrationOptions common;
        double cameraFocalLengthGuess{625.0};

        int monocularFlags{
            // cv::CALIB_USE_INTRINSIC_GUESS | to be added later if performance is not good enough
            // cv::CALIB_FIX_PRINCIPAL_POINT |
            // cv::CALIB_FIX_TANGENT_DIST |
            // camCalib uses the legacy four-coefficient pinhole model, so the
            // OpenCV-only fifth coefficient must remain fixed at zero.
            cv::CALIB_FIX_K3 
        };

        int stereoFlags{
            cv::CALIB_FIX_ASPECT_RATIO |
            cv::CALIB_FIX_INTRINSIC |
            cv::CALIB_FIX_K3
        };

        cv::TermCriteria criteria{
            cv::TermCriteria::COUNT |
            cv::TermCriteria::EPS,
            100,
            1e-5
        };

        bool isValid() const
        {
            return (common.imageSize.width > 0 &&
                common.imageSize.height > 0 &&
                (!(criteria.type & cv::TermCriteria::COUNT) || criteria.maxCount > 0) &&
                (!(criteria.type & cv::TermCriteria::EPS) ||
                 (std::isfinite(criteria.epsilon) && criteria.epsilon > 0.0)));
        }
    };
    
    struct FisheyeCalibrationOptions
    {
        CommonCalibrationOptions common;
        double cameraFocalLengthGuess{625.0};

        int monocularFlags{
            cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
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

        bool isValid() const
        {
            return (common.imageSize.width > 0 &&
                common.imageSize.height > 0 &&
                (!(criteria.type & cv::TermCriteria::COUNT) || criteria.maxCount > 0) &&
                (!(criteria.type & cv::TermCriteria::EPS) ||
                 (std::isfinite(criteria.epsilon) && criteria.epsilon > 0.0)));
        }
    };

    struct CameraCalibrationResult
    {
        CameraModel model{CameraModel::Pinhole};
        cv::Size imageSize;

        // 3x3 intrinsic camera matrix
        cv::Mat K;

        // Pinhole distortion coefficients: [k1, k2, p1, p2]
        // Fisheye distortion coefficients: [k1, k2, k3, k4]
        cv::Mat D;

        // Board pose for each accepted observation
        std::vector<cv::Mat> rotationVectors;
        std::vector<cv::Mat> translationVectors;

        // Reprojection RMS for every accepted calibration observation.
        std::vector<double> perViewRms;

        double rms{-1.0};
        
        bool isValid() const
        {
            if(model != CameraModel::Pinhole &&
                model != CameraModel::Fisheye)
                return false;

            if(imageSize.width <=0 || imageSize.height <= 0)
                return false;
            
            if(K.empty() ||
                K.rows != 3 ||
                K.cols != 3 ||
                K.type() != CV_64F ||
                !cv::checkRange(K, true))
                return false;
            
            if(D.empty() ||
                D.total() != 4 ||
                D.type() != CV_64F ||
                !cv::checkRange(D, true))
                return false;
            
            if(K.at<double>(0, 0) <= 0.0 ||
                K.at<double>(1,1) <= 0.0)
                return false;

            if(!std::isfinite(rms) || rms < 0.0)
                return false;
            
            if(rotationVectors.size() != translationVectors.size() ||
                rotationVectors.size() != perViewRms.size())
                return false;
            
            for (size_t i = 0; i < rotationVectors.size(); i++)
            {
                const cv::Mat& rotation = rotationVectors[i];
                const cv::Mat& translation = translationVectors[i];

                if(rotation.empty() ||
                    rotation.total() != 3 ||
                    rotation.type() != CV_64F ||
                    !cv::checkRange(rotation, true))
                    return false;

                if(translation.empty() ||
                    translation.total() != 3 ||
                    translation.type() != CV_64F ||
                    !cv::checkRange(translation, true))
                    return false;

                if(!std::isfinite(perViewRms[i]) ||
                    perViewRms[i] < 0.0)
                    return false;
            }
            
            return true;
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

        double rms{-1.0};

        bool isValid() const
        {
            if(R.empty() ||
                R.rows != 3 || 
                R.cols != 3 ||
                R.type() != CV_64F ||
                !cv::checkRange(R, true)
                )
                return false;

            if(T.empty() ||
                T.total() != 3 ||
                T.type() != CV_64F ||
                !cv::checkRange(T, true))
                return false;

            if(!std::isfinite(rms) ||
                rms < 0.0 )
                return false;
            
            const cv::Mat identity = cv::Mat::eye(3,3,CV_64F);
            if(std::abs(cv::determinant(R) - 1.0) >= 1e-3 ||
                cv::norm(R.t() * R - identity, cv::NORM_INF) >= 1e-3 ||
                cv::norm(T) <= 1e-9)
                return false;
            
            return true;
        }
    };

    struct CalibrationQualityMetrics
    {
        std::size_t synchronizedPairs{0};
        std::size_t acceptedObservations{0};
        std::size_t rejectedDetections{0};

        double meanTimestampDeltaMs {0.0};
        double maxTimestampDeltaMs {0.0};

        // Baseline expressed in the same unit as StereoCalibrationResult::T.
        // It is calculated by the calibration engine, not by persistence.
        double baseline{-1.0};
    };

    struct CalibrationResult
    {
        CameraModel model{CameraModel::Pinhole};
        CalibrationMode mode{CalibrationMode::StereoFull};

        CameraCalibrationResult leftCamera;
        CameraCalibrationResult rightCamera;

        StereoCalibrationResult stereo;

        CalibrationQualityMetrics quality;

        bool isValid() const
        {
            const auto validCamera = 
                [this](const CameraCalibrationResult& camera)
                {
                    return camera.model == model &&
                        camera.isValid();
                };

            switch (mode)
            {
                case CalibrationMode::MonocularLeft:
                    return validCamera(leftCamera);
                case CalibrationMode::MonocularRight:
                    return validCamera(rightCamera);
                case CalibrationMode::MonocularBoth:
                    return (validCamera(leftCamera) && validCamera(rightCamera));
                case CalibrationMode::StereoFull:
                    return (validCamera(leftCamera) &&
                        validCamera(rightCamera) &&
                        stereo.isValid());
            }
            return false;
        }
    };

} // namespace stereo_calib

#endif

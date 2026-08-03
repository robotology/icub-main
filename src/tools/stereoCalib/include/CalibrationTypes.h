#ifndef ICUB_STEREOCALIB_CALIBRATION_TYPES_H
#define ICUB_STEREOCALIB_CALIBRATION_TYPES_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

namespace stereo_calib
{

    enum class CameraSide
    {
        Left, 
        Right
    };

    struct ChessboardConfiguration
    {
        int cornersX{0};
        int cornersY{0};

        // Physical side length of one chessboard square
        double squareSizeMeters{0.0};

        Size patternSize() const
        {
            return Size(cornersX, cornersY);
        }
        
        size_t cornersCount() const
        {
            if(!isValid())
            {
                return 0;
            }
            return static_cast<size_t>(cornersX)*static_cast<size_t>(cornersY);
        }

        bool isValid() const
        {
            return (cornersX > 1 && cornersY > 1 && squareSizeMeters > 0.0);
        }

        vector<Point3f> createObjectPoints() const
        {
            vector<Point3f> points;

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
        Size imageSize;

        vector<Point3f> objectPoints;
        vector<Point2f> leftImagePoints;
        vector<Point2f> rightImagePoints;

        double leftTimestampSeconds {0.0};
        double rightTimestampSeconds {0.0};
        double timestampDeltaSeconds {0.0};

        int64_t leftSequence {-1};
        int64_t rightSequence {-1};

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
        Size imageSize{1920, 1080};

        int monocularFlags{
            fisheye::CALIB_RECOMPUTE_EXTRINSIC |
            fisheye::CALIB_CHECK_COND |
            fisheye::CALIB_FIX_SKEW
        };

        int stereoFlags{
            fisheye::CALIB_FIX_INTRINSIC |
            fisheye::CALIB_CHECK_COND |
            fisheye::CALIB_FIX_SKEW
        };

        TermCriteria criteria{
            TermCriteria::COUNT |
            TermCriteria::EPS,
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
                rectificationBalance >= 0.0 &&
                rectificationBalance <= 1.0 &&
                rectificationFovScale > 0.0);
        }
    };

    struct CameraCalibrationResult
    {
        Size imageSize;

        // 3x3 intrinsic camera matrix
        Mat K;

        // Four fisheye coefficients: k1, k2, k3, k4.
        // Standardize internally on 4x1 CV_64F matrix
        Mat D;

        // Board pose for each accepted observation
        vector<Mat> rotationVectors;
        vector<Mat> translationVectors;

        // Optional quality value calculated for each view
        vector<double> perViewRms;

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
        Mat R;
        Mat T;

        // Optional quality value calculated for each stereo observation
        vector<double> perPairRms;

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
        Size outputImageSize;

        // Rectification rotations.
        Mat R1;
        Mat R2;

        // Rectified projection matrices
        Mat P1;
        Mat P2;

        // Disparity-to-depth mapping matrix
        Mat Q;

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
        size_t synchronizedPairs{0};
        size_t acceptedObservations{0};
        size_t rejectedDections{0};

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
        CameraCalibrationResult leftCamera;
        CameraCalibrationResult rightCamera;

        StereoCalibrationResult stereo;
        RectificationResult rectification;

        CalibrationQualityMetrics quality;

        bool isValid() const
        {
            return (leftCamera.isValid() &&
                rightCamera.isValid() &&
                stereo.isValid() &&
                rectification.isValid());
        }
    };

} // namespace stereo_calib

#endif
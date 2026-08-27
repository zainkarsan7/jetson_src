#pragma once

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"


#include <tf2/LinearMath/Transform.h>

class ArucoDetector{
    public: 
        struct Detection{
            int id;
            std::vector<cv::Point2f> corners;
            tf2::Transform T_cam_marker;
        };

        ArucoDetector(int dict_id, double marker_size);

        void setCamInfo(
            const cv::Mat& cam_mat, const cv::Mat& dist_coeffs
        );

        std::vector<Detection> detect(const cv::Mat& img);

        bool cam_info_set() const;

    private:
        tf2::Transform cvPoseToTf(
            const cv::Vec3d& rvec,
            const cv::Vec3d& tvec 
        )const;

        double marker_size_;
        cv::Ptr<cv::aruco::Dictionary> dict_;
        cv::Ptr<cv::aruco::DetectorParameters> det_params_;

        cv::Mat cam_mat_;
        cv::Mat dist_coeffs_;
        bool cam_info_set_{false};
        
};
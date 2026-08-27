#include "trolley_cam/aruco_detector.hpp"
#include <stdexcept>
#include "opencv2/calib3d.hpp"
#include <tf2/LinearMath/Transform.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/LinearMath/Matrix3x3.h>

ArucoDetector::ArucoDetector(int dict_id, double marker_size)
:marker_size_(marker_size)
{
    dict_ = cv::aruco::getPredefinedDictionary(dict_id);
    det_params_ = cv::aruco::DetectorParameters::create();

}

void ArucoDetector::setCamInfo( const cv::Mat& cam_mat, const cv::Mat& dist_coeffs){
    cam_mat_ = cam_mat.clone();
    dist_coeffs_ = dist_coeffs.clone();
    cam_info_set_ = true;
}

bool ArucoDetector::cam_info_set() const{return cam_info_set_;}

std::vector<ArucoDetector::Detection> ArucoDetector::detect(const cv::Mat& img){
    if (!cam_info_set_) 
    {
        throw std::runtime_error("Cam cal missing");
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    
    cv::aruco::detectMarkers(img,dict_,corners,ids,det_params_,rejected);

    std::vector<Detection> detections;

    if (ids.empty()){
        return detections;
    }

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;

    cv::aruco::estimatePoseSingleMarkers(corners,marker_size_,cam_mat_,dist_coeffs_,rvecs,tvecs);

    detections.reserve(ids.size());

    for (size_t i=0; i<ids.size(); ++i){
        Detection det_;
        det_.id = ids[i];
        det_.corners = corners[i];
        det_.T_cam_marker = cvPoseToTf(rvecs[i],tvecs[i]);
        detections.push_back(det_);

    }
    return detections;
}

    tf2::Transform ArucoDetector::cvPoseToTf(const cv::Vec3d& rvec,
            const cv::Vec3d& tvec 
        )const{

        }

}




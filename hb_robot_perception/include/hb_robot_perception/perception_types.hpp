#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstddef>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>

namespace hb_perception{
    using PointT = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointT>;
struct Observation{
    // put image header, camera pose?
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth_to_rgb;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr rgb_camera_info;
    geometry_msgs::msg::TransformStamped camera_pose;
    rclcpp::Time stamp;

};

struct SectionModel{
    PointCloud::Ptr cloud;
    float longitudinal_position = 0.0f;
    Eigen::Isometry3f frame = Eigen::Isometry3f::Identity();
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::UnitX();
    std::vector<Eigen::Vector2f> points_2d;
    float thickness = 0.0f;
};

struct CutCandidate{
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    Eigen::Vector3d tangent;
};

struct WorkpieceModel{
    
    //sub cloud
    PointCloud::Ptr cloud;
    Eigen::Isometry3d workpiece_frame = Eigen::Isometry3d::Identity();
    std::vector<CutCandidate> cut_candidates;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Matrix3f p_axes = Eigen::Matrix3f::Identity();
    Eigen::Vector3f l_axes = Eigen::Vector3f::UnitX();
    Eigen::Vector3f eigs = Eigen::Vector3f::Zero();
};

inline PointCloud::Ptr depthToCloud(const sensor_msgs::msg::Image::ConstSharedPtr& depth, 
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info, double depth_range){
        
    constexpr float MIN_DEPTH = 0.02f;

    auto cloud = std::make_shared<PointCloud>();

        const double fx = info->k[0];
        const double fy = info->k[4];
        const double cx = info->k[2];
        const double cy = info->k[5];
        

        auto depth_cv = cv_bridge::toCvShare(depth, sensor_msgs::image_encodings::TYPE_32FC1);
        const cv::Mat& depth_mat = depth_cv->image;
        cloud->points.reserve(
            static_cast<std::size_t>(depth_mat.rows) *
            static_cast<std::size_t>(depth_mat.cols));
        double min_val;
        double max_val;

        cv::minMaxLoc(depth_mat, &min_val, &max_val);

      
        if (depth_mat.type() != CV_32FC1) {
        
        return cloud;
        }


        std::size_t valid = 0;
        
        // for (std::uint32_t v = 0; v<depth.height; ++v){
        //     for(std::uint32_t u=0; u<depth.width; ++u){

        for (std::uint32_t v = 0; v<depth_mat.rows; ++v){
            const float* depth_row = depth_mat.ptr<float>(v);
           
            for (std::uint32_t u=0; u<depth_mat.cols; ++u){
                // const auto* depth_row = reinterpret_cast<const float*>(depth.data.data()+v*depth.step);
                const float raw_depth = depth_row[u];
                if (!std::isfinite(raw_depth) || raw_depth < MIN_DEPTH || raw_depth > depth_range){
                    continue;
                }
                valid++;
                
                const float z = static_cast<float>(raw_depth);

                PointT pt;
                pt.x = static_cast<float>((u-cx)* z / fx);
                pt.y = static_cast<float>((v-cy)*z/fy);
                pt.z = z;

                cloud->points.push_back(pt);
            }
        }
        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;



            return cloud;
        }



inline PointCloud::Ptr observationToCloud(const Observation& ob, double depth_range){
        
    constexpr float MIN_DEPTH = 0.02f;

    auto cloud = std::make_shared<PointCloud>();

        if(!ob.rgb || !ob.depth_to_rgb || !ob.rgb_camera_info){
            return cloud;
        }

        const auto& depth = *ob.depth_to_rgb;
        const auto& rgb = *ob.rgb;
        const auto& info = *ob.rgb_camera_info;

        const double fx = info.k[0];
        const double fy = info.k[4];
        const double cx = info.k[2];
        const double cy = info.k[5];
        


        auto depth_cv = cv_bridge::toCvShare(ob.depth_to_rgb, sensor_msgs::image_encodings::TYPE_32FC1);
        auto rgb_cv = cv_bridge::toCvShare(ob.rgb,sensor_msgs::image_encodings::BGRA8);
        const cv::Mat& depth_mat = depth_cv->image;
        const cv::Mat& rgb_mat = rgb_cv->image;
        cloud->points.reserve(
            static_cast<std::size_t>(depth_mat.rows) *
            static_cast<std::size_t>(depth_mat.cols));
        double min_val;
        double max_val;

        cv::minMaxLoc(depth_mat, &min_val, &max_val);

        std::cout << "depth min/max = "
                << min_val << " / "
                << max_val << std::endl;
        if (depth_mat.type() != CV_32FC1) {
        std::cerr << "Unexpected depth type: "
                << depth_mat.type()
                << " encoding: "
                << ob.depth_to_rgb->encoding
                << std::endl;
        return cloud;
        }

        if (rgb_mat.type() != CV_8UC4) {
            std::cerr << "Unexpected RGB type: "
                    << rgb_mat.type()
                    << " encoding: "
                    << ob.rgb->encoding
                    << std::endl;
            return cloud;
        }

        std::size_t valid = 0;
        
        // for (std::uint32_t v = 0; v<depth.height; ++v){
        //     for(std::uint32_t u=0; u<depth.width; ++u){

        for (std::uint32_t v = 0; v<depth_mat.rows; ++v){
            const float* depth_row = depth_mat.ptr<float>(v);
            const cv::Vec4b* rgb_row = rgb_mat.ptr<cv::Vec4b>(v);
            for (std::uint32_t u=0; u<depth_mat.cols; ++u){
                // const auto* depth_row = reinterpret_cast<const float*>(depth.data.data()+v*depth.step);
                const float raw_depth = depth_row[u];
                if (!std::isfinite(raw_depth) || raw_depth < MIN_DEPTH || raw_depth > depth_range){
                    continue;
                }
                valid++;
                
                const float z = static_cast<float>(raw_depth);

                PointT pt;
                pt.x = static_cast<float>((u-cx)* z / fx);
                pt.y = static_cast<float>((v-cy)*z/fy);
                pt.z = z;

                // color stuff
                const cv::Vec4b& pixel = rgb_row[u];
                pt.b = pixel[0];
                pt.g = pixel[1];
                pt.r = pixel[2];

                cloud->points.push_back(pt);
            }
        }
        cloud->width = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;


        std::cout << "valid depth pixels: "
              << valid << std::endl;

        std::cout << "generated cloud points: "
                << cloud->size() << std::endl;

            return cloud;
        }

    


}
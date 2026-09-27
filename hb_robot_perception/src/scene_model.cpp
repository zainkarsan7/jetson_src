#include "hb_robot_perception/scene_model.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/image_encodings.hpp>

namespace hb_perception{

    SceneModel::SceneModel(): scene_cloud_(std::make_shared<PointCloud>()){

    }
    constexpr float MIN_DEPTH = 0.02f;
    // constexpr float MAX_DEPTH = 3.0f;

    SceneModel::PointCloud::Ptr SceneModel::observationToCloud(const Observation& ob,double depth_range)const{
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


        cloud->points.reserve(static_cast<std::size_t>(depth.width)*depth.height);

        auto depth_cv = cv_bridge::toCvShare(ob.depth_to_rgb);
        auto rgb_cv = cv_bridge::toCvShare(ob.rgb);
        const cv::Mat& depth_mat = depth_cv->image;
        const cv::Mat& rgb_mat = rgb_cv->image;

        double min_val;
        double max_val;

        cv::minMaxLoc(depth_mat, &min_val, &max_val);

        std::cout << "depth min/max = "
                << min_val << " / "
                << max_val << std::endl;
        

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
    

        bool SceneModel::addObservation(Observation ob, double depth_range){
            std::cout<<"adding observation"<<std::endl;
        
        const Eigen::Isometry3d T_scene_cam = tf2::transformToEigen(ob.camera_pose);
        
        /*
        * exprss the incoming camera - point cloud to base frame.
        */
        PointCloud transformed_cloud_;
        std::cout<<"could get the camera pose"<<std::endl;
        auto cam_cloud = observationToCloud(ob,depth_range);
        if (!cam_cloud){
            std::cout<<"failed to get observation to cloud"<<std::endl;
        }
        pcl::transformPointCloud(*cam_cloud,transformed_cloud_,T_scene_cam.matrix().cast<float>());
        PointCloud voxel_downsampled_;
        std::cout<< "PointCloud before filtering: "<< transformed_cloud_.points.size()<< " data points"<<std::endl;
        pcl::VoxelGrid<PointT> vox;
        vox.setInputCloud(std::make_shared<PointCloud>(transformed_cloud_));
        vox.setLeafSize(0.005f,0.005f,0.005f);
        vox.filter(voxel_downsampled_);

        std::cout<< "PointCloud after filtering: "<< voxel_downsampled_.points.size()<< " data points"<<std::endl;

        PointCloud valid_cloud;
        valid_cloud.points.reserve(voxel_downsampled_.points.size());
        for (const auto& pt: voxel_downsampled_.points){
            if (!std::isfinite(pt.x)||
            !std::isfinite(pt.y)||
            !std::isfinite(pt.z)){
                continue;
            }
            valid_cloud.points.push_back(pt);

        }

        if(valid_cloud.empty()){
            
            return false;
        
        }
        valid_cloud.width = static_cast<uint32_t>(valid_cloud.points.size());
        valid_cloud.height = 1;
        valid_cloud.is_dense = true;

        {std::lock_guard<std::mutex> lock(scene_mutex);
        *scene_cloud_+=valid_cloud;
        }
        observation_buffer_.addObservation(ob);
        return true;
    }
    
    void SceneModel::downsample(float leafsize){
        std::lock_guard<std::mutex>lock(scene_mutex);
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(scene_cloud_);
        voxel.setLeafSize(leafsize,leafsize,leafsize);
        auto filtered = std::make_shared<PointCloud>();
        voxel.filter(*filtered);
        scene_cloud_ = std::move(filtered);
    }

    void SceneModel::clear(){
        observation_buffer_.clear();
        std::lock_guard<std::mutex>lock(scene_mutex);
        scene_cloud_->clear();
    }

    std::vector<Observation> SceneModel::observations() const{
        return observation_buffer_.observations();
    }

    SceneModel::PointCloud::Ptr SceneModel::cloud() const{
        std::lock_guard<std::mutex>lock(scene_mutex);
        return std::make_shared<PointCloud>(*scene_cloud_);
    }

    std::size_t SceneModel::observationCount() const{
        return observation_buffer_.size();
    }

    std::size_t SceneModel::pointCount() const{
        std::lock_guard<std::mutex>lock(scene_mutex);
        return scene_cloud_->size();
    }





}

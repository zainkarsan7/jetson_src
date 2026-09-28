#include "hb_robot_perception/workpiece_extractor.hpp"
#include <limits.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>


namespace hb_perception{

        using PointT = pcl::PointXYZRGB;
        using PointCloud = pcl::PointCloud<PointT>;
        
        PointCloud::Ptr WorkpieceExtractor::getNearestCluster(const PointCloud::ConstPtr& cloud, bool downsample)const{
            // convenient return if it fails
            PointCloud::Ptr empty =std::make_shared<PointCloud>();

            PointCloud::ConstPtr input;
            if (downsample){
                auto downsampled =std::make_shared<PointCloud>();
                pcl::VoxelGrid<PointT> vox;
                vox.setInputCloud(cloud);
                vox.setLeafSize(0.005f,0.005f,0.005f);
                vox.filter(*downsampled);
                input = std::move(downsampled);
            }

            else{
                input = cloud;
            }
            // make a kdtree:
            auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
            tree->setInputCloud(input);
            std::vector<pcl::PointIndices> clust_ind;
            pcl::EuclideanClusterExtraction<PointT> clustering;
            clustering.setClusterTolerance(0.04);
            clustering.setMinClusterSize(100);
            clustering.setMaxClusterSize(input->size());
            clustering.setSearchMethod(tree);
            clustering.extract(clust_ind);
            if (clust_ind.empty()){
                return empty;
            }

            float best_distance = std::numeric_limits<float>::max();
            const pcl::PointIndices* best_ = nullptr;
            for (const auto & ind : clust_ind){
                Eigen::Vector4d centroid;
                pcl::compute3DCentroid(*input, ind.indices,centroid);
                const float dist = centroid.head<3>().norm();
                if (dist<best_distance){
                    best_distance = dist;
                    best_ = &ind;
                }
            }
            if(!best_){
                return empty;
            }
            auto result = std::make_shared<PointCloud>();
            result->points.reserve(best_->indices.size());
            for (const int index: best_->indices){
                result->points.push_back(input->points[index]);
            }
            result->width = static_cast<uint32_t>(result->points.size());
            result->height = 1;
            result->is_dense = true;
            return result;

            
        }

        bool WorkpieceExtractor::estimatePrincipalGeometry(
            const PointCloud::ConstPtr& cloud,
            WorkpieceModel& workpiece) const{


            Eigen::Vector4f centroid4;
            pcl::compute3DCentroid(*cloud,centroid4);
            Eigen::Matrix3f cov;
            pcl::computeCovarianceMatrixNormalized(*cloud,centroid4,cov);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
            if(solver.info()!=Eigen::Success){
                return false;
            }
            workpiece.centroid = centroid4.head<3>();
            workpiece.eigs << solver.eigenvalues()(2), 
            solver.eigenvalues()(1), 
            solver.eigenvalues()(0);

            workpiece.p_axes.col(0)=solver.eigenvectors().col(2).normalized();
            workpiece.p_axes.col(1)=solver.eigenvectors().col(1).normalized();
            workpiece.p_axes.col(2)=solver.eigenvectors().col(0).normalized();

            workpiece.l_axes = workpiece.p_axes.col(0);

            return true;
        }

        std::optional<WorkpieceModel> WorkpieceExtractor::extract(
            const PointCloud::ConstPtr &cloud) const{
            if(!cloud || cloud->empty()){
                return std::nullopt;
            }
            auto workpiece_pc = getNearestCluster(cloud,true);
            if(!workpiece_pc || workpiece_pc->empty()){
                return std::nullopt;
            }
            WorkpieceModel workpiece;
            workpiece.cloud = workpiece_pc;

            if(!estimatePrincipalGeometry(workpiece.cloud,workpiece)){
                return std::nullopt;
            }
            return workpiece;

        }




}
#include "hb_robot_perception/workpiece_extractor.hpp"
#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include <limits.h>
#include <array>
#include <cmath>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/point_tests.h>

namespace hb_perception{


AxisValidation validateAxis(
    const WorkpieceExtractor& extractor,
    const ProfileMatcher& matcher,
    const WorkpieceModel& model,
    const std::vector<ProfileModel>& profiles)
{
    AxisValidation result;
    result.axis = model.l_axes;

    constexpr std::array<float, 5> offsets{
        -0.06f, -0.03f, 0.0f, 0.03f, 0.06f
    };

    float rms_sum = 0.0f;
    float inlier_sum = 0.0f;

    for (const float s : offsets)
    {
        const auto section =
            extractor.extractSection(model, s, 0.015f);

        if (!section || section->points_2d.size() < 30)
            continue;

        const auto matches =
            matcher.match(*section, profiles);

        if (matches.empty())
            continue;

        const auto& best = matches.front();

        if (!std::isfinite(best.rms_dist))
            continue;

        ++result.valid_sections;
        rms_sum += best.rms_dist;
        inlier_sum += best.inlier_fraction;
    }

    if (result.valid_sections == 0)
        return result;

    result.mean_rms =
        rms_sum / result.valid_sections;

    result.mean_inlier_fraction =
        inlier_sum / result.valid_sections;

    // Diagnostic ranking, not yet a calibrated confidence score.
    result.score =
        result.mean_rms
        + 0.01f * (1.0f - result.mean_inlier_fraction)
        + 0.01f * (5 - result.valid_sections);

    return result;
}

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
            clustering.setInputCloud(input);
            clustering.setClusterTolerance(0.04);
            clustering.setMinClusterSize(100);
            clustering.setMaxClusterSize(input->size());
            clustering.setSearchMethod(tree);
            clustering.extract(clust_ind);
            if (clust_ind.empty()){
                return empty;
            }

            float best_distance = std::numeric_limits<float>::max();
            float best_cluster_score = std::numeric_limits<float>::max();

            const pcl::PointIndices* best_ = nullptr;
            for (const auto & ind : clust_ind){
                Eigen::Vector4d centroid;
                pcl::compute3DCentroid(*input, ind.indices,centroid);
                const float dist = centroid.head<3>().norm();
                /// augment the distance metric here:
                float size_metric = static_cast<float>(input->points.size())/static_cast<float>(ind.indices.size());
                float cluster_score = dist + 0.1 * size_metric;
                // if (dist<best_distance){
                //     best_distance = dist;
                //     best_ = &ind;
                // }

                 if (cluster_score<best_cluster_score){
                    best_cluster_score = cluster_score;
                    best_ = &ind;
                }
            }
            if(!best_){
                return empty;
            }
            auto result = std::make_shared<PointCloud>();
            result->points.reserve(best_->indices.size());
            for (const int index: best_->indices){
                auto colored_pt = input->points[index];
                colored_pt.r = 255;
                colored_pt.g = 0;
                colored_pt.b = 0;

                result->points.push_back(colored_pt);
            }
            result->width = static_cast<uint32_t>(result->points.size());
            result->height = 1;
            result->is_dense = true;
            return result;

            
        }

        Eigen::Vector3f WorkpieceExtractor::estimateAxisFromNormals(
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
            float radius
        )const{
            std::vector<Eigen::Vector3f> normals;
            if(!cloud || cloud->size()<30){
                return Eigen::Vector3f::Zero();
            }
            pcl::NormalEstimationOMP<PointT, pcl::Normal>estimator;
            estimator.setNumberOfThreads(4);
            auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
            estimator.setSearchMethod(tree);
            estimator.setInputCloud(cloud);
            estimator.setRadiusSearch(radius);
            pcl::PointCloud<pcl::Normal>output;
            estimator.compute(output);
            normals.reserve(output.size());
            

            Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
            for (const auto & n : output.points){

               Eigen::Vector3f v(n.normal_x,n.normal_y,n.normal_z);
               if(!v.allFinite()||v.norm()<1e-5){
                continue;
               }
               const auto unit = v.normalized();
               M.noalias() += unit * unit.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>eig(M);
            return eig.eigenvectors().col(0).normalized();
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

        WorkpieceModel WorkpieceExtractor::withAxis(const WorkpieceModel& source, Eigen::Vector3f axis)const{
            WorkpieceModel result = source;
            axis.normalize();
            if(axis.dot(source.l_axes)<0.0f){
                axis=-axis;
            }
            Eigen::Vector3f x = source.p_axes.col(1);
            x -= x.dot(axis)*axis;

            if(x.norm() < 1e-4f){
                x = axis.unitOrthogonal();

            }
            x.normalize();

            const Eigen::Vector3f y = axis.cross(x).normalized();
            result.l_axes = axis;
            result.p_axes.col(0) = axis;
            result.p_axes.col(1) = x;
            result.p_axes.col(2) = y;

            return result;


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


        ///SECTION STUFF
        std::optional<SectionModel> WorkpieceExtractor::extractSection(const WorkpieceModel& model, 
        float long_pos,
        float thickness)const{
            if(!model.cloud || model.cloud->empty()){
                std::cout<<"either workpiece or cloud is empty"<<std::endl;
                return std::nullopt;
            }


            Eigen::Vector3f section_point = Eigen::Vector3f::Zero();
            try{
                std::cout<<"model l axes norm : "<<model.l_axes.norm()<<std::endl;
               section_point = long_pos * model.l_axes.normalized() + model.centroid;
            }
            catch(std::exception &e){
                std::cout<<"couldnt calculate section_point"<<std::endl;
                std::cout<<e.what()<<std::endl;
                return std::nullopt;
            }
            std::cout<<"section point is x: "<<section_point.x()<<" y:"<<section_point.y()<<" z:"<<section_point.z()<<std::endl;
            auto section = extractSection(model,section_point,thickness);

            if(section){
                section->longitudinal_position = long_pos;
                return section;
            }
            return std::nullopt;
            
        }
        std::optional<SectionModel> WorkpieceExtractor::extractSection(const WorkpieceModel& model, 
        const Eigen::Vector3f& section_point,
        float thickness)const{
            if(!model.cloud || model.cloud->empty()){
                std::cout<<"either workpiece or cloud is empty"<<std::endl;
                return std::nullopt;
            }
            if(thickness <=0.0f){
                std::cout<<"thikcness is zeor"<<std::endl;
                return std::nullopt;
            }
            SectionModel section_model_;
            auto section_cloud_ = std::make_shared<PointCloud>();
            section_cloud_->reserve(model.cloud->points.size());
            Eigen::Vector3f dir = model.l_axes;
            if(dir.norm() < 1e-6f){
                std::cout<<"long axis is fucked"<<std::endl;
                return std::nullopt;
            }
            dir.normalize();
            std::cout<<"culling points"<<std::endl;
            for (auto mp: model.cloud->points){
                if(!pcl::isFinite(mp)){
                    continue;
                }
                Eigen::Vector3f pt(mp.x,mp.y,mp.z);
                float dist = (pt - section_point).dot(dir);
                if (std::abs(dist) <= thickness/2.0){
                    section_cloud_->points.push_back(mp);
                }
            }
            if (section_cloud_->empty()){
                std::cout<<"empty fucking section"<<std::endl;
                return std::nullopt;
            }
            section_cloud_->width = static_cast<uint32_t>(section_cloud_->points.size());
            section_cloud_->height = 1;
            section_cloud_->is_dense= true;


            section_model_.cloud = section_cloud_;
            section_model_.origin = section_point;
            section_model_.normal = dir;
            section_model_.thickness = thickness;
            section_model_.frame = makeSectionFrame(model,section_point);
            section_model_.longitudinal_position = (section_point - model.centroid).dot(dir);
            
            // scene to section transform
            const Eigen::Isometry3f scene_to_section = section_model_.frame.inverse();
            section_model_.points_2d.reserve(section_cloud_->points.size());
            for (const auto& sp: section_cloud_->points){
                Eigen::Vector3f pt(sp.x,sp.y,sp.z);
                Eigen::Vector3f t_pt = scene_to_section * pt;
                section_model_.points_2d.emplace_back(t_pt.x(),t_pt.y());
            }
            
            return section_model_;

        }

        Eigen::Isometry3f WorkpieceExtractor::makeSectionFrame(const WorkpieceModel &model, 
            const Eigen::Vector3f &origin)const{

                Eigen::Vector3f z_ax = model.l_axes.normalized();
                Eigen::Vector3f x_ax = model.p_axes.col(1).normalized(); //the second largest eigV
                x_ax -= x_ax.dot(z_ax)*z_ax; //pointless subtraction 
                if(x_ax.norm()<1e-6f){
                    x_ax = z_ax.unitOrthogonal(); // weird trick to get a random x axis just in case
                }
                x_ax.normalize();
                Eigen::Vector3f y_ax = z_ax.cross(x_ax).normalized();
                x_ax = y_ax.cross(z_ax).normalized();
                Eigen::Matrix3f Rot;
                Rot.col(0) = x_ax;
                Rot.col(1) = y_ax;
                Rot.col(2) = z_ax;
                Eigen::Isometry3f frame = Eigen::Isometry3f::Identity();
                frame.translation() = origin;
                frame.linear() = Rot;
                return frame;




        }



}
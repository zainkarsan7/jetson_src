#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace hb_perception{
namespace{
    constexpr float pi = 3.141592653589f;

    float norm_angle (float angle){
        while (angle > pi){
            angle -= 2.0*pi;
        }

        while(angle < -pi){
            angle += 2.0*pi;
        }
        return angle;
    }
}

ProfileMatcher::ProfileMatcher(): params_(){}


ProfileMatcher::ProfileMatcher(const Parameters& params):params_(params){}

std::vector<ProfileMatch> ProfileMatcher::match(
                const SectionModel& section,
                const std::vector<ProfileModel>& candidates
            ) const{
                std::vector<ProfileMatch> results;
                if (section.points_2d.empty()){
                    return results;
                }
                results.reserve(candidates.size());
                for (const auto& cand : candidates){
                    if(cand.boundary.empty()){
                        std::cout<<cand.name<<" has empty profile"<<std::endl;
                        continue;
                    }

                    auto result = fitCand(section,cand);
                    results.emplace_back(result);

                }
                std::sort(results.begin(),results.end(),
            [](const ProfileMatch& a , const ProfileMatch& b){

                if(a.rms_dist != b.rms_dist){
                    return a.rms_dist<b.rms_dist;
                }
                //if cost leasds ot tie break yea right. 
                return a.inlier_fraction>b.inlier_fraction;


            });

            return results;

            }

ProfileMatch ProfileMatcher::fitCand(const SectionModel& section, 
            const ProfileModel& profile)const {

                ProfileMatch best;
                best.profile = profile;

                const float tmax = params_.max_translation;
                const float rot_max = params_.max_rotation;
                const float tstep = params_.translation_step;
                const float rstep = params_.rotation_step;


                for(float theta = -rot_max; theta<rot_max; theta+=rstep){
                    const Eigen::Rotation2Df rotation(theta);
                    for (float tx = - tmax; tx<tmax; tx += tstep){
                        for (float ty = -tmax; ty<tmax; ty+=tstep){
                            Eigen::Isometry2f transform = Eigen::Isometry2f::Identity();
                            transform.linear() = rotation.toRotationMatrix();
                            transform.translation() = Eigen::Vector2f(tx,ty);
                            ProfileMatch current = evalTransform(section,profile,transform);
                            if(current.rms_dist<best.rms_dist){
                                best = std::move(current);
                            }
                        }
                    }
                }
                return best;
                
            }

ProfileMatch ProfileMatcher::evalTransform(const SectionModel& section,
            const ProfileModel& profile,
        const Eigen::Isometry2f& transform) const{

            ProfileMatch result;
            result.profile = profile;
            result.transform = transform;
            if (section.points_2d.empty()){ return result;}
            const float count = static_cast<float>(section.points_2d.size());
            result.residuals.reserve(section.points_2d.size());
            const float truncation = params_.truncation_distance;
            const float sq_truncation = truncation * truncation;
            float sq_error = 0.0f;
            std::size_t inlier_count = 0;

            for (const auto sp: section.points_2d){
                const float dist_err = pointToProfileDistance(sp,profile,transform);
                result.residuals.push_back(dist_err);
                const float sq_dist = dist_err * dist_err;
                sq_error += std::min(sq_dist,sq_truncation);
                if(dist_err <= params_.inlier_tolerance){
                    ++inlier_count;
                }
            }

            result.rms_dist = std::sqrt(sq_error/count);
            result.inlier_fraction = static_cast<float>(inlier_count)/count;
            const float residual_score = std::max(0.0f,1.0f - (result.rms_dist/truncation));

            result.score = residual_score * result.inlier_fraction;
            return result;
        }

float ProfileMatcher::pointToProfileDistance(
                const Eigen::Vector2f& point,
                const ProfileModel& profile,
                const Eigen::Isometry2f& profile_to_section) const{
                    /**
                     * transform section points to the profile coordinate.
                     *
                     */
                     const Eigen::Vector2f profile_point = profile_to_section.inverse() * point;
                     float min_dist = std::numeric_limits<float>::infinity();
                     // iterate over each line segment

                     for (const auto& prim: profile.boundary){
                        if(const auto* line = std::get_if<LineSegment2D>(&prim)){
                            float dist = pointToLineSegmentDistance(profile_point,*line);
                            min_dist = std::min(min_dist,dist);
                        }
                        
                     }
                     return min_dist;
                }

float ProfileMatcher::pointToLineSegmentDistance(
                const Eigen::Vector2f& point,
                const LineSegment2D& segment
            ){

                const Eigen::Vector2f ab = segment.b -segment.a;
                const float sq_length = ab.squaredNorm();

                if(sq_length < 1e-12f){
                    return (point-segment.a).norm();
                }

                float t = (point - segment.a).dot(ab) / sq_length;
                t = std::clamp(t,0.0f,1.0f);

                const Eigen::Vector2f nearest = segment.a + t* ab;
                return (point- nearest).norm();

            }

            
visualization_msgs::msg::MarkerArray ProfileMatcher::getVisualization(
    const SectionModel& section,
    const std::vector<ProfileMatch>& matches,
    const std::string& frame_id,
    std::size_t max_matches) const
{
    visualization_msgs::msg::MarkerArray array;

    int id = 0;

    /*
     * Convert section-local 2-D coordinate into the
     * scene/world coordinate system.
     *
     * Section convention:
     *
     *   x,y = section plane
     *   z   = longitudinal axis
     */
    auto sectionToWorld =
        [&section](
            const Eigen::Vector2f& p,
            float longitudinal_offset = 0.0f)
        {
            return section.frame *
                Eigen::Vector3f(
                    p.x(),
                    p.y(),
                    longitudinal_offset);
        };


    /*
     * ------------------------------------------------------
     * Observed section points
     * ------------------------------------------------------
     */

    visualization_msgs::msg::Marker observed;

    observed.header.frame_id = frame_id;
    observed.ns = "section_observed";
    observed.id = id++;

    observed.type =
        visualization_msgs::msg::Marker::POINTS;

    observed.action =
        visualization_msgs::msg::Marker::ADD;

    observed.pose.orientation.w = 1.0;

    observed.scale.x = 0.003;
    observed.scale.y = 0.003;

    observed.color.r = 1.0f;
    observed.color.g = 1.0f;
    observed.color.b = 1.0f;
    observed.color.a = 1.0f;


    for (const auto& p : section.points_2d)
    {
        const Eigen::Vector3f world =
            sectionToWorld(p);

        geometry_msgs::msg::Point msg;

        msg.x = world.x();
        msg.y = world.y();
        msg.z = world.z();

        observed.points.push_back(msg);
    }

    array.markers.push_back(
        std::move(observed));


    /*
     * ------------------------------------------------------
     * Profile hypotheses
     * ------------------------------------------------------
     *
     * Best profile lies directly on section.
     *
     * Subsequent matches are shifted along section Z
     * so they can be inspected individually.
     */

    const std::size_t count =
        std::min(
            max_matches,
            matches.size());

    constexpr float hypothesis_spacing =
        0.040f; // 40 mm


    for (std::size_t rank = 0;
         rank < count;
         ++rank)
    {
        const auto& match =
            matches[rank];

        const float z_offset =
            static_cast<float>(rank) *
            hypothesis_spacing;


        /*
         * -------- profile boundary --------
         */

        visualization_msgs::msg::Marker boundary;

        boundary.header.frame_id = frame_id;

        boundary.ns =
            "profile_boundary";

        boundary.id = id++;

        boundary.type =
            visualization_msgs::msg::Marker::LINE_LIST;

        boundary.action =
            visualization_msgs::msg::Marker::ADD;

        boundary.pose.orientation.w = 1.0;

        boundary.scale.x = 0.002;


        /*
         * Best candidate green.
         *
         * Remaining candidates orange-ish.
         */
        if (rank == 0)
        {
            boundary.color.r = 0.1f;
            boundary.color.g = 1.0f;
            boundary.color.b = 0.1f;
        }
        else
        {
            boundary.color.r = 1.0f;
            boundary.color.g = 0.5f;
            boundary.color.b = 0.1f;
        }

        boundary.color.a = 1.0f;

        
        const auto profile_points =
            sampleProfileBoundary(
                match.profile, 0.002f);


        for (const auto& profile_point :
             profile_points)
        {
            /*
             * Apply fitted profile -> section transform.
             */
            const Eigen::Vector2f section_point =
                match.transform *
                profile_point;

            const Eigen::Vector3f world =
                sectionToWorld(
                    section_point,
                    z_offset);

            geometry_msgs::msg::Point msg;

            msg.x = world.x();
            msg.y = world.y();
            msg.z = world.z();

            boundary.points.push_back(msg);
        }

        array.markers.push_back(
            std::move(boundary));


        /*
         * -------- label --------
         */

        visualization_msgs::msg::Marker label;

        label.header.frame_id = frame_id;

        label.ns =
            "profile_labels";

        label.id = id++;

        label.type =
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

        label.action =
            visualization_msgs::msg::Marker::ADD;

        label.pose.orientation.w = 1.0;


        /*
         * Put label just above the profile.
         */
        const Eigen::Vector2f label_2d(
            0.0f,
            0.5f * match.profile.height +
                0.025f);

        const Eigen::Vector3f label_world =
            sectionToWorld(
                match.transform * label_2d,
                z_offset);

        label.pose.position.x =
            label_world.x();

        label.pose.position.y =
            label_world.y();

        label.pose.position.z =
            label_world.z();


        label.scale.z = 0.015;

        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;


        std::ostringstream ss;

        ss << "#" << rank
           << " "
           << match.profile.name
           << "  rms="
           << match.rms_dist * 1000.0f
           << "mm"
           << "  support="
           << match.inlier_fraction * 100.0f
           << "%";

        label.text = ss.str();

        array.markers.push_back(
            std::move(label));
    }


    return array;
}
   

std::vector<Eigen::Vector2f>
ProfileMatcher::sampleProfileBoundary(
    const ProfileModel& profile,
    float arc_resolution)
{
    std::vector<Eigen::Vector2f> points;

    for (const auto& primitive : profile.boundary)
    {
        if (const auto* line =
                std::get_if<LineSegment2D>(&primitive))
        {
            // Two points are sufficient for LINE_LIST.
            points.push_back(line->a);
            points.push_back(line->b);
        }
        else if (const auto* arc =
                     std::get_if<ArcSegment2D>(&primitive))
        {
            if (arc->radius <= 0.0f) {
                continue;
            }

            float span =
                norm_angle(
                    arc->end -
                    arc->start);

            // Assume arcs describe the short path for now.
            const float arc_length =
                std::abs(span) * arc->radius;

            const int samples =
                std::max(
                    2,
                    static_cast<int>(
                        std::ceil(
                            arc_length /
                            arc_resolution)));

            Eigen::Vector2f previous =
                arc->center +
                arc->radius *
                Eigen::Vector2f(
                    std::cos(arc->start),
                    std::sin(arc->start));

            for (int i = 1; i <= samples; ++i)
            {
                const float alpha =
                    static_cast<float>(i) /
                    static_cast<float>(samples);

                const float angle =
                    arc->start +
                    alpha * span;

                const Eigen::Vector2f current =
                    arc->center +
                    arc->radius *
                    Eigen::Vector2f(
                        std::cos(angle),
                        std::sin(angle));

                // Store as LINE_LIST pair.
                points.push_back(previous);
                points.push_back(current);

                previous = current;
            }
        }
    }

    return points;
}



}

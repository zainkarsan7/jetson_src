#include "hb_robot_perception/profile_matcher.hpp"
#include "hb_robot_perception/profile_types.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <limits>
#include <unordered_set>
#include <vector>
#include <iostream>



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


/*
 * Cheap 2-D voxel/grid downsampling.
 *
 * We only need representative section points for profile matching.
 * There is no reason to score thousands of Kinect points lying
 * within a few millimetres of one another.
 */
std::vector<Eigen::Vector2f> downsample2D(
    const std::vector<Eigen::Vector2f>& input,
    float resolution)
{
    if (input.empty() || resolution <= 0.0f) {
        return input;
    }

    struct CellHash
    {
        std::size_t operator()(
            const std::pair<int, int>& p) const
        {
            const std::size_t h1 =
                std::hash<int>{}(p.first);

            const std::size_t h2 =
                std::hash<int>{}(p.second);

            return h1 ^
                (h2 + 0x9e3779b9 +
                 (h1 << 6) +
                 (h1 >> 2));
        }
    };

    std::unordered_set<
        std::pair<int, int>,
        CellHash> occupied;

    std::vector<Eigen::Vector2f> output;
    output.reserve(input.size());

    const float inv =
        1.0f / resolution;

    for (const auto& p : input)
    {
        if (!p.allFinite()) {
            continue;
        }

        const int ix =
            static_cast<int>(
                std::floor(p.x() * inv));

        const int iy =
            static_cast<int>(
                std::floor(p.y() * inv));

        const std::pair<int, int> key(ix, iy);

        if (occupied.insert(key).second) {
            output.push_back(p);
        }
    }

    return output;
}

Eigen::Vector2f
trimmedSectionCenter(
    const SectionModel& section)
{
    std::vector<float> xs;
    std::vector<float> ys;

    xs.reserve(section.points_2d.size());
    ys.reserve(section.points_2d.size());

    for (const auto& p : section.points_2d)
    {
        if (!p.allFinite()) {
            continue;
        }

        xs.push_back(p.x());
        ys.push_back(p.y());
    }

    if (xs.empty()) {
        return Eigen::Vector2f::Zero();
    }

    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    const std::size_t lo =
        static_cast<std::size_t>(
            0.05 * static_cast<double>(xs.size() - 1));

    const std::size_t hi =
        static_cast<std::size_t>(
            0.95 * static_cast<double>(xs.size() - 1));

    return Eigen::Vector2f(
        0.5f * (xs[lo] + xs[hi]),
        0.5f * (ys[lo] + ys[hi]));
}

/*
 * Get an approximate profile bounding box.
 *
 * For the current IPN models this can simply use the known
 * nominal width/height.
 */
bool insideExpandedProfileBounds(
    const Eigen::Vector2f& p,
    const ProfileModel& profile,
    float margin)
{
    const float hx =
        0.5f * profile.width + margin;

    const float hy =
        0.5f * profile.height + margin;

    return
        std::abs(p.x()) <= hx &&
        std::abs(p.y()) <= hy;
}






ProfileMatcher::ProfileMatcher(): params_(){}


ProfileMatcher::ProfileMatcher(const Parameters& params):params_(params){}

Eigen::Isometry3f ProfileMatcher::makeProfilePose(const SectionModel& section, const ProfileMatch& match){
    // convert the match 2d frame into a 3d frame and get it relative to world:
    Eigen::Isometry3f T_section_profile = Eigen::Isometry3f::Identity();

    T_section_profile.linear().block<2,2>(0,0) = match.transform.linear();
    T_section_profile.translation().x() = match.transform.translation().x();
    T_section_profile.translation().y() = match.transform.translation().y();
    T_section_profile.translation().z() = 0.0f;
    return section.frame * T_section_profile;
}

hb_robot_interfaces::msg::ProfileEstimate ProfileMatcher::getProfileEstimateMsg(const std::string& planning_frame, const SectionModel& section, const ProfileMatch& match){
    hb_robot_interfaces::msg::ProfileEstimate msg;
    msg.header.frame_id = planning_frame;
    msg.pose = tf2::toMsg(static_cast<Eigen::Isometry3d>(makeProfilePose(section,match)));
    msg.profile_name = match.profile.name;
    msg.inlier_fraction = match.inlier_fraction;
    msg.rms_distance = match.rms_dist;
    msg.score = match.score;
    msg.section_thickness = section.thickness;
    msg.observation_points_count = section.points_2d.size();
    


    

    return msg;
}

std::vector<ProfileMatch> ProfileMatcher::match(
                const SectionModel& section,
                const std::vector<ProfileModel>& candidates
            ) const{
                std::vector<ProfileMatch> results;
                if (section.points_2d.empty()){
                    return results;
                }

                SectionModel reduced_section = section;
                reduced_section.points_2d = downsample2D(section.points_2d,0.003f);

                 std::cout
                    << "ProfileMatcher: "
                    << section.points_2d.size()
                    << " -> "
                    << reduced_section.points_2d.size()
                    << " section points"
                    << std::endl;

                results.reserve(candidates.size());

                for (const auto& cand : candidates){
                    if(cand.boundary.empty()){
                        std::cout<<cand.name<<" has empty profile"<<std::endl;
                        continue;
                    }
                    results.emplace_back(
                                fitCand(
                                    reduced_section,
                                    cand));
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


/*
 * Fast scalar cost evaluation.
 *
 * This is intentionally separate from evalTransform().
 *
 * During optimization we DON'T need:
 *
 *   - residual vector
 *   - ProfileMatch
 *   - profile copy
 *   - inlier fraction
 *
 * We only need one scalar saying whether this transform
 * is better than the previous transform.
 */
float ProfileMatcher::transformCost(
    const SectionModel& section,
    const ProfileModel& profile,
    const Eigen::Isometry2f& transform) const
{
    if (section.points_2d.empty()) {
        return std::numeric_limits<float>::infinity();
    }
     /*
     * Instead of inverse() for every individual point,
     * invert the transform ONCE.
     */
    const Eigen::Isometry2f section_to_profile =
        transform.inverse();

     const float truncation =
        params_.truncation_distance;

    const float sq_truncation =
        truncation * truncation;

    float sq_error = 0.0f;

      /*
     * Keep track of useful points separately.
     *
     * Gross scene/section outliers should not dominate
     * the search.
     */
    std::size_t evaluated = 0;


    for (const auto& section_point: section.points_2d){
        const Eigen::Vector2f p = section_to_profile * section_point;
        /*
         * Extremely cheap rejection.
         *
         * A point far outside the candidate's nominal bounding
         * box cannot produce a useful correspondence.
         *
         * Give it truncated cost without walking every
         * profile primitive.
         */
        if (!insideExpandedProfileBounds(
                p,
                profile,
                truncation))
        {
            sq_error += sq_truncation;
            ++evaluated;
            continue;
        }

         float min_dist =
            std::numeric_limits<float>::infinity();

         for (const auto& prim :
             profile.boundary)
        {
            if(const auto* line = std::get_if<LineSegment2D>(&prim)){
                const float dist = pointToLineSegmentDistance(p,*line);
                min_dist =
                    std::min(
                        min_dist,
                        dist);

                /*
                 * Can't improve meaningfully once we're
                 * essentially on the profile.
                 */
                if (min_dist < 0.0005f) {
                    break;
                }
            }
        }
        if (!std::isfinite(min_dist)) {
            continue;
        }
        const float sq_dist =
            min_dist * min_dist;

        sq_error +=
            std::min(
                sq_dist,
                sq_truncation);

        ++evaluated;
    }
    if (evaluated == 0) {
        return std::numeric_limits<float>::infinity();
    }

    return std::sqrt(
        sq_error /
        static_cast<float>(evaluated));
}
std::vector<Eigen::Isometry2f>
ProfileMatcher::canonicalTransforms(
    const SectionModel& section) const
{
    std::vector<Eigen::Isometry2f> transforms;

    if (section.points_2d.empty()) {
        return transforms;
    }

    /*
     * -------------------------------------------------------
     * Robust-ish initial translation.
     * -------------------------------------------------------
     *
     * Do not assume SectionModel origin == profile center.
     *
     * Using the midpoint of the observed extents is deliberately
     * simple. It is much less sensitive than assuming (0,0), and
     * unlike the mean it isn't weighted by point density.
     */
    // Eigen::Vector2f min_pt =
    //     section.points_2d.front();

    // Eigen::Vector2f max_pt =
    //     section.points_2d.front();

    // for (const auto& p : section.points_2d)
    // {
    //     if (!p.allFinite()) {
    //         continue;
    //     }

    //     min_pt =
    //         min_pt.cwiseMin(p);

    //     max_pt =
    //         max_pt.cwiseMax(p);
    // }

    // const Eigen::Vector2f center =
    //     0.5f * (min_pt + max_pt);

    const Eigen::Vector2f center =
    trimmedSectionCenter(section);

    /*
     * -------------------------------------------------------
     * Canonical orientations.
     * -------------------------------------------------------
     *
     * PCA may give:
     *
     *   X,Y
     *  -X,Y
     *   X,-Y
     *  -X,-Y
     *
     * and may interchange the two in-plane axes.
     *
     * Quarter-turn seeds deal with the axis interchange/sign
     * ambiguity for ordinary rotations.
     */
    constexpr float canonical_angles[] =
    {
        0.0f,
        0.5f * pi,
        pi,
        1.5f * pi
    };


    for (const float angle : canonical_angles)
    {
        Eigen::Isometry2f transform =
            Eigen::Isometry2f::Identity();

        transform.linear() =
            Eigen::Rotation2Df(angle)
                .toRotationMatrix();

        transform.translation() =
            center;

        transforms.push_back(transform);
    }


    /*
     * -------------------------------------------------------
     * Reflected canonical hypotheses.
     * -------------------------------------------------------
     *
     * A SectionModel frame constructed from independently
     * signed PCA vectors can effectively leave us with a
     * reflected 2-D representation relative to the nominal
     * profile convention.
     *
     * Test those explicitly rather than making the optimizer
     * somehow discover a reflection.
     */
    Eigen::Matrix2f reflection =
        Eigen::Matrix2f::Identity();

    reflection(0, 0) = -1.0f;


    for (const float angle : canonical_angles)
    {
        Eigen::Isometry2f transform =
            Eigen::Isometry2f::Identity();

        transform.linear() =
            Eigen::Rotation2Df(angle)
                .toRotationMatrix() *
            reflection;

        transform.translation() =
            center;

        transforms.push_back(transform);
    }


    return transforms;
}

ProfileMatch ProfileMatcher::fitCand(const SectionModel& section, 
            const ProfileModel& profile)const {

    ProfileMatch best;

    best.profile = profile;


    const auto seeds =
        canonicalTransforms(section);

    if (seeds.empty()) {
        return best;
    }            

    float best_cost =
        std::numeric_limits<float>::infinity();

    Eigen::Isometry2f best_transform =
        Eigen::Isometry2f::Identity();


    /*
     * =======================================================
     * STAGE 1:
     * Coarse search around every canonical hypothesis.
     * =======================================================
     */

    const float coarse_tstep =
        std::max(
            params_.translation_step * 2.0f,
            0.010f);

    const float coarse_rstep =
        std::max(
            params_.rotation_step * 2.0f,
            4.0f * pi / 180.0f);


    for (const auto& seed : seeds)
    {
        for (float dtheta = -params_.max_rotation;
             dtheta <= params_.max_rotation + 1e-6f;
             dtheta += coarse_rstep)
        {
            const Eigen::Matrix2f local_rotation =
                Eigen::Rotation2Df(dtheta)
                    .toRotationMatrix();


            for (float dx = -params_.max_translation;
                 dx <= params_.max_translation + 1e-6f;
                 dx += coarse_tstep)
            {
                for (float dy = -params_.max_translation;
                     dy <= params_.max_translation + 1e-6f;
                     dy += coarse_tstep)
                {
                    /*
                     * Start from canonical hypothesis.
                     */
                    Eigen::Isometry2f candidate =
                        seed;

                    /*
                     * Apply local rotational correction while
                     * preserving a possible reflection in seed.
                     */
                    candidate.linear() =
                        local_rotation *
                        seed.linear();

                    /*
                     * Translation search is in section
                     * coordinates.
                     */
                    candidate.translation() =
                        seed.translation() +
                        Eigen::Vector2f(dx, dy);


                    const float cost =
                        transformCost(
                            section,
                            profile,
                            candidate);


                    if (cost < best_cost)
                    {
                        best_cost =
                            cost;

                        best_transform =
                            candidate;
                    }
                }
            }
        }
    }


    /*
     * =======================================================
     * STAGE 2:
     * Fine search around the globally best coarse solution.
     * =======================================================
     *
     * Crucially, we don't refine every canonical hypothesis.
     * Only the winner enters this stage.
     */

    const Eigen::Isometry2f coarse_best =
        best_transform;


    const float fine_tstep =
        params_.translation_step;

    const float fine_rstep =
        params_.rotation_step;


    for (float dtheta = -coarse_rstep;
         dtheta <= coarse_rstep + 1e-6f;
         dtheta += fine_rstep)
    {
        const Eigen::Matrix2f correction =
            Eigen::Rotation2Df(dtheta)
                .toRotationMatrix();


        for (float dx = -coarse_tstep;
             dx <= coarse_tstep + 1e-6f;
             dx += fine_tstep)
        {
            for (float dy = -coarse_tstep;
                 dy <= coarse_tstep + 1e-6f;
                 dy += fine_tstep)
            {
                Eigen::Isometry2f candidate =
                    coarse_best;


                candidate.linear() =
                    correction *
                    coarse_best.linear();


                candidate.translation() =
                    coarse_best.translation() +
                    Eigen::Vector2f(dx, dy);


                const float cost =
                    transformCost(
                        section,
                        profile,
                        candidate);


                if (cost < best_cost)
                {
                    best_cost =
                        cost;

                    best_transform =
                        candidate;
                }
            }
        }
    }

    /*
     * Generate residuals / support only once for the final
     * winning transform.
     */
    return evalTransform(
        section,
        profile,
        best_transform);
    }



ProfileMatch ProfileMatcher::evalTransform(const SectionModel& section,
            const ProfileModel& profile,
        const Eigen::Isometry2f& transform) const{

            ProfileMatch result;
            result.profile = profile;
            result.transform = transform;
            if (section.points_2d.empty()){ return result;}
            
            result.residuals.reserve(section.points_2d.size());

            const Eigen::Isometry2f section_to_profile =
        transform.inverse();

            const float truncation = params_.truncation_distance;
            const float sq_truncation = truncation * truncation;
            float sq_error = 0.0f;
            std::size_t inlier_count = 0;
            const float count = static_cast<float>(section.points_2d.size());


            for (const auto& section_point :
                section.points_2d)
            {
                const Eigen::Vector2f profile_point =
            section_to_profile *
            section_point;

              float min_dist =
            std::numeric_limits<float>::infinity();
                if (!insideExpandedProfileBounds(
                profile_point,
                profile,
                truncation))
        {
            min_dist = truncation;
        }
        else{
            for (const auto& prim :
                 profile.boundary)
            {
                if (const auto* line =
                        std::get_if<LineSegment2D>(
                            &prim))
                {
                    const float dist =
                        pointToLineSegmentDistance(
                            profile_point,
                            *line);

                    min_dist =
                        std::min(
                            min_dist,
                            dist);
                }
            }

        }

         if (!std::isfinite(min_dist)) {
            min_dist = truncation;
        }


        result.residuals.push_back(
            min_dist);
        
            sq_error +=
        std::min(
            min_dist * min_dist,
            sq_truncation);

         if (min_dist <=
            params_.inlier_tolerance)
        {
            ++inlier_count;
        }    

    }
    

    result.rms_dist =
        std::sqrt(
            sq_error / count);


    result.inlier_fraction =
        static_cast<float>(
            inlier_count) /
        count;


    const float residual_score =
        std::max(
            0.0f,
            1.0f -
            result.rms_dist /
            truncation);


    result.score =
        residual_score *
        result.inlier_fraction;


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

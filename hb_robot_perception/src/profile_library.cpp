#include "hb_robot_perception/profile_library.hpp"

namespace hb_perception{


    ProfileModel ProfileLibrary::makeIPN(
                const std::string& name,
                float h,
                float b,
                float tw,
                float tf,
                float slope
             ){

                ProfileModel profile;
                profile.name = name;
                profile.height = h;
                profile.width = b;

                const float x_outer = 0.5f * b;
                const float x_web = 0.5f * tw;

                const float y_top = 0.5f * h;
                const float y_bottom = -y_top;

                const float half_flange_run = std::max(0.0f,x_outer-x_web);
                const float flange_rise = slope * half_flange_run;

                const float y_top_inner = y_top - tf;
                const float y_top_web = y_top_inner - flange_rise;

                const float y_bot_inner = y_bottom + tf;
                const float y_bot_web = y_bot_inner + flange_rise;


                auto addLine = [&profile](
                    
                    float x1, float x2, float y1, float y2
                ){


                    LineSegment2D seg;
                    seg.a = Eigen::Vector2f(x1,y1);
                    seg.b = Eigen::Vector2f(x2,y2);
                    profile.boundary.emplace_back(seg);

                    

                };

                /**
                 * order clockwise top around profile
                 */
                //top flange
                addLine(-x_outer,x_outer,y_top,y_top);
                ProfileCutFeature top_flange;
                top_flange.name="top_flange";
                top_flange.type = ProfileCutFeatureType::Flange;
                top_flange.start = Eigen::Vector2f(-x_outer,y_top);
                top_flange.end = Eigen::Vector2f(x_outer,y_top);
                top_flange.outward_normal = Eigen::Vector2f::UnitY();
                profile.cut_features.push_back(top_flange);

                addLine(x_outer,x_outer,y_top,y_top_inner);
                addLine(x_outer,x_web,y_top_inner,y_top_web);

                //right web
                addLine(x_web,x_web,y_top_web,y_bot_web);

                ProfileCutFeature right_web;
                right_web.name="right_web";
                right_web.type = ProfileCutFeatureType::Web;
                right_web.start = Eigen::Vector2f(x_web,y_top_web);
                right_web.end = Eigen::Vector2f(x_web,y_bot_web);
                right_web.outward_normal = Eigen::Vector2f::UnitX();
                profile.cut_features.push_back(right_web);

                addLine(x_web,x_outer,y_bot_web,y_bot_inner);
                addLine(x_outer,x_outer,y_bot_inner,y_bottom);

                //bottom flange
                addLine(x_outer,-x_outer,y_bottom,y_bottom);
                ProfileCutFeature bottom_flange;
                bottom_flange.name="bottom_flange";
                bottom_flange.type = ProfileCutFeatureType::Flange;
                bottom_flange.start = Eigen::Vector2f(x_outer,y_bottom);
                bottom_flange.end = Eigen::Vector2f(-x_outer,y_bottom);
                bottom_flange.outward_normal = -Eigen::Vector2f::UnitY();
                profile.cut_features.push_back(bottom_flange);


                addLine(-x_outer,-x_outer,y_bottom,y_bot_inner);
                addLine(-x_outer,-x_web,y_bot_inner,y_bot_web);

                // left web
                addLine(-x_web,-x_web,y_bot_web,y_top_web);
                ProfileCutFeature left_web;
                left_web.name="left_web";
                left_web.type = ProfileCutFeatureType::Web;
                left_web.start = Eigen::Vector2f(-x_web,y_top_web);
                left_web.end = Eigen::Vector2f(-x_web,y_bot_web);
                left_web.outward_normal = -Eigen::Vector2f::UnitX();
                profile.cut_features.push_back(left_web);


                addLine(-x_web,-x_outer,y_top_web,y_top_inner);
                addLine(-x_outer,-x_outer,y_top_inner,y_top);

                return profile;
             }

    std::vector<ProfileModel> ProfileLibrary::ipnProfiles(){
        std::vector<ProfileModel>profiles;

        // profiles.emplace_back(
        // makeIPN(
        //     "IPN_120",
        //     0.120f,
        //     0.058f,
        //     0.0051f,
        //     0.0077f));

        // profiles.emplace_back(
        //     makeIPN(
        //         "IPN_140",
        //         0.140f,
        //         0.066f,
        //         0.0057f,
        //         0.0086f));

        profiles.emplace_back(
            makeIPN(
                "IPN_160",
                0.160f,
                0.074f,
                0.0063f,
                0.0095f));

        // profiles.emplace_back(
        //     makeIPN(
        //         "IPN_180",
        //         0.180f,
        //         0.082f,
        //         0.0069f,
        //         0.0104f));

        // profiles.emplace_back(
        //     makeIPN(
        //         "IPN_200",
        //         0.200f,
        //         0.090f,
        //         0.0075f,
        //         0.0113f));
        return profiles;

    };   
    
    std::optional<ProfileModel> ProfileLibrary::find(const std::string& name){
        const auto profiles = ipnProfiles();
        const auto it = std::find_if(profiles.begin(),profiles.end(),[&name](const ProfileModel& p){
            return p.name==name;
        });
        if (it==profiles.end()){
            return std::nullopt;
        }
        return *it;
    }


}
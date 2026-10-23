#pragma once

#include <optional>
#include <shape_msgs/msg/mesh.hpp>
#include "hb_robot_perception/profile_library.hpp"

namespace hb_perception{

    class ProfileMeshBuilder{
        public:
        std::optional<shape_msgs::msg::Mesh> extrude(
            const ProfileModel& profile,
            double z_min,
            double z_max
        )const;
    };

}


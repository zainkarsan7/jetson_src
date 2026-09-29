#pragma once
#include "hb_robot_perception/profile_types.hpp"
#include <vector>


namespace hb_perception{

    class ProfileLibrary{

        public:

            /**
             * set of ipn profiles
             */
            static std::vector<ProfileModel> ipnProfiles();

            /**
             * function to make ipn profiles
             * h -> height, b -> width, tw -> web thick, tf -> flange thick
             */

             static ProfileModel makeIPN(
                const std::string& name,
                float h,
                float b,
                float tw,
                float tf,
                float slope=0.14f
             );

    };




}
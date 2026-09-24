#pragma once

#include "hb_robot_perception/perception_types.hpp"


namespace hb_perception{

class ObservationBuffer{
    public:
        void addObservation(Observation obs);
        void clear();
        const std::vector<Observation>& observations() const;
        std::size_t size() const;
        void registerObservations();
        
    private:

        std::vector<Observation> observations_;
};
}
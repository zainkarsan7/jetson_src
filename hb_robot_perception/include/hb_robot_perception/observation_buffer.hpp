#pragma once
#include <cstddef>
#include <mutex>
#include <optional>
#include <vector>
#include "hb_robot_perception/perception_types.hpp"


namespace hb_perception{

class ObservationBuffer{
    public:

        ObservationBuffer() = default;

        void addObservation(Observation ob);
        void clear();
        bool empty() const;
        std::optional<Observation>get(std::size_t index) const;
        std::optional<Observation>get_latest() const;
        const std::vector<Observation>& observations() const;
        std::size_t size() const;
        
    private:
        mutable std::mutex mutex_;
        std::vector<Observation> observations_;
};
}
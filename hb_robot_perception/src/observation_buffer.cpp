
#include "hb_robot_perception/observation_buffer.hpp"
#include <utility>

namespace hb_perception{

    void ObservationBuffer::addObservation(Observation ob)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        observations_.push_back(ob);

    }

    std::size_t ObservationBuffer::size() const{ 
        std::lock_guard<std::mutex> lock(mutex_);
        return observations_.size();
    }

    void ObservationBuffer::clear() { 
    std::lock_guard<std::mutex> lock(mutex_);
    observations_.clear();

    }
    std::optional<Observation> ObservationBuffer::get(std::size_t index) const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(index>=observations_.size()){
            return std::nullopt;
        }

        return observations_[index];
    }
    std::optional<Observation> ObservationBuffer::get_latest() const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(observations_.empty()){
            return std::nullopt;
        }
        return observations_.back();
    }
    const std::vector<Observation>& ObservationBuffer::observations() const{
         std::lock_guard<std::mutex> lock(mutex_);
         return observations_;
    }


    bool ObservationBuffer::empty() const{
        std::lock_guard<std::mutex>lock(mutex_);
        return observations_.empty();
    }
   

}
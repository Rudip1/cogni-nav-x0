#ifndef GAZEBO_2D_GROUNDTRUTH_MAP_COMMON_HPP
#define GAZEBO_2D_GROUNDTRUTH_MAP_COMMON_HPP

#include <sdf/sdf.hh>
#include <iostream>
#include <string>

namespace gazebo
{

/// Safe SDF parameter reader
template <typename T>
inline bool getSdfParam(sdf::ElementPtr sdf, const std::string &name, T &param,
                        const T &default_value, bool verbose = false)
{
    if (sdf->HasElement(name))
    {
        param = sdf->Get<T>(name);
        return true;
    }
    else
    {
        param = default_value;
        if (verbose)
            std::cerr << "[gazebo_2d_groundtruth_map] Warning: parameter \"" 
                      << name << "\" not found. Using default: " 
                      << default_value << std::endl;
        return false;
    }
}

} // namespace gazebo

#endif // GAZEBO_2D_GROUNDTRUTH_MAP_COMMON_HPP
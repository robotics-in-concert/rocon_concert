
#ifndef ROCON_TF_RECONSTRUCTOR_UTILS_H_
#define ROCON_TF_RECONSTRUCTOR_UTILS_H_

#include <algorithm>
#include <string>

namespace rocon {

std::string space2underscore(const std::string str);
std::string get_ros_friendly_name(const std::string name);

}

#endif

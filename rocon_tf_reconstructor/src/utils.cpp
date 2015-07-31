
#include "rocon_tf_reconstructor/utils.h"

namespace rocon {

std::string space2underscore(std::string str)
{
  for(int i = 0; i < str.length(); i++)
  {
      if(str[i] == ' ')
          str[i] = '_';
  }
  return str;
}

std::string get_ros_friendly_name(const std::string name)
{
  std::string new_name = name;
  std::transform(new_name.begin(), new_name.end(), new_name.begin(), ::tolower);

  return space2underscore(new_name);
}
}

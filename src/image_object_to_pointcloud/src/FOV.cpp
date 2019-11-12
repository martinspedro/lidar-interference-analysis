
#include "image_object_to_pointcloud/FOV.hpp"

std::ostream& operator<<(std::ostream& out, const FOV& object)
{
  out << "(" << object.x << ", " << object.y << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, FOV& object)
{
  out << "(" << object.x << ", " << object.y << ")";
  return out;
}

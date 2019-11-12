

#ifndef FOV_H
#define FOV_H

#include <ostream>

struct FOV
{
  union
  {
    float data[2];
    struct
    {
      float x;
      float y;
    };
  };

  friend std::ostream& operator<<(std::ostream& out, const FOV& object);
  friend std::ostream& operator<<(std::ostream& out, FOV& object);
};

#endif  // FOV_H

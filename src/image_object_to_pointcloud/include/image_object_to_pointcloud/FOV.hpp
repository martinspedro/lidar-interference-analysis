/*!
 * \file   FOV.hpp
 * \brief  Data structure that implements a field of view data type - header file
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef FOV_H
#define FOV_H

#include <ostream>

/*!
 * \struct FOV
 * \brief Structure that implements a Fiel of View data structure
 */
struct FOV
{
  /*!
   * \brief Union for acessing the FOV data in array or member like
   */
  union
  {
    float data[2];  //!< Array with the x and y coordinates
    /*!
     * \brief Structure that the two components of the FOV
     */
    struct
    {
      float x;  //!< Field of View along the X axis
      float y;  //!< Field of View along the Y axis
    };
  };

  /*!
   * \brief Overload stream operator for constant FOV objects
   */
  friend std::ostream& operator<<(std::ostream& out, const FOV& object);

  /*!
   * \brief Overload stream operator for FOV objects
   */
  friend std::ostream& operator<<(std::ostream& out, FOV& object);
};

#endif  // FOV_H

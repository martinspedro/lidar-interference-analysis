/*!
 *  \file   exceptions.hpp
 *  \brief  Class to implemented a NotImplemented exception
 *
 *  \author Pedro Martins (martinspedro@ua.pt)
 *  \remark Based on
 * https://stackoverflow.com/questions/24469927/does-c-have-an-equivalent-to-nets-notimplementedexception
 */

#include <stdexcept>

/*!
 * Implementation of a class for a NotImplemented exception
 * Inherits from std::logic_error
 */
class NotImplemented : public std::logic_error
{
public:
  NotImplemented() : std::logic_error("Function not yet implemented!"){};
};

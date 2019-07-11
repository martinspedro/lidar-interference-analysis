#include <stdexcept>

// https://stackoverflow.com/questions/24469927/does-c-have-an-equivalent-to-nets-notimplementedexception
class NotImplemented : public std::logic_error
{
public:
    NotImplemented() : std::logic_error("Function not yet implemented") { };
};

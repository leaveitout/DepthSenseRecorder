#ifndef ERROR_HPP
#define ERROR_HPP

#include <system_error>

namespace rgbd {

    class UnsupportedException: public std::domain_error {
    public:
        UnsupportedException(const std::string& cause) :
                std::domain_error(cause + ": unsupported") {}
    };

}

#endif // ERROR_HPP

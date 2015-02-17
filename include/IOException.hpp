#ifndef IO_EXCEPTION_HPP
#define IO_EXCEPTION_HPP

#include <exception>

namespace rgbd {

    class IOException : public std::exception {
    public:
        IOException(const std::string& cause) :
                                   std::exception(cause + ": IOException") {}
    };

}

#endif
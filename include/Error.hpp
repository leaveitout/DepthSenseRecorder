#ifndef ERROR_HPP
#define ERROR_HPP

#include <exception>

namespace rgbd {

    class UnsupportedException: public std::exception {
    public:
        virtual const char* what() const throw() {
            return "Unsupported, calling virutal method";
        }
    };

}

#endif // ERROR_HPP

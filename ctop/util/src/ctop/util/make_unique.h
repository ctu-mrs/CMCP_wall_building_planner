//
// Created by michalks on 11/26/17.
//

#ifndef CTOP_PLANNERMAKE_UNIQUE_H
#define CTOP_PLANNERMAKE_UNIQUE_H

#include <utility>
#include <memory>

namespace ctop {

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

}

#endif //CTOP_PLANNERMAKE_UNIQUE_H

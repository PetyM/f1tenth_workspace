#ifndef FUZZYCOMPARE_H
#define FUZZYCOMPARE_H

#include <cmath>
#include <type_traits>


template<typename T>
    requires std::is_floating_point_v<T> // Restrict to floating-point numbers
bool equal(T a, T b)
{
    return std::abs(a - b) < std::numeric_limits<T>::epsilon();
}

template<typename T>
    requires std::is_arithmetic_v<T>
bool nearZero(T a)
{
    return abs(a) < std::numeric_limits<T>::epsilon();
}

#endif // FUZZYCOMPARE_H
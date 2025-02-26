#ifndef __INCLUDED__NODEGRAPH__HELPERS__HPP__
#define __INCLUDED__NODEGRAPH__HELPERS__HPP__

#define DBG(expr)
// #define DBG(expr) std::cout << "DBG> " << expr << std::endl

/// @brief An interface capable of loading a value into somewhere
/// @tparam T 
template<class T> struct IValueLoader
{
    virtual void LoadValue(T &&value) = 0;

protected:
    // Only derived class can call this destructor
    ~IValueLoader() {}
};

/// @brief An interface capable of obtaining a value from somewhere
/// @tparam T 
template<class T> struct IValueHolder
{
    virtual T const &GetValue() const = 0;

protected:
    // Only derived class can call this destructor
    ~IValueHolder() {}
};

#endif//__INCLUDED__NODEGRAPH__HELPERS__HPP__

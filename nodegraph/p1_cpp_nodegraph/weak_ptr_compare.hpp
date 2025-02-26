#ifndef __INCLUDED_NODEGRAPH__WEAK_PTR_COMPARE__HPP__
#define __INCLUDED_NODEGRAPH__WEAK_PTR_COMPARE__HPP__

#include<memory>

/// @brief Compare two weak pointers by their pointer value
/// @remark Inspired by: https://stackoverflow.com/questions/32668742/a-set-of-weak-ptr
template<class T> struct weak_ptr_compare {
    bool operator() (const std::weak_ptr<T> &lhs, const std::weak_ptr<T> &rhs)const {
        auto lptr = lhs.lock(), rptr = rhs.lock();
        if (!rptr) return false; // nothing after expired pointer 
        if (!lptr) return true;  // every not expired after expired pointer
        return lptr.get() < rptr.get();
    }
};

#endif//__INCLUDED_NODEGRAPH__WEAK_PTR_COMPARE__HPP__
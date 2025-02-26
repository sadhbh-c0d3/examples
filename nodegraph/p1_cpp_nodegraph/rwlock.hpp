#ifndef __INCLUDED_NODEGRAPH__RW_LOCK__HPP__
#define __INCLUDED_NODEGRAPH__RW_LOCK__HPP__

#include<optional>
#include<shared_mutex>

/// @brief Lock guard for unlocked data
/// @tparam L Type of lock used
/// @tparam T Type of data unlocked
template<class L, class T> class RwLockGuard
{
public:
    RwLockGuard(L &&m, T *d): m_lock{std::move(m)}, m_data{d} {}

    RwLockGuard(RwLockGuard &&other): m_lock{std::move(other.m_lock)}, m_data{std::move(other.m_data)} {}
    RwLockGuard &operator=(RwLockGuard &&other)
    {
        m_lock = std::move(other.m_lock);
        m_data = std::move(other.m_data);
    }

    T *get() { return m_data; }
    T *operator->() { return m_data; }
    T& operator*() { return *m_data; }

private:
    L m_lock;
    T *m_data;
    
    RwLockGuard(RwLockGuard const &) = delete;
    RwLockGuard &operator=(RwLockGuard const &) = delete;
};

/// @brief Lock guard of data unlocked for reading
/// @tparam T Type of unlocked data
template<class T> using RwLockRead = RwLockGuard<std::shared_lock<std::shared_mutex>, T>;

/// @brief Lock guard of data unlocked for writing
/// @tparam T Type of unlocked data
template<class T> using RwLockWrite = RwLockGuard<std::unique_lock<std::shared_mutex>, T>;

/// @brief Interface for supporting locking of polymorphic data
/// @tparam I Type of interface
template<class I> class IRwLock
{
public:
    /// @brief Test for concrete data type and unlock for writing
    /// @tparam T Type of data to be unlocked
    /// @return Optional unlocked data
    template<class T> std::optional<RwLockWrite<T>> try_write() const
    {
        if (T *ptr = dynamic_cast<T *>(get_ptr()); nullptr != ptr)
        {
            return RwLockWrite<T>{std::unique_lock(m_mutex), ptr};
        }
        else { 
            return {};
        }
    }
   
    /// @brief Test for concrete data type and unlock for reading
    /// @tparam T Type of data to be unlocked
    /// @return Optional unlocked data
    template<class T> std::optional<RwLockRead<T const>> try_read() const
    {
        if (T const *ptr = dynamic_cast<T const *>(get_ptr()); nullptr != ptr)
        {
            return RwLockRead<T const>(std::shared_lock(m_mutex), ptr);
        }
        else { 
            return {};
        }
    }
    
    /// @brief Unlock abstract data for reading
    RwLockRead<I const> read() const
    {
        return {std::shared_lock(m_mutex), get_ptr()};
    }
    
    /// @brief Unlock abstract data for writing
    RwLockWrite<I> write() const
    {
        return {std::unique_lock(m_mutex), get_ptr()};
    }

protected:
    mutable std::shared_mutex m_mutex{};
    I &m_data_ref;
    
    I *get_ptr() const { return &m_data_ref; }
    
    IRwLock(I &data_ref) : m_data_ref{data_ref} {}
    ~IRwLock() {}

    IRwLock(IRwLock const &) = delete;
    IRwLock(IRwLock &&) = delete;

    IRwLock &operator=(IRwLock const &) = delete;
    IRwLock &operator=(IRwLock &&) = delete;
};

/// @brief Locked data supporting unlocking for reading and writing
/// @tparam T Type of data locked
template<class T> class RwLock : public IRwLock<typename T::base_interface>
{
public:
    /// @brief Type of data locked must provide abstract interface type so that
    // we can use IRwLock<I> and then be able to downcast to RwLock<T> when needed.
    using I = typename T::base_interface;

    template<class ...Args> RwLock(Args &&...args): IRwLock<I>{m_data}, m_data{std::forward<Args>(args)...} {}

    RwLock(RwLock const &) = delete;
    RwLock &operator=(RwLock const &) = delete;

    /// @brief Unlock data for reading
    /// @remark This is shared lock, and can be acquired multiple times simultaneously.
    RwLockRead<T const> read() const
    {
        return RwLockRead<T const>(std::shared_lock{IRwLock<I>::m_mutex}, get_ptr());
    }

    /// @brief Unlock data for writing
    /// @remark This is non-reentrant lock, and calling write() again while
    /// already write-locked will cause dead-lock.
    RwLockWrite<T> write() const
    {
        return RwLockWrite<T>(std::unique_lock{IRwLock<I>::m_mutex}, get_ptr());
    }

private:
    mutable T m_data;

protected:
    T *get_ptr() const { return &m_data; }
};

#endif//__INCLUDED_NODEGRAPH__RW_LOCK__HPP__
#ifndef __INCLUDED__NODEGRAPH__HELPERS__HPP__
#define __INCLUDED__NODEGRAPH__HELPERS__HPP__

#ifndef DBG_ENABLE
    #define DBG_ENABLE 0
#endif

#if DBG_ENABLE
    #include <iostream>
    //#include <syncstream> // was not available on my compiler
    #include <sstream>
    #include <thread>
    #include <mutex>

    extern std::ostream debug_out;
    extern std::mutex debug_out_mutex;
    bool init_debugout(char const *path = "debug.log");
    
    struct sync_debug_out : std::stringstream
    {
        ~sync_debug_out() { debug_out << str(); }
        sync_debug_out(): m_lock{debug_out_mutex} {}
        std::unique_lock<std::mutex> m_lock;
    };

    std::string name_of_thread(std::thread::id);
    std::string name_of_entity(void *);

    #define DBG_IF(cond, expr) ((cond) ? (expr) : (debug_out))
    #define DBG_ID(id) "[" << name_of_entity((void*)id) << "] "
    #define DBG_PARAM(expr) , expr
    #define DBG_INDENT(expr) ((expr) + "    ")
    //#define DBG(expr) (std::osyncstream(debug_out) << "DBG> " << DBG_ID(std::this_thread::get_id()) << expr << std::endl)
    #define DBG(expr) (sync_debug_out() << "[" << name_of_thread(std::this_thread::get_id()) << "] " << expr << std::endl)
    #define DBG_OR(normal_expr, debug_expr) debug_expr
#else
    #define init_debugout(...) (true)
    #define DBG(expr)
    #define DBG_IF(cond, expr)
    #define DBG_ID(id)
    #define DBG_PARAM(expr)
    #define DBG_OR(normal_expr, debug_expr) normal_expr
#endif

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

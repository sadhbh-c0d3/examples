#ifndef __INCLUDED__NODEGRAPH__ACTION__HPP__
#define __INCLUDED__NODEGRAPH__ACTION__HPP__

#include <functional>
#include <execution>
#include <ranges>
#include <optional>
#include <vector>
#include <ostream>
#include <coroutine>

#include "helpers.hpp"

#ifndef DBG_ENABLE_ACTION
#define DBG_ENABLE_ACTION 0
#endif

#if DBG_ENABLE_ACTION
    #define ACTION_DBG(expr) DBG(expr)
#else
    #define ACTION_DBG(expr)
#endif

struct Single;
struct Sequential;
struct Parallel;
struct Coroutine;

using Actionable = std::variant<Single, Sequential, Parallel, Coroutine>;
using Action = std::optional<Actionable>;

template<class T>
concept ExecutionPolicyConcept = requires(T policy, Sequential seq, Parallel par, Coroutine coro) {
    { policy(std::move(seq)) } -> std::same_as<void>;
    { policy(std::move(par)) } -> std::same_as<void>;
    { policy(std::move(coro)) } -> std::same_as<void>;
};

struct Single: std::function<Action ()> {
    Single(std::function<Action ()> &&func): std::function<Action ()>{std::move(func)} {}
};

struct Sequential : std::vector<Action> {
    Sequential(std::vector<Action> &&actions);
};

struct Parallel : std::vector<Action> {
    Parallel(std::vector<Action> &&actions);
};

struct Promise;

struct Coroutine : std::coroutine_handle<Promise> {
    using promise_type = Promise;

    bool next(ExecutionPolicyConcept auto &&policy
                DBG_PARAM(std::string dbg_indent));
};

class Promise {
public:
    Coroutine get_return_object() { return {Coroutine::from_promise(*this)}; }

    std::suspend_always initial_suspend() noexcept { return {}; }
    std::suspend_always final_suspend() noexcept { return {}; }

    std::suspend_always await_transform(Action other) { 
        m_next = std::move(other);
        return {};
    }

    void return_void() {}
    void unhandled_exception() { m_exception = std::current_exception(); }

    bool next(ExecutionPolicyConcept auto &&policy
                DBG_PARAM(std::string dbg_indent));

private:
    Action m_next;
    std::exception_ptr m_exception;
};

inline bool Coroutine::next(ExecutionPolicyConcept auto &&policy
    DBG_PARAM(std::string dbg_indent)) {

    if (promise().next(std::forward<decltype(policy)>(policy)
        DBG_PARAM(dbg_indent))) { return true; }

    resume();
    return not done();
}


inline Sequential::Sequential(std::vector<Action> &&actions): std::vector<Action>{std::move(actions)} {}
inline Parallel::Parallel(std::vector<Action> &&actions): std::vector<Action>{std::move(actions)} {}

inline Action deferred_end() {
    return {};
}

inline Action deferred_action(std::function<Action()> action) {
    return Single(std::move(action));
}

inline Action deferred_coroutine(Coroutine coro) {
    return coro;
}

template<class... Actions> Sequential defer_sequential(Actions &&...actions) {
    return std::vector<Action>{ std::forward<Actions>(actions)... };
}

template<class... Actions> Parallel defer_parallel(Actions &&...actions) {
    return std::vector<Action>{ std::forward<Actions>(actions)... };
}

inline Sequential defer_sequential(std::vector<Action> actions) {
    return std::move(actions);
}

inline Parallel defer_parallel(std::vector<Action> actions) {
    return std::move(actions);
}

template<class InputRage, class TransformFunc>
std::vector<Action> trasform_into_deferred_actions(InputRage &&input_range, TransformFunc &&transform_func) {
    std::vector<Action> actions;
    std::ranges::transform(
        std::forward<InputRage>(input_range),
        std::back_inserter(actions),
        std::forward<TransformFunc>(transform_func));
    return std::move(actions);
}

void run_actions(ExecutionPolicyConcept auto &&policy, Action action
    DBG_PARAM(std::string dbg_indent = "")) {

    ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "{{ ");
    
    while (action.has_value()) {
        ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "  {");

        auto actionable = std::exchange(action, {}).value();

        if (auto *single = std::get_if<Single>(&actionable); nullptr != single) {
            action = (*single)();
        }
        else if (auto *sequential = std::get_if<Sequential>(&actionable); nullptr != sequential) {
            policy(std::move(*sequential)  DBG_PARAM(dbg_indent));
        }
        else if (auto *parallel = std::get_if<Parallel>(&actionable); nullptr != parallel) {
            policy(std::move(*parallel)  DBG_PARAM(dbg_indent));
        }
        else if (auto *coro = std::get_if<Coroutine>(&actionable); nullptr != coro) {
            policy(std::move(*coro)  DBG_PARAM(dbg_indent));
        }
        else { break; }

        ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "  }");
    }

    ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "}}");
}

bool Promise::next(ExecutionPolicyConcept auto &&policy
    DBG_PARAM(std::string dbg_indent)) {

    if (m_exception) { std::rethrow_exception(m_exception); }

    if (not m_next.has_value()) { return false; }
    
    if (auto *coro = std::get_if<Coroutine>(&m_next.value()); nullptr != coro) {

        if (coro->done() || not coro->next(std::forward<decltype(policy)>(policy)
            DBG_PARAM(DBG_INDENT(dbg_indent)))) {

            m_next = {};
            return false;
        }
        
        return true;
    }
    else {
        run_actions(std::forward<decltype(policy)>(policy),
            std::exchange(m_next, {})
            DBG_PARAM(dbg_indent));

        return true;
    }
}


template<const bool EnableParallel = true> struct StandardPolicy
{
    void operator()(Sequential &&sequential  DBG_PARAM(std::string dbg_indent = ""))
    {
        std::for_each(std::execution::seq, sequential.begin(), sequential.end(), [this
            DBG_PARAM(dbg_indent)](auto &&a) { run_actions(*this, 
                std::forward<decltype(a)>(a)  DBG_PARAM(DBG_INDENT(dbg_indent))); });
    }
    
    void operator()(Parallel &&parallel  DBG_PARAM(std::string dbg_indent = ""))
    {
        if constexpr (EnableParallel) {
            std::for_each(std::execution::par, parallel.begin(), parallel.end(), [this
                DBG_PARAM(dbg_indent)](auto &&a) { run_actions(*this,
                    std::forward<decltype(a)>(a)  DBG_PARAM(DBG_INDENT(dbg_indent))); });
        } else {
            std::for_each(std::execution::seq, parallel.begin(), parallel.end(), [this
                DBG_PARAM(dbg_indent)](auto &&a) { run_actions(*this,
                    std::forward<decltype(a)>(a)  DBG_PARAM(DBG_INDENT(dbg_indent))); });
        }
    }

    void operator()(Coroutine &&coro   DBG_PARAM(std::string dbg_indent = ""))
    {
        while (coro.next(*this   DBG_PARAM(DBG_INDENT(dbg_indent)))) ;
    }
};

template<bool EnableParallel> std::ostream &operator<<(std::ostream &os, StandardPolicy<EnableParallel> const &policy)
{
    if constexpr (EnableParallel) {
        return (os << "Parallel Execution");
    } else {
        return (os << "Sequential Execution");
    }
}

#endif//__INCLUDED__NODEGRAPH__ACTION__HPP__
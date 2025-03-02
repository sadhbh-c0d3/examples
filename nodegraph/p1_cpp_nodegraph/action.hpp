#ifndef __INCLUDED__NODEGRAPH__ACTION__HPP__
#define __INCLUDED__NODEGRAPH__ACTION__HPP__

#include <functional>
#include <execution>
#include <ranges>
#include <optional>
#include <vector>

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

using Actionable = std::variant<Single, Sequential, Parallel>;
using Action = std::optional<Actionable>;

struct Single: std::function<Action ()> {
    Single(std::function<Action ()> &&func): std::function<Action ()>{std::move(func)} {}
};

struct Sequential : std::vector<Action> {
    Sequential(std::vector<Action> &&actions);
};

struct Parallel : std::vector<Action> {
    Parallel(std::vector<Action> &&actions);
};

inline Sequential::Sequential(std::vector<Action> &&actions): std::vector<Action>{std::move(actions)} {}
inline Parallel::Parallel(std::vector<Action> &&actions): std::vector<Action>{std::move(actions)} {}

template<class ExecutionPolicy> void run_actions(ExecutionPolicy &&policy, Action action
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
        else { break; }

        ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "  }");
    }

    ACTION_DBG(DBG_ID(&action) << "run_actions: " << dbg_indent << "}}");
}

template<const bool EnableParallel = true> struct StandardPolicy
{
    void operator()(Sequential sequential  DBG_PARAM(std::string dbg_indent = ""))
    {
        std::for_each(std::execution::seq, sequential.begin(), sequential.end(), [this
            DBG_PARAM(dbg_indent)](auto &a) { run_actions(*this, a  DBG_PARAM(DBG_INDENT(dbg_indent))); });
    }
    
    void operator()(Parallel parallel  DBG_PARAM(std::string dbg_indent = ""))
    {
        if constexpr (EnableParallel) {
            std::for_each(std::execution::par, parallel.begin(), parallel.end(), [this
                DBG_PARAM(dbg_indent)](auto &a) { run_actions(*this, a  DBG_PARAM(DBG_INDENT(dbg_indent))); });
        } else {
            std::for_each(std::execution::seq, parallel.begin(), parallel.end(), [this
                DBG_PARAM(dbg_indent)](auto &a) { run_actions(*this, a  DBG_PARAM(DBG_INDENT(dbg_indent))); });
        }
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

inline Action deferred_end() {
    return {};
}

inline Action deferred_action(std::function<Action()> action) {
    return Single(std::move(action));
}

inline Sequential defer_sequential(Action first, Action then) {
    return std::vector{ std::move(first), std::move(then) };
}

inline Parallel defer_parallel(Action first, Action then) {
    return std::vector{ std::move(first), std::move(then) };
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


#endif//__INCLUDED__NODEGRAPH__ACTION__HPP__
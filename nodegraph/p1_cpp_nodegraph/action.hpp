#ifndef __INCLUDED__NODEGRAPH__ACTION__HPP__
#define __INCLUDED__NODEGRAPH__ACTION__HPP__

#include <functional>
#include <ranges>
#include <optional>
#include <vector>

#include "helpers.hpp"

//#define ACTION_DBG(expr)
#define ACTION_DBG(expr) DBG(expr)

struct Actionable;

using Action = std::optional<Actionable>;

struct Actionable : std::function<Action ()> {
    Actionable(std::function<Action ()> &&func): std::function<Action ()>{std::move(func)} {}
};

inline void run_synchronously(Action action) {
    ACTION_DBG("run_synchronous: {{");
    while (action.has_value()) {
        ACTION_DBG("run_synchronous: {");
        action = action.value()();
        ACTION_DBG("run_synchronous: }");
    }
    ACTION_DBG("run_synchronous: }}");
}

inline Action deferred_end() {
    return {};
}

inline Action deferred_action(std::function<Action()> action) {
    return std::move(action);
}

inline Action defer_synchronous(Action first, Action then) {
    return deferred_action([first = std::move(first), then = std::move(then)] () {
        run_synchronously(std::move(first));
        run_synchronously(std::move(then));
        return deferred_end();
    });
}

inline Action defer_synchronous(std::vector<Action> actions) {
    return deferred_action([actions = std::move(actions)] () mutable {
        for (Action &action : actions) {
            run_synchronously(std::move(action));
        }
        return deferred_end();
    });
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
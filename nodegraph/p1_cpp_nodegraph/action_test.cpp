#include "action.hpp"

Action foo () {
    return Actionable{[]() {
        return Action{};
    }};
}

int action_test() {
    run_synchronously(foo());

    defer_synchronous(foo(), deferred_action([]() {
        return deferred_end();
    }));

    std::vector<int> ints{1,2,3,4};

    trasform_into_deferred_actions(ints, [](auto &a) {
        return deferred_end();
    });

    return 0;
}
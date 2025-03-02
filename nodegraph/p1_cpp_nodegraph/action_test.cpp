#include "action.hpp"

Action foo () {
    return deferred_action([]{
        return deferred_end();
    });
}

int action_test() {
    StandardPolicy<false> policy;

    run_actions(policy, foo());

    defer_sequential(foo(), deferred_action([]() {
        return deferred_end();
    }));

    std::vector<int> ints{1,2,3,4};

    trasform_into_deferred_actions(ints, [](auto &a) {
        return deferred_end();
    });

    return 0;
}
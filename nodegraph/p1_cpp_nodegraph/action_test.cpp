#include "action.hpp"
#include "rwlock.hpp"

namespace {

Coroutine yet_another_coroutine(std::string text) {
    DBG(DBG_ID(&text) << "Yet Another Coroutine with text: " << text);
    co_return;
}

Action foo (std::string text) {
    return defer_sequential(
        deferred_action([text = std::move(text)]{
            DBG(DBG_ID(&text) << "Foo stage 1 with text: " << text);
            return deferred_coroutine([text] () -> Coroutine {
                DBG(DBG_ID(&text) << "Foo inner-stage with text: " << text);
                co_await yet_another_coroutine(text);
            }());
        }),
        deferred_action([text = std::move(text)]{
            DBG(DBG_ID(&text) << "Foo stage 2 with text: " << text);
            return deferred_end();
        })
    );
}

Coroutine other_coroutine(std::string text) {
    DBG(DBG_ID(&text) << "Other Coroutine with text: " << text);
    co_await foo(text);
}

Coroutine happy_coroutine(std::string text) {
    DBG(DBG_ID(&text) << "Happy Coroutine with text: " << text);
    co_await other_coroutine(text);
}

struct Fuzz {
    using base_interface = Fuzz;

    Coroutine process() {
        co_await happy_coroutine("Fuzz process");
    }
};

class Buzz {
public:
    Coroutine process_a() const {
        return fuzz.write()->process();
    }
    
    Coroutine process_b() const {
        co_await fuzz.write()->process();
    }

    Coroutine process_c() const {
        auto coro = fuzz.write()->process();
        co_await coro;
    }
private:
    RwLock<Fuzz> fuzz;
};

} // namespace

int action_test() {
    DBG("TEST: Action Test");

    StandardPolicy<false> policy;

    run_actions(policy, defer_sequential(
        foo("Hello"),
        deferred_action([]() {
            return deferred_end();
        }),
        deferred_coroutine([]() -> Coroutine {
            auto hello = happy_coroutine("Hello World!!!");
            co_await hello;
            auto bye = happy_coroutine("Good Bye!");
            co_await bye;
        }())
    ));

    auto buzz = std::make_unique<Buzz>();
    run_actions(policy,
        deferred_coroutine([buzz = std::move(buzz)]() -> Coroutine {
            co_await buzz->process_a();
            co_await buzz->process_b();
            co_await buzz->process_c();
        }()));

    std::vector<int> ints{1,2,3,4};

    trasform_into_deferred_actions(ints, [](auto &a) {
        return deferred_end();
    });

    return 0;
}
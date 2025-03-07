
#include <coroutine>
#include <optional>

#include "action.hpp"

namespace {

Coroutine fizz(int i) {
    DBG("fizz: " << i);
    co_return;
}

Coroutine fuzz(int i) {
    DBG("fuzz: " << i);
    co_return;
}


Coroutine bar(int i) {
    DBG("bar: " << i);
    auto coro1 = fizz(i);
    co_await coro1;
    auto coro2 = fuzz(i);
    co_await coro2;
}

Coroutine baz() {
    auto coro = bar(1);
    co_await coro;
    auto coro1 = bar(2);
    co_await coro1;
}

void foo() {
    StandardPolicy<false> policy;
    auto coro = baz();
    while (coro.next(policy  DBG_PARAM(""))) {
        DBG("(poke)");
    }
    DBG("(end)");
}

} // namespace

void coro_test() {
    DBG("TEST: Coroutine Test");

    foo();
    DBG("(done)");
}

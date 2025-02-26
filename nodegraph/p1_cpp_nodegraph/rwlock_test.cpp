#include "rwlock.hpp"
#include <iostream>

struct Base {
    virtual ~Base() { std::cout << "~Base()" << std::endl; }
    void foo() { std::cout << "foo()" << std::endl; }
    void bar() const { std::cout << "bar()" << std::endl; }
};

struct Derived : Base {
    using base_interface = Base;
    void baz() { std::cout << "baz()" << std::endl; }
    void zab() const { std::cout << "zab()" << std::endl; }
};

void rwlock_test()
{
    RwLock<Derived> derived;

    derived.write()->baz();
    derived.read()->bar();

    derived.read_base()->bar();
    derived.write_base()->foo();

    IRwLock<Base> &base{derived};

    {
        auto maybe_derived = base.try_write<Derived>();
        maybe_derived.value()->baz();
    }

    {
        auto maybe = base.try_read<Derived>();
        maybe.value()->zab();
    }
}
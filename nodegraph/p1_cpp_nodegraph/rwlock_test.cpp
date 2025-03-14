#include "rwlock.hpp"
#include <iostream>

struct Base {
    virtual ~Base() { std::cout << "~Base()" << std::endl; }
    void foo() { std::cout << "foo()" << std::endl; }
    void bar() const { std::cout << "bar()" << std::endl; }
};

struct Derived : Base, EnableRwLockFromThis<Derived>{
    using base_interface = Base;
    void baz() { std::cout << "baz()" << std::endl; }
    void zab() const { std::cout << "zab()" << std::endl; }
};

struct Derived2 : Base {
    using base_interface = Base;
    void baz() { std::cout << "baz()" << std::endl; }
    void zab() const { std::cout << "zab()" << std::endl; }
};

void rwlock_test()
{
    RwLock<Derived> derived;

    derived.write()->baz();
    derived.read()->bar();

    IRwLock<Base> &base{derived};
    
    base.read()->bar();
    base.write()->foo();

    {
        auto maybe_derived = base.try_write<Derived>();
        maybe_derived.value()->baz();
    }

    {
        auto maybe = base.try_read<Derived>();
        maybe.value()->zab();
    }

    {
        Derived *d = derived.write().get();
        auto rw_lock = d->lock_from_this();
        rw_lock->read()->bar();
    }

    RwLock<Derived2> derived2;

    derived2.read()->bar();
    derived2.write()->baz();
}
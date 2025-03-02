#include "helpers.hpp"

// We don't have a header, because we're not going to reuse these declarations
void test_s1_static_graph();
void test_s1_static_graph_with_shared_ptr_data();
void test_s2_dynamic_graph();
void test_s3_dynamic_graph_and_pins();
void test_s4_dynamic_graph_and_shared_ptr();
void test_s5_dynamic_graph_and_try_connect();
void test_s6_dynamic_graph_and_fork();
void test_s7_dynamic_graph_and_fork_fix();
void test_s8_dynamic_graph_and_lock();
void test_s9_dynamic_graph_and_deferred();

int main(int argc, char **argv)
{
    if (!init_debugout()) {
        return -1;
    }

    DBG("main");

    // S1: Static Node Graph
    test_s1_static_graph();
    test_s1_static_graph_with_shared_ptr_data();

    // S2: Dynamic Nodes Graph
    test_s2_dynamic_graph();

    // S3: Dynamic Nodes Graph with dynamic pins
    test_s3_dynamic_graph_and_pins();

    // S4: Dynamic graph with dynamic pins via shared_ptr
    test_s4_dynamic_graph_and_shared_ptr();

    // S5: Dyamic graph with dynamic connections
    test_s5_dynamic_graph_and_try_connect();

    // S6: Dynamic graph with forked output
    test_s6_dynamic_graph_and_fork();

    // S7: Forward propagation fix for forked output
    test_s7_dynamic_graph_and_fork_fix();

    // S8: Locking
    test_s8_dynamic_graph_and_lock();

    // S9: Async
    test_s9_dynamic_graph_and_deferred();

    return 0;
}

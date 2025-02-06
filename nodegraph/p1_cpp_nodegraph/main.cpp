// We don't have a header, because we're not going to reuse these declarations
void test_int_to_double();
void test_with_shared_ptr();
void test_with_node_graph();


int main(int argc, char **argv)
{
    // S1: Static Node Graph
    test_int_to_double();
    test_with_node_graph();

    // S2: Dynamic Nodes Graph
    test_with_node_graph();

    return 0;
}

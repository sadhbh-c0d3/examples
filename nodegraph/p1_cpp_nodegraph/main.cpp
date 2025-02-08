// We don't have a header, because we're not going to reuse these declarations
void test_s1_static_graph();
void test_s1_static_graph_with_shared_ptr_data();
void test_s2_dynamic_graph();
void test_s3_dynamic_graph_and_pins();


int main(int argc, char **argv)
{
    // S1: Static Node Graph
    test_s1_static_graph();
    test_s1_static_graph_with_shared_ptr_data();

    // S2: Dynamic Nodes Graph
    test_s2_dynamic_graph();

    // S3: Dynamic Nodes Graph with dynamic pins
    test_s3_dynamic_graph_and_pins();

    return 0;
}

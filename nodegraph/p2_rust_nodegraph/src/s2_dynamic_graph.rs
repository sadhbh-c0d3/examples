use std::cell::RefCell;


trait GraphNode<'a>
{
    fn process_forwards(&self);
    fn process_backwards(&self);
    fn setup(&'a self);
}

#[derive(Default)]
struct GraphNodePin<'a, T> where T: Clone + Default
{
    node: RefCell<Option<&'a dyn GraphNode<'a>>>,
    connection: RefCell<Option<&'a GraphNodePin<'a, T>>>,
    data: RefCell<T>
}

impl<'a, T> GraphNodePin<'a, T> where T: Clone + Default
{
    pub fn get_node(&self) -> Option<&'a dyn GraphNode> {
        self.node.borrow().clone()
    }

    pub fn get_connection(&self) -> Option<&'a GraphNodePin<'a, T>> {
        self.connection.borrow().clone()
    }

    pub fn setup(&self, node: &'a dyn GraphNode<'a>) {
        self.node.replace(node.into());
    }

    pub fn connect(&self, pin: &'a GraphNodePin<'a, T>) {
        self.connection.replace(pin.into());
    }

    pub fn set_data(&self, data: T) {
        self.data.replace(data);
    }

    pub fn get_data(&self) -> T {
        self.data.borrow().clone()
    }
}

#[derive(Default)]
struct GraphNodeInputPin<'a, T> where T: Clone + Default
{
    pin: GraphNodePin<'a, T>
}

impl<'a, T> GraphNodeInputPin<'a, T> where T: Clone + Default
{
    pub fn connect(&'a self, output_pin: &'a GraphNodeOutputPin<'a, T>) {
        self.pin.connect(&output_pin.pin);
        output_pin.pin.connect(&self.pin);
    }

    pub fn receive_data(&self) {
        if let Some(input) = self.pin.get_connection() {
            self.pin.set_data(input.get_data());
        }
    }

    pub fn propagate_backwards(&self) {
        if let Some(input) = self.pin.get_connection() {
            if let Some(node) = input.get_node() {
                node.process_backwards();
            }
        }
    }
}

#[derive(Default)]
struct GraphNodeOutputPin<'a, T> where T: Clone + Default
{
    pin: GraphNodePin<'a, T>
}

impl<'a, T> GraphNodeOutputPin<'a, T> where T: Clone + Default
{
    pub fn connect(&'a self, input_pin: &'a GraphNodeInputPin<'a, T>) {
        self.pin.connect(&input_pin.pin);
        input_pin.pin.connect(&self.pin);
    }

    pub fn send_data(&self) {
        if let Some(output) = self.pin.get_connection() {
            output.set_data(self.pin.get_data());
        }
    }
    
    pub fn propagate_forwards(&self) {
        if let Some(output) = self.pin.get_connection() {
            if let Some(node) = output.get_node() {
                node.process_forwards();
            }
        }
    }
}

#[derive(Default)]
struct SourceGraphNode<'a, T> where T: Clone + Default
{
    pub output_pin: GraphNodeOutputPin<'a, T>
}

impl<'a, T> SourceGraphNode<'a, T> where T: Clone + Default
{
    pub fn load_value(&self, data: T)
    {
        self.output_pin.pin.set_data(data);
    }
}

impl<'a, T> GraphNode<'a> for SourceGraphNode<'a, T> where T: Clone + Default
{
    fn process_backwards(&self) {
    }

    fn process_forwards(&self) {
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }

    fn setup(&'a self) {
        self.output_pin.pin.setup(self);
    }
}

#[derive(Default)]
struct TargetGraphNode<'a, T> where T: Clone + Default
{
    pub input_pin: GraphNodeInputPin<'a, T>
}

impl<'a, T> TargetGraphNode<'a, T> where T: Clone + Default
{
    pub fn get_value(&self) -> T
    {
        self.input_pin.pin.get_data()
    }
}

impl<'a, T> GraphNode<'a> for TargetGraphNode<'a, T> where T: Clone + Default
{
    fn process_backwards(&self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
    }

    fn process_forwards(&self) {
    }
    
    fn setup(&'a self) {
        self.input_pin.pin.setup(self);
    }
}

struct TransformGraphNode<'a, X, Y, F>
where
    X: Clone + Default,
    Y: Clone + Default,
    F: Fn(X) -> Y
{
    pub input_pin: GraphNodeInputPin<'a, X>,
    pub output_pin: GraphNodeOutputPin<'a, Y>,
    pub func: F
}

impl<'a, X, Y, F> TransformGraphNode<'a, X, Y, F>
where
    X: Clone + Default,
    Y: Clone + Default,
    F: Fn(X) -> Y
{
    pub fn new(func: F) -> Self {
        Self {
            input_pin: Default::default(),
            output_pin: Default::default(),
            func
        }
    }
}

impl<'a, X, Y, F> GraphNode<'a> for TransformGraphNode<'a, X, Y, F>
where
    X: Clone + Default,
    Y: Clone + Default,
    F: Fn(X) -> Y
{
    fn process_backwards(&self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
        self.output_pin.pin.set_data((&self.func)(self.input_pin.pin.get_data()));
    }

    fn process_forwards(&self) {
        self.output_pin.pin.set_data((&self.func)(self.input_pin.pin.get_data()));
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }
    
    fn setup(&'a self) {
        self.input_pin.pin.setup(self);
        self.output_pin.pin.setup(self);
    }
}

struct NodeGraph<'a> {
    nodes: Vec<Box<dyn GraphNode<'a> + 'a>>
}

#[test]
fn test_me() {
    let foo = |x: i32| -> i32 { x * x };

    let source_node = SourceGraphNode::<'_, i32>::default();
    let target_node = TargetGraphNode::<'_, i32>::default();
    let transform_node = TransformGraphNode::new(foo);

    let graph = NodeGraph {
        nodes: vec![
            Box::new(source_node),
            Box::new(target_node),
            Box::new(transform_node)
        ]
    };

    // We cannot turn static NodeGraph with lifetime references into dynamic
    // graph by boxing the nodes. This is because boxing nodes would allow nodes
    // to live different lifetime and that prohibits cyclic references created
    // when connecting nodes, and self-references when we box the nodes.
    
    //(&graph.nodes[0]).setup();
}

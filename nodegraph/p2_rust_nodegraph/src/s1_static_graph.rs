use std::cell::RefCell;

pub trait GraphNode<'a>
{
    fn process_forwards(&'a self);
    fn process_backwards(&'a self);
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
    pub fn get_node(&'a self) -> Option<&'a dyn GraphNode<'a>> {
        *self.node.borrow()
    }

    pub fn get_connection(&'a self) -> Option<&'a GraphNodePin<'a, T>> {
        *self.connection.borrow()
    }

    pub fn setup(&'a self, node: &'a dyn GraphNode<'a>) {
        self.node.replace(node.into());
    }

    pub fn connect(&'a self, pin: &'a GraphNodePin<'a, T>) {
        self.connection.replace(pin.into());
    }

    pub fn set_data(&'a self, data: T) {
        self.data.replace(data);
    }

    pub fn get_data(&'a self) -> T {
        self.data.borrow().clone()
    }
}

#[derive(Default)]
pub struct GraphNodeInputPin<'a, T> where T: Clone + Default
{
    pin: GraphNodePin<'a, T>
}

impl<'a, T> GraphNodeInputPin<'a, T> where T: Clone + Default
{
    pub fn connect(&'a self, output_pin: &'a GraphNodeOutputPin<'a, T>) {
        self.pin.connect(&output_pin.pin);
        output_pin.pin.connect(&self.pin);
    }

    pub fn receive_data(&'a self) {
        if let Some(input) = self.pin.get_connection() {
            self.pin.set_data(input.get_data());
        }
    }

    pub fn propagate_backwards(&'a self) {
        if let Some(input) = self.pin.get_connection() {
            if let Some(node) = input.get_node() {
                node.process_backwards();
            }
        }
    }
}

#[derive(Default)]
pub struct GraphNodeOutputPin<'a, T> where T: Clone + Default
{
    pin: GraphNodePin<'a, T>
}

impl<'a, T> GraphNodeOutputPin<'a, T> where T: Clone + Default
{
    pub fn connect(&'a self, input_pin: &'a GraphNodeInputPin<'a, T>) {
        self.pin.connect(&input_pin.pin);
        input_pin.pin.connect(&self.pin);
    }

    pub fn send_data(&'a self) {
        if let Some(output) = self.pin.get_connection() {
            output.set_data(self.pin.get_data());
        }
    }
    
    pub fn propagate_forwards(&'a self) {
        if let Some(output) = self.pin.get_connection() {
            if let Some(node) = output.get_node() {
                node.process_forwards();
            }
        }
    }
}

#[derive(Default)]
pub struct SourceGraphNode<'a, T> where T: Clone + Default
{
    pub output_pin: GraphNodeOutputPin<'a, T>
}

impl<'a, T> SourceGraphNode<'a, T> where T: Clone + Default
{
    pub fn setup(&'a self) {
        self.output_pin.pin.setup(self);
    }

    pub fn load_value(&'a self, data: T)
    {
        self.output_pin.pin.set_data(data);
    }
}

impl<'a, T> GraphNode<'a> for SourceGraphNode<'a, T> where T: Clone + Default
{
    fn process_backwards(&'a self) {
    }

    fn process_forwards(&'a self) {
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }
}

#[derive(Default)]
pub struct TargetGraphNode<'a, T> where T: Clone + Default
{
    pub input_pin: GraphNodeInputPin<'a, T>
}

impl<'a, T> TargetGraphNode<'a, T> where T: Clone + Default
{
    pub fn setup(&'a self) {
        self.input_pin.pin.setup(self);
    }

    pub fn get_value(&'a self) -> T
    {
        self.input_pin.pin.get_data()
    }
}

impl<'a, T> GraphNode<'a> for TargetGraphNode<'a, T> where T: Clone + Default
{
    fn process_backwards(&'a self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
    }

    fn process_forwards(&'a self) {
    }
}

pub struct TransformGraphNode<'a, X, Y, F>
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
    
    pub fn setup(&'a self) {
        self.input_pin.pin.setup(self);
        self.output_pin.pin.setup(self);
    }
}

impl<'a, X, Y, F> GraphNode<'a> for TransformGraphNode<'a, X, Y, F>
where
    X: Clone + Default,
    Y: Clone + Default,
    F: Fn(X) -> Y
{
    fn process_backwards(&'a self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
        self.output_pin.pin.set_data((self.func)(self.input_pin.pin.get_data()));
    }

    fn process_forwards(&'a self) {
        self.output_pin.pin.set_data((self.func)(self.input_pin.pin.get_data()));
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }
}


#[test]
fn test_me() {
    let foo = |x: i32| -> i32 { x * x };

    let source_node = SourceGraphNode::default();
    let target_node = TargetGraphNode::default();
    let transform_node = TransformGraphNode::new(foo);

    source_node.setup();
    target_node.setup();
    transform_node.setup();

    transform_node.input_pin.connect(&source_node.output_pin);
    transform_node.output_pin.connect(&target_node.input_pin);

    source_node.load_value(10);
    source_node.process_forwards();
    let value1 = target_node.get_value();

    println!("Result of process forwards: {}", value1);
    
    source_node.load_value(20);
    target_node.process_backwards();
    let value2 = target_node.get_value();
    
    println!("Result of process backwards: {}", value2);
}
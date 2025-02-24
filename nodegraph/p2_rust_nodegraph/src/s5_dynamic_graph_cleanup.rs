use std::{cell::RefCell, error::Error, rc::{Rc, Weak}};

use downcast_rs::{impl_downcast, Downcast};

pub trait GraphNodePinBase : Downcast
{
    fn try_connect(&mut self, other: Rc<RefCell<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>>;
    
    fn is_connected(&self) -> bool;
    fn disconnect(&mut self);
}

impl_downcast!(GraphNodePinBase);

pub trait GraphNode : Downcast
{
    fn process_forwards(&self);
    fn process_backwards(&self);

    fn get_inputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>>;
    fn get_outputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>>;
}

impl_downcast!(GraphNode);

struct GraphNodePin<T> where T: Clone + Default
{
    node: Weak<dyn GraphNode>,
    data: T
}

impl<T> GraphNodePin<T> where T: Clone + Default + 'static
{
    pub fn new(node: Weak<dyn GraphNode>) -> Self {
        Self {
            node,
            data: Default::default()
        }
    }

    pub fn get_node(&self) -> Option<Rc<dyn GraphNode>> {
        self.node.upgrade()
    }

    pub fn set_data(&mut self, data: T) {
        self.data = data;
    }

    pub fn get_data(&self) -> &T {
        &self.data
    }
}

pub struct GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pin: GraphNodePin<T>,
    me: Weak<RefCell<GraphNodeInputPin<T>>>,
    connection: Weak<RefCell<GraphNodeOutputPin<T>>>
}

impl<T> GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode>) -> Rc<RefCell<Self>> {
        Rc::new_cyclic(|me|{
            RefCell::new(Self {
                pin: GraphNodePin::new(node),
                me: me.clone(),
                connection: Default::default()
            })
        })
    }

    pub fn connect(&mut self, output_pin: &mut GraphNodeOutputPin<T>) {
        self.connect_internal(output_pin);
        output_pin.connect_internal(self);
    }

    fn connect_internal(&mut self, output_pin: &GraphNodeOutputPin<T>) {
        self.connection = output_pin.me.clone();
    }

    pub fn get_connection(&self) -> Option<Rc<RefCell<GraphNodeOutputPin<T>>>> {
        self.connection.upgrade()
    }

    pub fn receive_data(&mut self) {
        if let Some(output) = self.get_connection() {
            self.pin.set_data(output.borrow().pin.get_data().clone());
        }
    }

    fn get_output_node(&self) -> Option<Rc<dyn GraphNode>> {
        match self.get_connection() {
            Some(output) => output.borrow().pin.get_node().clone(),
            _ => None
        }
    }

    pub fn propagate_backwards(&self) {
        if let Some(node) = self.get_output_node() {
            node.process_backwards();
        }
    }
}

impl<T> GraphNodePinBase for GraphNodeInputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&mut self, other: Rc<RefCell<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>> {
        match other.borrow_mut().downcast_mut::<GraphNodeOutputPin<T>>() {
            Some(other_pin) => {
                self.connect(other_pin);
                Ok(())
            },
            None => {
                let type_name = std::any::type_name::<T>();
                let error_text= format!("Cannot cast to type {}", type_name);
                Err(error_text.into())
            }
        }
    }

    fn is_connected(&self) -> bool {
        self.connection.upgrade().is_some()
    }

    fn disconnect(&mut self) {
        let output_weak = self.connection.clone();
        self.connection = Default::default();

        if let Some(output) = output_weak.upgrade() {
            output.borrow_mut().disconnect_pin(self);
        }
    }

}

pub struct GraphNodeOutputPin<T> where T: Clone + Default + 'static 
{
    pin: GraphNodePin<T>,
    me: Weak<RefCell<GraphNodeOutputPin<T>>>,
    connections: Vec<Weak<RefCell<GraphNodeInputPin<T>>>>
}

impl<T> GraphNodeOutputPin<T> where T: Clone + Default + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode>) -> Rc<RefCell<Self>> {
        Rc::new_cyclic(|me|{
            RefCell::new(Self {
                pin: GraphNodePin::new(node),
                me: me.clone(),
                connections: Default::default()
            })
        })
    }

    pub fn connect(&mut self, input_pin: &mut GraphNodeInputPin<T>) {
        self.connect_internal(input_pin);
        input_pin.connect_internal(self);
    }

    fn connect_internal(&mut self, input_pin: &GraphNodeInputPin<T>) {
        self.connections.push(input_pin.me.clone());
    }

    pub fn disconnect_pin(&mut self, input_pin: &GraphNodeInputPin<T>) {
        self.connections.retain(
            |connection| !connection.ptr_eq(&input_pin.me));
    }

    pub fn get_connections(&self) -> Vec<Rc<RefCell<GraphNodeInputPin<T>>>> {
        self.connections.iter().filter_map(
            |connection| connection.upgrade()).collect()
    }

    pub fn send_data(&self) {
        for input in self.get_connections().iter() {
            input.borrow_mut().pin.set_data(self.pin.get_data().clone());
        }
    }
    
    pub fn propagate_forwards(&self) {
        for input in self.get_connections().iter() {
            if let Some(node) = input.borrow().pin.get_node() {
                node.process_forwards();
            }
        }
    }
}

impl<T> GraphNodePinBase for GraphNodeOutputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&mut self, other: Rc<RefCell<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>> {
        match other.borrow_mut().downcast_mut::<GraphNodeInputPin<T>>() {
            Some(other_pin) => {
                self.connect(other_pin);
                Ok(())
            },
            None => {
                let type_name = std::any::type_name::<T>();
                let error_text= format!("Cannot cast to type {}", type_name);
                Err(error_text.into())
            }
        }
    }

    fn is_connected(&self) -> bool {
        !self.get_connections().is_empty()
    }

    fn disconnect(&mut self) {
        let connections = self.get_connections();
        self.connections.clear();

        for input in connections.iter() {
            input.borrow_mut().disconnect();
        }
    }

}

pub struct SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub output_pin: Rc<RefCell<GraphNodeOutputPin<T>>>
}

impl<T> SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub fn new_rc() -> Rc<dyn GraphNode> {
        Rc::new_cyclic(|me| {
            Self {
                output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode>)
            }
        })
    }

    pub fn load_value(&self, data: T) {
        self.output_pin.borrow_mut().pin.set_data(data);
    }
}

impl<T> GraphNode<> for SourceGraphNode<T> where T: Clone + Default + 'static
{
    fn process_backwards(&self) {
    }

    fn process_forwards(&self) {
        let borrowed = self.output_pin.borrow();
        borrowed.send_data();
        borrowed.propagate_forwards();
    }

    fn get_inputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        None
    }

    fn get_outputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct TargetGraphNode<T> where T: Clone + Default + 'static
{
    pub input_pin: Rc<RefCell<GraphNodeInputPin<T>>>
}

impl<T> TargetGraphNode<T> where T: Clone + Default
{
    pub fn new_rc() -> Rc<dyn GraphNode> {
        Rc::new_cyclic(|me| {
            Self {
                input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode>)
            }
        })
    }

    pub fn get_value(&self) -> T {
        self.input_pin.borrow().pin.get_data().clone()
    }
}

impl<T> GraphNode<> for TargetGraphNode<T> where T: Clone + Default + 'static
{
    fn process_backwards(&self) {
        self.input_pin.borrow().propagate_backwards();
        self.input_pin.borrow_mut().receive_data();
    }

    fn process_forwards(&self) {
    }

    fn get_inputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        None
    }
}

pub struct TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y + 'static
{
    pub input_pin: Rc<RefCell<GraphNodeInputPin<X>>>,
    pub output_pin: Rc<RefCell<GraphNodeOutputPin<Y>>>,
    pub func: F
}

impl<X, Y, F> TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y
{
    pub fn new_rc(func: F) -> Rc<dyn GraphNode> {
        Rc::new_cyclic(|me| {
            Self {
                input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode>),
                output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode>),
                func
            }
        })
    }
    
    fn send_data(&self, data: Y) {
        let mut pin_write = self.output_pin.borrow_mut();
        pin_write.pin.set_data(data);
        pin_write.send_data();
    }

    fn receive_data(&self) -> X {
        let mut pin_write = self.input_pin.borrow_mut();
        pin_write.receive_data();
        pin_write.pin.get_data().clone()
    }
}

impl<X, Y, F> GraphNode<> for TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y
{
    fn process_backwards(&self) {
        self.input_pin.borrow().propagate_backwards();
        let result = (self.func)(self.receive_data());
        self.output_pin.borrow_mut().pin.set_data(result);
    }

    fn process_forwards(&self) {
        let x = self.input_pin.borrow().pin.get_data().clone();
        self.send_data((self.func)(x));
        self.output_pin.borrow().propagate_forwards();
    }

    
    fn get_inputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Rc<RefCell<dyn GraphNodePinBase>>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct NodeGraph {
    nodes: Vec<Rc<dyn GraphNode>>
}

impl NodeGraph {
    pub fn new(nodes: Vec<Rc<dyn GraphNode>>) -> Self {
        Self {
            nodes
        }
    }

    pub fn get_matching_nodes<F>(&self, f: F) -> Vec<Rc<dyn GraphNode>> 
        where F: Fn(&&Rc<dyn GraphNode>) -> bool + 'static {
           self.nodes.iter().filter(f).cloned().collect()
    }

    pub fn get_source_nodes(&self) -> Vec<Rc<dyn GraphNode>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_none() && node.get_outputs().is_some())
    }
    
    pub fn get_target_nodes(&self) -> Vec<Rc<dyn GraphNode>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_some() && node.get_outputs().is_none())
    }
    
    pub fn get_transform_nodes(&self) -> Vec<Rc<dyn GraphNode>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_some() && node.get_outputs().is_some())
    }
    
}

#[cfg(test)]
#[derive(Clone, Default)]
struct ExampleInput {
    x: i32,
    y: i32
}

#[cfg(test)]
#[derive(Clone, Default)]
struct ExampleResultU {
    u: f64
}

#[cfg(test)]
#[derive(Clone, Default)]
struct ExampleResultV {
    v: f64
}

#[cfg(test)]
fn build_some_graph() -> NodeGraph {
    let foo_u = |value: ExampleInput| ExampleResultU{ u: (value.x as f64) * (value.y as f64)};
    let foo_v = |value: ExampleInput| ExampleResultV{ v: (value.x as f64) / (value.y as f64)};

    let source_node = SourceGraphNode::<ExampleInput>::new_rc();
    let target_node_u = TargetGraphNode::<ExampleResultU>::new_rc();
    let target_node_v = TargetGraphNode::<ExampleResultV>::new_rc();
    let transform_node_u = TransformGraphNode::new_rc(foo_u);
    let transform_node_v = TransformGraphNode::new_rc(foo_v);

    NodeGraph::new(vec![
        source_node,
        target_node_u,
        target_node_v,
        transform_node_u,
        transform_node_v])
}

#[cfg(test)]
fn connect_output_and_input_nodes(output_node: &Rc<dyn GraphNode>, input_node: &Rc<dyn GraphNode>)
 -> Result<(), Box<dyn Error + 'static>>
{
    let output_pins= output_node.get_outputs().ok_or("Cannot get output output")?;
    let input_pins= input_node.get_inputs().ok_or("Cannot get input input")?;

    assert_eq!(output_pins.len(), 1);
    assert_eq!(input_pins.len(), 1);

    input_pins[0].borrow_mut().try_connect(output_pins[0].clone())?;

    Ok(())
}

#[cfg(test)]
fn find_matching_transform_node_by_output_data_type<T>(graph: &NodeGraph) -> Option<Rc<dyn GraphNode>>
where T: Clone + Default + 'static {

    graph.get_transform_nodes().iter().find(|node| 
            node.get_outputs().and_then(|pins| 
                pins.iter().find(|pin| 
                    (*pin).clone().borrow().downcast_ref::<GraphNodeOutputPin<T>>().is_some()
                ).map(|_| true)
            ).is_some()
        ).cloned()
}

#[cfg(test)]
fn find_matching_target_node_by_input_data_type<T>(graph: &NodeGraph) -> Option<Rc<dyn GraphNode>>
where T: Clone + Default + 'static {

    graph.get_target_nodes().iter().find(|node| 
        (*node).clone().downcast_ref::<TargetGraphNode<T>>().is_some()
    ).cloned()
}

#[cfg(test)]
fn connect_some_nodes(graph: &NodeGraph)
 -> Result<(Rc<dyn GraphNode>, Rc<dyn GraphNode>, Rc<dyn GraphNode>),
 Box<dyn Error + 'static>> {

    let source_node = graph.get_source_nodes().first().ok_or("Cannot find source node")?.clone();
    
    let transform_node_u = find_matching_transform_node_by_output_data_type
        ::<ExampleResultU>(graph).ok_or("Cannot find transform node U")?.clone();

    let transform_node_v = find_matching_transform_node_by_output_data_type
        ::<ExampleResultV>(graph).ok_or("Cannot find transform node V")?.clone();

    let target_node_u = find_matching_target_node_by_input_data_type
        ::<ExampleResultU>(&graph).ok_or("Cannot find target node U")?.clone();

    let target_node_v = find_matching_target_node_by_input_data_type
        ::<ExampleResultV>(&graph).ok_or("Cannot find target node V")?.clone();

    connect_output_and_input_nodes(&source_node, &transform_node_u)?;
    connect_output_and_input_nodes(&source_node, &transform_node_v)?;

    connect_output_and_input_nodes(&transform_node_u, &target_node_u)?;
    connect_output_and_input_nodes(&transform_node_v, &target_node_v)?;

    Ok((source_node, target_node_u, target_node_v))
}

#[cfg(test)]
fn load_example_value(source_node: Rc<dyn GraphNode>, value: ExampleInput)
{
    if let Some(node) = source_node.downcast_ref::<SourceGraphNode<ExampleInput>>() {
        node.load_value(value);
    }
}

#[cfg(test)]
fn get_example_result<T>(target_node: Rc<dyn GraphNode>) -> Option<T>
where
    T: Default + Clone + 'static
{
    match target_node.downcast_ref::<TargetGraphNode<T>>() {
        Some(node) => Some(node.get_value()),
        _ => None
    }
}

#[test]
fn test_me() {

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node_u, target_node_v) = connect_result.unwrap();

    load_example_value(source_node.clone(), ExampleInput{x: 10, y:20});
    source_node.process_forwards();
    let value1u: ExampleResultU = get_example_result(target_node_u.clone()).unwrap();
    let value1v: ExampleResultV = get_example_result(target_node_v.clone()).unwrap();

    println!("Result of process forwards: {}, {}", value1u.u, value1v.v);

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node_u, target_node_v) = connect_result.unwrap();

    load_example_value(source_node.clone(), ExampleInput{x:20, y:80});
    target_node_u.process_backwards();
    target_node_v.process_backwards();
    let value2u: ExampleResultU = get_example_result(target_node_u.clone()).unwrap();
    let value2v: ExampleResultV = get_example_result(target_node_v.clone()).unwrap();

    println!("Result of process backwards: {}, {}", value2u.u, value2v.v);

}

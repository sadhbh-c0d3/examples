use std::{cell::RefCell, error::Error, rc::{Rc, Weak}};

use downcast_rs::{impl_downcast, Downcast};

pub trait GraphNodePinBase : Downcast
{
    fn try_connect(&self, other: Rc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>>;
    
    fn is_connected(&self) -> bool;
    fn disconnect(&self);
}

impl_downcast!(GraphNodePinBase);

pub trait GraphNode : Downcast
{
    fn process_forwards(&self);
    fn process_backwards(&self);

    fn get_inputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>>;
    fn get_outputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>>;
}

impl_downcast!(GraphNode);

struct GraphNodePin<T> where T: Clone + Default
{
    node: Weak<Box<dyn GraphNode>>,
    connection: RefCell<Option<Weak<GraphNodePin<T>>>>,
    data: RefCell<T>
}

impl<T> GraphNodePin<T> where T: Clone + Default + 'static
{
    pub fn new(node: Weak<Box<dyn GraphNode>>) -> Self {
        Self {
            node,
            connection: Default::default(),
            data: Default::default()
        }
    }

    pub fn get_node(&self) -> Option<Rc<Box<dyn GraphNode>>> {
        self.node.upgrade()
    }

    pub fn get_connection(&self) -> Option<Rc<GraphNodePin<T>>> {
        self.connection.borrow().as_ref().unwrap().upgrade()
    }

    pub fn connect(&self, pin: Rc<GraphNodePin<T>>) {
        let weak = Rc::downgrade(&pin);
        self.connection.replace(weak.into());
    }

    pub fn set_data(&self, data: T) {
        self.data.replace(data);
    }

    pub fn get_data(&self) -> T {
        self.data.borrow().clone()
    }
    
    fn is_connected(&self) -> bool {
        match self.connection.borrow().as_ref() {
            Some(connection_weak) => connection_weak.upgrade().is_some(),
            _ => false
        }
    }

    fn disconnect(&self) {
        if let Some(connection_weak) = self.connection.replace(None) {
            if let Some(connection) = connection_weak.upgrade() {
                connection.disconnect();
            }
        }
    }
}

#[derive(Clone)]
pub struct GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pin: Rc<GraphNodePin<T>>
}

impl<T> GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pub fn connect(&self, output_pin: &GraphNodeOutputPin<T>) {
        self.pin.connect(output_pin.pin.clone());
        output_pin.pin.connect(self.pin.clone());
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

impl<T> GraphNodePinBase for GraphNodeInputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&self, other: Rc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>> {
        match other.downcast_ref::<GraphNodeOutputPin<T>>() {
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
        self.pin.is_connected()
    }

    fn disconnect(&self) {
        self.pin.disconnect();
    }
}

#[derive(Clone)]
pub struct GraphNodeOutputPin<T> where T: Clone + Default + 'static 
{
    pin: Rc<GraphNodePin<T>>
}

impl<T> GraphNodeOutputPin<T> where T: Clone + Default + 'static
{
    pub fn connect(&self, input_pin: &GraphNodeInputPin<T>) {
        self.pin.connect(input_pin.pin.clone());
        input_pin.pin.connect(self.pin.clone());
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

impl<T> GraphNodePinBase for GraphNodeOutputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&self, other: Rc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>> {
        match other.downcast_ref::<GraphNodeInputPin<T>>() {
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
        self.pin.is_connected()
    }

    fn disconnect(&self) {
        self.pin.disconnect();
    }
}

pub struct SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub output_pin: GraphNodeOutputPin<T>
}

impl<T> SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub fn new_rc() -> Rc<Box<dyn GraphNode>> {
        Rc::new_cyclic(|me| {
            Box::new(Self {
                    output_pin: GraphNodeOutputPin {
                        pin: Rc::new(GraphNodePin::new(me.clone()))
                    }
                })
        })
    }

    pub fn load_value(&self, data: T) {
        self.output_pin.pin.set_data(data);
    }
}

impl<T> GraphNode<> for SourceGraphNode<T> where T: Clone + Default + 'static
{
    fn process_backwards(&self) {
    }

    fn process_forwards(&self) {
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }

    fn get_inputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        None
    }

    fn get_outputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        let pin = Rc::new(self.output_pin.clone());
        Some(vec![pin])
    }
}

pub struct TargetGraphNode<T> where T: Clone + Default + 'static
{
    pub input_pin: GraphNodeInputPin<T>
}

impl<T> TargetGraphNode<T> where T: Clone + Default
{
    pub fn new_rc() -> Rc<Box<dyn GraphNode>> {
        Rc::new_cyclic(|me| {
            Box::new(Self {
                    input_pin: GraphNodeInputPin {
                        pin: Rc::new(GraphNodePin::new(me.clone()))
                    }
                })
        })
    }

    pub fn get_value(&self) -> T {
        self.input_pin.pin.get_data()
    }
}

impl<T> GraphNode<> for TargetGraphNode<T> where T: Clone + Default + 'static
{
    fn process_backwards(&self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
    }

    fn process_forwards(&self) {
    }

    fn get_inputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        let pin = Rc::new(self.input_pin.clone());
        Some(vec![pin])
    }

    fn get_outputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        None
    }
}

pub struct TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y + 'static
{
    pub input_pin: GraphNodeInputPin<X>,
    pub output_pin: GraphNodeOutputPin<Y>,
    pub func: F
}

impl<X, Y, F> TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y
{
    pub fn new_rc(func: F) -> Rc<Box<dyn GraphNode>> {
        Rc::new_cyclic(|me| {
            Box::new(Self {
                    input_pin: GraphNodeInputPin {
                        pin: Rc::new(GraphNodePin::new(me.clone()))
                    },
                    output_pin: GraphNodeOutputPin {
                        pin: Rc::new(GraphNodePin::new(me.clone()))
                    },
                    func
                })
        })
    }

}

impl<X, Y, F> GraphNode<> for TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y
{
    fn process_backwards(&self) {
        self.input_pin.propagate_backwards();
        self.input_pin.receive_data();
        self.output_pin.pin.set_data((self.func)(self.input_pin.pin.get_data()));
    }

    fn process_forwards(&self) {
        self.output_pin.pin.set_data((self.func)(self.input_pin.pin.get_data()));
        self.output_pin.send_data();
        self.output_pin.propagate_forwards();
    }
    
    fn get_inputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        let pin = Rc::new(self.input_pin.clone());
        Some(vec![pin])
    }

    fn get_outputs(&self) -> Option<Vec<Rc<dyn GraphNodePinBase>>> {
        let pin = Rc::new(self.output_pin.clone());
        Some(vec![pin])
    }
}

pub struct NodeGraph {
    nodes: Vec<Rc<Box<dyn GraphNode>>>
}

impl NodeGraph {
    pub fn new(nodes: Vec<Rc<Box<dyn GraphNode>>>) -> Self {
        Self {
            nodes
        }
    }

    pub fn get_source_nodes(&self) -> Vec<Rc<Box<dyn GraphNode>>> {
        self.nodes.iter().filter(|node| {
            node.get_inputs().is_none() && node.get_outputs().is_some()
        }).cloned().collect()
    }
    
    pub fn get_target_nodes(&self) -> Vec<Rc<Box<dyn GraphNode>>> {
        self.nodes.iter().filter(|node| {
            node.get_inputs().is_some() && node.get_outputs().is_none()
        }).cloned().collect()
    }
    
    pub fn get_transform_nodes(&self) -> Vec<Rc<Box<dyn GraphNode>>> {
        self.nodes.iter().filter(|node| {
            node.get_inputs().is_some() && node.get_outputs().is_some()
        }).cloned().collect()
    }
}

#[cfg(test)]
fn build_some_graph() -> NodeGraph {
    let foo = |x: i32| -> i32 { x * x };

    let source_node = SourceGraphNode::<i32>::new_rc();
    let target_node = TargetGraphNode::<i32>::new_rc();
    let transform_node = TransformGraphNode::new_rc(foo);

    NodeGraph::new(vec![
        source_node,
        target_node,
        transform_node])
}

#[cfg(test)]
fn connect_some_nodes(graph: &NodeGraph) -> Result<(Rc<Box<dyn GraphNode>>, Rc<Box<dyn GraphNode>>), Box<dyn Error + 'static>>{

    let source_node = graph.get_source_nodes().first().ok_or("Cannot find source node")?.clone();
    let transform_node = graph.get_transform_nodes().first().ok_or("Cannot find transform node")?.clone();
    let target_node = graph.get_target_nodes().first().ok_or("Cannot find target node")?.clone();

    let source_outputs= source_node.get_outputs().ok_or("Cannot get source output")?;
    let transform_inputs = transform_node.get_inputs().ok_or("Cannot get transfom input")?;
    let transform_outputs= transform_node.get_outputs().ok_or("Cannot get transform output")?;
    let target_inputs= target_node.get_inputs().ok_or("Cannot get target input")?;

    assert_eq!(source_outputs.len(), 1);
    assert_eq!(transform_inputs.len(), 1);
    assert_eq!(transform_outputs.len(), 1);
    assert_eq!(target_inputs.len(), 1);

    transform_inputs[0].try_connect(source_outputs[0].clone())?;
    transform_outputs[0].try_connect(target_inputs[0].clone())?;

    Ok((source_node, target_node))
}

#[test]
fn test_me() {

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node) = connect_result.unwrap();

    let source_node_downcast_result = source_node.downcast_ref::<SourceGraphNode<i32>>();
    let target_node_downcast_result = target_node.downcast_ref::<TargetGraphNode<i32>>();
    assert!(source_node_downcast_result.is_some());
    assert!(target_node_downcast_result.is_some());

    let source_node_source = source_node_downcast_result.unwrap();
    let target_node_target = target_node_downcast_result.unwrap();

    source_node_source.load_value(10);
    source_node.process_forwards();
    let value1 = target_node_target.get_value();

    println!("Result of process forwards: {}", value1);
    
    source_node_source.load_value(20);
    target_node.process_backwards();
    let value2 = target_node_target.get_value();

    println!("Result of process backwards: {}", value2);

}

use std::{error::Error, sync::{Arc, RwLock, Weak}};

use downcast_rs::{impl_downcast, Downcast};

pub trait GraphNodePinBase : Downcast
{
    fn try_connect(&self, other: Arc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>>;
    
    fn is_connected(&self) -> bool;
    fn disconnect(&self);
}

impl_downcast!(GraphNodePinBase);

pub trait GraphNode : Downcast
{
    fn process_forwards(&self);
    fn process_backwards(&self);

    fn get_inputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>>;
    fn get_outputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>>;
}

impl_downcast!(GraphNode);

struct GraphNodePin<T> where T: Clone + Default
{
    node: Weak<dyn GraphNode>,
    data: RwLock<T>
}

impl<T> GraphNodePin<T> where T: Clone + Default + 'static
{
    pub fn new(node: Weak<dyn GraphNode>) -> Self {
        Self {
            node,
            data: Default::default()
        }
    }

    pub fn get_node(&self) -> Option<Arc<dyn GraphNode>> {
        self.node.upgrade()
    }

    pub fn set_data(&self, data: T) {
        *self.data.write().unwrap() = data;
    }

    pub fn get_data(&self) -> T {
        self.data.read().unwrap().clone()
    }
}

pub struct GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pin: GraphNodePin<T>,
    me: Weak<GraphNodeInputPin<T>>,
    connection: RwLock<Option<Weak<GraphNodeOutputPin<T>>>>
}

impl<T> GraphNodeInputPin<T> where T: Clone + Default + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode>) -> Arc<Self> {
        Arc::new_cyclic(|me|{
            Self {
                pin: GraphNodePin::new(node),
                me: me.clone(),
                connection: Default::default()
            }
        })
    }

    pub fn connect(&self, output_pin: &GraphNodeOutputPin<T>) {
        self.connect_internal(output_pin);
        output_pin.connect_internal(self);
    }

    fn connect_internal(&self, output_pin: &GraphNodeOutputPin<T>) {
        *self.connection.write().unwrap() = output_pin.me.clone().into();
    }

    pub fn get_connection(&self) -> Option<Arc<GraphNodeOutputPin<T>>> {
        self.connection.read().unwrap().as_ref().unwrap().upgrade()
    }

    pub fn receive_data(&self) {
        if let Some(output) = self.get_connection() {
            self.pin.set_data(output.pin.get_data());
        }
    }

    pub fn propagate_backwards(&self) {
        if let Some(output) = self.get_connection() {
            if let Some(node) = output.pin.get_node() {
                node.process_backwards();
            }
        }
    }
}

impl<T> GraphNodePinBase for GraphNodeInputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&self, other: Arc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>> {
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
        match self.connection.read().unwrap().as_ref() {
            Some(connection_weak) => connection_weak.upgrade().is_some(),
            _ => false
        }
    }

    fn disconnect(&self) {
        let connection = self.get_connection();
        *self.connection.write().unwrap() = None;

        if let Some(output) = connection {
            output.disconnect_pin(self);
        }
    }

}

pub struct GraphNodeOutputPin<T> where T: Clone + Default + 'static 
{
    pin: GraphNodePin<T>,
    me: Weak<GraphNodeOutputPin<T>>,
    connections: RwLock<Vec<Weak<GraphNodeInputPin<T>>>>
}

impl<T> GraphNodeOutputPin<T> where T: Clone + Default + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode>) -> Arc<Self> {
        Arc::new_cyclic(|me|{
            Self {
                pin: GraphNodePin::new(node),
                me: me.clone(),
                connections: Default::default()
            }
        })
    }

    pub fn connect(&self, input_pin: &GraphNodeInputPin<T>) {
        self.connect_internal(input_pin);
        input_pin.connect_internal(self);
    }

    fn connect_internal(&self, input_pin: &GraphNodeInputPin<T>) {
        self.connections.write().unwrap().push(input_pin.me.clone());
    }

    pub fn disconnect_pin(&self, input_pin: &GraphNodeInputPin<T>) {
        self.connections.write().unwrap().retain(
            |connection| !connection.ptr_eq(&input_pin.me));
    }

    pub fn get_connections(&self) -> Vec<Arc<GraphNodeInputPin<T>>> {
        self.connections.read().unwrap().iter().filter_map(
            |connection| connection.upgrade()).collect()
    }

    pub fn send_data(&self) {
        for input in self.get_connections().iter() {
            input.pin.set_data(self.pin.get_data());
        }
    }
    
    pub fn propagate_forwards(&self) {
        for input in self.get_connections().iter() {
            if let Some(node) = input.pin.get_node() {
                node.process_forwards();
            }
        }
    }
}

impl<T> GraphNodePinBase for GraphNodeOutputPin<T> where T: Clone + Default + 'static {
    
    fn try_connect(&self, other: Arc<dyn GraphNodePinBase>) -> Result<(), Box<dyn Error + 'static>> {
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
        !self.get_connections().is_empty()
    }

    fn disconnect(&self) {
        let connections = self.get_connections();
        self.connections.write().unwrap().clear();

        for input in connections.iter() {
            input.disconnect();
        }
    }

}

pub struct SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub output_pin: Arc<GraphNodeOutputPin<T>>
}

impl<T> SourceGraphNode<T> where T: Clone + Default + 'static
{
    pub fn new_rc() -> Arc<dyn GraphNode> {
        Arc::new_cyclic(|me| {
            Self {
                output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode>)
            }
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

    fn get_inputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        None
    }

    fn get_outputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct TargetGraphNode<T> where T: Clone + Default + 'static
{
    pub input_pin: Arc<GraphNodeInputPin<T>>
}

impl<T> TargetGraphNode<T> where T: Clone + Default
{
    pub fn new_rc() -> Arc<dyn GraphNode> {
        Arc::new_cyclic(|me| {
            Self {
                input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode>)
            }
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

    fn get_inputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        None
    }
}

pub struct TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y + 'static
{
    pub input_pin: Arc<GraphNodeInputPin<X>>,
    pub output_pin: Arc<GraphNodeOutputPin<Y>>,
    pub func: F
}

impl<X, Y, F> TransformGraphNode<X, Y, F>
where
    X: Clone + Default + 'static,
    Y: Clone + Default + 'static,
    F: Fn(X) -> Y
{
    pub fn new_rc(func: F) -> Arc<dyn GraphNode> {
        Arc::new_cyclic(|me| {
            Self {
                    input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode>),
                    output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode>),
                    func
                }
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
    
    fn get_inputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Arc<dyn GraphNodePinBase>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct NodeGraph {
    nodes: Vec<Arc<dyn GraphNode>>
}

impl NodeGraph {
    pub fn new(nodes: Vec<Arc<dyn GraphNode>>) -> Self {
        Self {
            nodes
        }
    }

    pub fn get_matching_nodes<F>(&self, f: F) -> Vec<Arc<dyn GraphNode>> 
        where F: Fn(&&Arc<dyn GraphNode>) -> bool + 'static {
           self.nodes.iter().filter(f).cloned().collect()
    }

    pub fn get_source_nodes(&self) -> Vec<Arc<dyn GraphNode>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_none() && node.get_outputs().is_some())
    }
    
    pub fn get_target_nodes(&self) -> Vec<Arc<dyn GraphNode>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_some() && node.get_outputs().is_none())
    }
    
    pub fn get_transform_nodes(&self) -> Vec<Arc<dyn GraphNode>> {
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
fn connect_output_and_input_nodes(output_node: &Arc<dyn GraphNode>, input_node: &Arc<dyn GraphNode>) -> Result<(), Box<dyn Error + 'static>>
{
    let output_pins= output_node.get_outputs().ok_or("Cannot get output output")?;
    let input_pins= input_node.get_inputs().ok_or("Cannot get input input")?;

    assert_eq!(output_pins.len(), 1);
    assert_eq!(input_pins.len(), 1);

    input_pins[0].try_connect(output_pins[0].clone())?;

    Ok(())
}

#[cfg(test)]
fn find_matching_transform_node_by_output_data_type<T>(graph: &NodeGraph) -> Option<Arc<dyn GraphNode>> 
where T: Clone + Default + 'static {

    graph.get_transform_nodes().iter().find(|node| 
            node.get_outputs().and_then(|pins| 
                pins.iter().find(|pin| 
                    (*pin).clone().downcast_ref::<GraphNodeOutputPin<T>>().is_some()
                ).map(|_| true)
            ).is_some()
        ).cloned()
}

#[cfg(test)]
fn find_matching_target_node_by_input_data_type<T>(graph: &NodeGraph) -> Option<Arc<dyn GraphNode>>
where T: Clone + Default + 'static {

    graph.get_target_nodes().iter().find(|node| 
        (*node).clone().downcast_ref::<TargetGraphNode<T>>().is_some()
    ).cloned()
}

#[cfg(test)]
fn connect_some_nodes(graph: &NodeGraph) -> Result<(Arc<dyn GraphNode>, Arc<dyn GraphNode>, Arc<dyn GraphNode>), Box<dyn Error + 'static>>{

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

#[test]
fn test_me() {

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node_u, target_node_v) = connect_result.unwrap();

    let source_node_downcast_result = source_node.downcast_ref::<SourceGraphNode<ExampleInput>>();
    let target_node_u_downcast_result = target_node_u.downcast_ref::<TargetGraphNode<ExampleResultU>>();
    let target_node_v_downcast_result = target_node_v.downcast_ref::<TargetGraphNode<ExampleResultV>>();
    assert!(source_node_downcast_result.is_some());
    assert!(target_node_u_downcast_result.is_some());
    assert!(target_node_v_downcast_result.is_some());

    let source_node_source = source_node_downcast_result.unwrap();
    let target_node_u_target = target_node_u_downcast_result.unwrap();
    let target_node_v_target = target_node_v_downcast_result.unwrap();

    source_node_source.load_value(ExampleInput{x: 10, y:20});
    source_node.process_forwards();
    let value1u = target_node_u_target.get_value();
    let value1v = target_node_v_target.get_value();

    println!("Result of process forwards: {}, {}", value1u.u, value1v.v);
    
    source_node_source.load_value(ExampleInput{x:20, y:80});
    target_node_u.process_backwards();
    target_node_v.process_backwards();
    let value2u = target_node_u_target.get_value();
    let value2v = target_node_v_target.get_value();

    println!("Result of process backwards: {}, {}", value2u.u, value2v.v);

}

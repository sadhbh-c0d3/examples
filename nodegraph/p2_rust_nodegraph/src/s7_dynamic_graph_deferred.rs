use std::{error::Error, sync::{Arc, RwLock, Weak}};

use downcast_rs::{impl_downcast, Downcast};
use rayon::iter::{IntoParallelIterator, ParallelIterator};

pub enum Actionable {
    Single(Box<dyn Fn() -> Action + Send + Sync>),
    Sequential(Vec<Action>),
    Parallel(Vec<Action>)
}

type Action = Option<Actionable>;

pub fn defer_action(action: impl Fn() -> Action + Send + Sync + 'static) -> Action {
    Some(Actionable::Single(Box::new(action)))
}

pub fn defer_sequential(actions: Vec<Action>) -> Action {
    Some(Actionable::Sequential(actions))
}

pub fn defer_parallel(actions: Vec<Action>) -> Action {
    Some(Actionable::Parallel(actions))
}

pub trait ExecutionPolicy {
    fn apply_sequential(&self, v: Vec<Action>);
    fn apply_parallel(&self, v: Vec<Action>);
}

pub struct BasicPolicy;

impl ExecutionPolicy for BasicPolicy {
    fn apply_sequential(&self, v: Vec<Action>) {
        v.into_iter().for_each(|a| run_actions(self, a));
    }

    fn apply_parallel(&self, v: Vec<Action>) {
        v.into_iter().for_each(|a| run_actions(self, a));
    }
}

pub struct RayonPolicy;

impl ExecutionPolicy for RayonPolicy {
    fn apply_sequential(&self, v: Vec<Action>) {
        v.into_iter().for_each(|a| run_actions(self, a));
    }

    fn apply_parallel(&self, v: Vec<Action>) {
        v.into_par_iter().for_each(|a| run_actions(self, a));
    }
}

pub fn run_actions(policy: &impl ExecutionPolicy, mut action: Action) {
    while action.is_some() {
        action = match action.unwrap() {
            Actionable::Single(a) => a(),
            Actionable::Sequential(v) => {
                policy.apply_sequential(v);
                None
            },
            Actionable::Parallel(v) => {
                policy.apply_parallel(v);
                None
            }
        };
    }
}

pub trait GraphNodePinBase : Downcast
{
    fn try_connect(&mut self, other: Arc<RwLock<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>>;
    
    fn is_connected(&self) -> bool;
    fn disconnect(&mut self);
}

impl_downcast!(GraphNodePinBase);

pub trait GraphNode : Downcast
{
    fn process_forwards(&self) -> Action;
    fn process_backwards(&self) -> Action;

    fn get_inputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>>;
    fn get_outputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>>;
}

impl_downcast!(GraphNode);

struct GraphNodePin<T> where T: Clone + Default + Send + Sync + 'static
{
    node: Weak<dyn GraphNode + Send + Sync + 'static>,
    data: T
}

impl<T> GraphNodePin<T> where T: Clone + Default + Send + Sync + 'static
{
    pub fn new(node: Weak<dyn GraphNode + Send + Sync + 'static>) -> Self {
        Self {
            node,
            data: Default::default()
        }
    }

    pub fn get_node(&self) -> Option<Arc<dyn GraphNode + Send + Sync + 'static>> {
        self.node.upgrade()
    }

    pub fn set_data(&mut self, data: T) {
        self.data = data;
    }

    pub fn get_data(&self) -> &T {
        &self.data
    }
}

pub struct GraphNodeInputPin<T> where T: Clone + Default + Send + Sync + 'static
{
    pin: GraphNodePin<T>,
    me: Weak<RwLock<GraphNodeInputPin<T>>>,
    connection: Weak<RwLock<GraphNodeOutputPin<T>>>
}

impl<T> GraphNodeInputPin<T> where T: Clone + Default + Send + Sync + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode + Send + Sync + 'static>) -> Arc<RwLock<Self>> {
        Arc::new_cyclic(|me|{
            RwLock::new(Self {
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

    pub fn get_connection(&self) -> Option<Arc<RwLock<GraphNodeOutputPin<T>>>> {
        self.connection.upgrade()
    }

    pub fn receive_data(&mut self) {
        if let Some(output) = self.get_connection() {
            self.pin.set_data(output.read().unwrap().pin.get_data().clone());
        }
    }

    fn get_output_node(&self) -> Option<Arc<dyn GraphNode + Send + Sync + 'static>> {
        match self.get_connection() {
            Some(output) => output.read().unwrap().pin.get_node().clone(),
            _ => None
        }
    }

    pub fn propagate_backwards(&self) -> Action {
        match self.get_output_node() {
            Some(node) => node.process_backwards(),
            _ => None
        }
    }
}

impl<T> GraphNodePinBase for GraphNodeInputPin<T> where T: Clone + Default + Send + Sync + 'static {
    
    fn try_connect(&mut self, other: Arc<RwLock<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>> {
        match other.write().unwrap().downcast_mut::<GraphNodeOutputPin<T>>() {
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
            output.write().unwrap().disconnect_pin(self);
        }
    }

}

pub struct GraphNodeOutputPin<T> where T: Clone + Default + Send + Sync + 'static 
{
    pin: GraphNodePin<T>,
    me: Weak<RwLock<GraphNodeOutputPin<T>>>,
    connections: Vec<Weak<RwLock<GraphNodeInputPin<T>>>>
}

impl<T> GraphNodeOutputPin<T> where T: Clone + Default + Send + Sync + 'static
{
    pub fn new_rc(node: Weak<dyn GraphNode + Send + Sync + 'static>) -> Arc<RwLock<Self>> {
        Arc::new_cyclic(|me|{
            RwLock::new(Self {
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

    pub fn get_connections(&self) -> Vec<Arc<RwLock<GraphNodeInputPin<T>>>> {
        self.connections.iter().filter_map(
            |connection| connection.upgrade()).collect()
    }

    pub fn send_data(&self) {
        for input in self.get_connections().iter() {
            input.write().unwrap().pin.set_data(self.pin.get_data().clone());
        }
    }
    
    pub fn propagate_forwards(&self) -> Action {
        let actions = self.get_connections().iter().map(
            |input|
                match input.read().unwrap().pin.get_node() {
                    Some(node) => node.process_forwards(),
                    _ => None
                }).collect();
        defer_parallel(actions)
    }
}

impl<T> GraphNodePinBase for GraphNodeOutputPin<T> where T: Clone + Default + Send + Sync + 'static {
    
    fn try_connect(&mut self, other: Arc<RwLock<dyn GraphNodePinBase>>) -> Result<(), Box<dyn Error + 'static>> {
        match other.write().unwrap().downcast_mut::<GraphNodeInputPin<T>>() {
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
            input.write().unwrap().disconnect();
        }
    }

}

pub struct SourceGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    pub output_pin: Arc<RwLock<GraphNodeOutputPin<T>>>
}

impl<T> SourceGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    pub fn new_rc() -> Arc<dyn GraphNode + Send + Sync> {
        Arc::new_cyclic(|me| {
            Self {
                output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode + Send + Sync + 'static>)
            }
        })
    }

    pub fn load_value(&self, data: T) {
        self.output_pin.write().unwrap().pin.set_data(data);
    }
}

impl<T> GraphNode<> for SourceGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    fn process_backwards(&self) -> Action {
        None
    }

    fn process_forwards(&self) -> Action {
        let pin = self.output_pin.clone();
        defer_action(move || {
            let guard = pin.read().unwrap();
            guard.send_data();
            guard.propagate_forwards()
        })
    }

    fn get_inputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        None
    }

    fn get_outputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct TargetGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    pub input_pin: Arc<RwLock<GraphNodeInputPin<T>>>
}

impl<T> TargetGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    pub fn new_rc() -> Arc<dyn GraphNode + Send + Sync> {
        Arc::new_cyclic(|me| {
            Self {
                input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode + Send + Sync + 'static>)
            }
        })
    }

    pub fn get_value(&self) -> T {
        self.input_pin.read().unwrap().pin.get_data().clone()
    }
}

impl<T> GraphNode<> for TargetGraphNode<T> where T: Clone + Default + Send + Sync + 'static
{
    fn process_backwards(&self) -> Action {
        let pin1 = self.input_pin.clone();
        let pin2 = self.input_pin.clone();
        defer_sequential(vec![
            defer_action(move || { pin1.read().unwrap().propagate_backwards() }),
            defer_action(move || { pin2.write().unwrap().receive_data(); None })])
    }

    fn process_forwards(&self) -> Action {
        None
    }

    fn get_inputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        None
    }
}

pub struct TransformGraphNode<X, Y, F>
where
    X: Clone + Default + Send + Sync + 'static,
    Y: Clone + Default + Send + Sync + 'static,
    F: Fn(X) -> Y + Send + Sync + 'static
{
    pub input_pin: Arc<RwLock<GraphNodeInputPin<X>>>,
    pub output_pin: Arc<RwLock<GraphNodeOutputPin<Y>>>,
    pub func: F
}

impl<X, Y, F> TransformGraphNode<X, Y, F>
where
    X: Clone + Default + Send + Sync + 'static,
    Y: Clone + Default + Send + Sync + 'static,
    F: Fn(X) -> Y + Clone + Send + Sync + 'static
{
    pub fn new_rc(func: F) -> Arc<dyn GraphNode + Send + Sync + 'static> {
        Arc::new_cyclic(|me| {
            Self {
                input_pin: GraphNodeInputPin::new_rc(me.clone() as Weak<dyn GraphNode + Send + Sync + 'static>),
                output_pin: GraphNodeOutputPin::new_rc(me.clone() as Weak<dyn GraphNode + Send + Sync + 'static>),
                func
            }
        })
    }
}

impl<X, Y, F> GraphNode<> for TransformGraphNode<X, Y, F>
where
    X: Clone + Default + Send + Sync + 'static,
    Y: Clone + Default + Send + Sync + 'static,
    F: Fn(X) -> Y + Clone + Send + Sync + 'static
{
    fn process_backwards(&self) -> Action {
        let input_pin1 = self.input_pin.clone();
        let input_pin2 = self.input_pin.clone();
        let output_pin = self.output_pin.clone();
        let func = self.func.clone();

        let receive_data = move || {
            let mut pin_write = input_pin2.write().unwrap();
            pin_write.receive_data();
            pin_write.pin.get_data().clone()
        };

        defer_sequential(vec![
            defer_action(move || {
                input_pin1.read().unwrap().propagate_backwards()
            }),
            defer_action(move || {
                let result = func(receive_data());
                output_pin.write().unwrap().pin.set_data(result);
                None
            })
        ])
    }

    fn process_forwards(&self) -> Action {
        let input_pin = self.input_pin.clone();
        let output_pin1 = self.output_pin.clone();
        let output_pin2 = self.output_pin.clone();
        let func = self.func.clone();

        let send_data = move |data| {
            let mut pin_write = output_pin1.write().unwrap();
            pin_write.pin.set_data(data);
            pin_write.send_data();
        };

        defer_action(move || {
            let x = input_pin.read().unwrap().pin.get_data().clone();
            send_data((func)(x));
            output_pin2.read().unwrap().propagate_forwards()
        })
    }

    fn get_inputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        Some(vec![self.input_pin.clone()])
    }

    fn get_outputs(&self) -> Option<Vec<Arc<RwLock<dyn GraphNodePinBase>>>> {
        Some(vec![self.output_pin.clone()])
    }
}

pub struct NodeGraph {
    nodes: Vec<Arc<dyn GraphNode + Send + Sync>>
}

impl NodeGraph {
    pub fn new(nodes: Vec<Arc<dyn GraphNode + Send + Sync>>) -> Self {
        Self {
            nodes
        }
    }

    pub fn get_matching_nodes<F>(&self, f: F) -> Vec<Arc<dyn GraphNode + Send + Sync>> 
        where F: Fn(&&Arc<dyn GraphNode + Send + Sync>) -> bool + 'static {
           self.nodes.iter().filter(f).cloned().collect()
    }

    pub fn get_source_nodes(&self) -> Vec<Arc<dyn GraphNode + Send + Sync>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_none() && node.get_outputs().is_some())
    }
    
    pub fn get_target_nodes(&self) -> Vec<Arc<dyn GraphNode + Send + Sync>> {
        self.get_matching_nodes(|node|
            node.get_inputs().is_some() && node.get_outputs().is_none())
    }
    
    pub fn get_transform_nodes(&self) -> Vec<Arc<dyn GraphNode + Send + Sync>> {
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
fn connect_output_and_input_nodes(output_node: &Arc<dyn GraphNode + Send + Sync>, input_node: &Arc<dyn GraphNode + Send + Sync>)
 -> Result<(), Box<dyn Error + 'static>>
{
    let output_pins= output_node.get_outputs().ok_or("Cannot get output output")?;
    let input_pins= input_node.get_inputs().ok_or("Cannot get input input")?;

    assert_eq!(output_pins.len(), 1);
    assert_eq!(input_pins.len(), 1);

    input_pins[0].write().unwrap().try_connect(output_pins[0].clone())?;

    Ok(())
}

#[cfg(test)]
fn find_matching_transform_node_by_output_data_type<T>(graph: &NodeGraph) -> Option<Arc<dyn GraphNode + Send + Sync>>
where T: Clone + Default + Send + Sync + 'static {

    graph.get_transform_nodes().iter().find(|node| 
            node.get_outputs().and_then(|pins| 
                pins.iter().find(|pin| 
                    (*pin).clone().read().unwrap().downcast_ref::<GraphNodeOutputPin<T>>().is_some()
                ).map(|_| true)
            ).is_some()
        ).cloned()
}

#[cfg(test)]
fn find_matching_target_node_by_input_data_type<T>(graph: &NodeGraph) -> Option<Arc<dyn GraphNode + Send + Sync>>
where T: Clone + Default + Send + Sync + 'static {

    graph.get_target_nodes().iter().find(|node| 
        {
            ((*node).clone() as Arc<dyn GraphNode>).downcast_ref::<TargetGraphNode<T>>().is_some()
        }
    ).cloned()
}

#[cfg(test)]
fn connect_some_nodes(graph: &NodeGraph)
 -> Result<(Arc<dyn GraphNode + Send + Sync>, Arc<dyn GraphNode + Send + Sync>, Arc<dyn GraphNode + Send + Sync>),
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
fn load_example_value(source_node: Arc<dyn GraphNode>, value: ExampleInput)
{
    if let Some(node) = source_node.downcast_ref::<SourceGraphNode<ExampleInput>>() {
        node.load_value(value);
    }
}

#[cfg(test)]
fn get_example_result<T>(target_node: Arc<dyn GraphNode>) -> Option<T>
where
    T: Default + Clone + Send + Sync + 'static
{
    match target_node.downcast_ref::<TargetGraphNode<T>>() {
        Some(node) => Some(node.get_value()),
        _ => None
    }
}

#[cfg(test)]
fn run_graph(policy: &impl ExecutionPolicy) {

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node_u, target_node_v) = connect_result.unwrap();

    load_example_value(source_node.clone(), ExampleInput{x: 10, y:20});
    run_actions(policy, source_node.process_forwards());

    let value1u: ExampleResultU = get_example_result(target_node_u.clone()).unwrap();
    let value1v: ExampleResultV = get_example_result(target_node_v.clone()).unwrap();

    println!("Result of process forwards: {}, {}", value1u.u, value1v.v);

    let graph = build_some_graph();

    let connect_result = connect_some_nodes(&graph);
    assert!(connect_result.is_ok());

    let (source_node, target_node_u, target_node_v) = connect_result.unwrap();

    load_example_value(source_node.clone(), ExampleInput{x:20, y:80});
    let target_node_u1 = target_node_u.clone();
    let target_node_v1 = target_node_v.clone();
    run_actions(policy, defer_parallel(vec![
        defer_action(move || target_node_u1.process_backwards()),
        defer_action(move || target_node_v1.process_backwards())
    ]));

    let value2u: ExampleResultU = get_example_result(target_node_u.clone()).unwrap();
    let value2v: ExampleResultV = get_example_result(target_node_v.clone()).unwrap();

    println!("Result of process backwards: {}, {}", value2u.u, value2v.v);

}



#[test]
fn test_me() {
    let basic_policy = BasicPolicy{};
    let rayon_policy = RayonPolicy{};

    println!("\n * * * Running with Sequencial Execution * * * \n");
    run_graph(&basic_policy);

    println!("\n * * * Running with Parallel Execution * * * \n");
    run_graph(&rayon_policy);
}
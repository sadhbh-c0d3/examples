#include<memory>
#include<set>
#include<iostream>
#include<algorithm>
#include<ranges>
#include<assert.h>

#include "rwlock.hpp"
#include "weak_ptr_compare.hpp"
#include "helpers.hpp"

// local namespace
namespace ns8 {

struct IGraphNode;

using Action = std::function<void ()>;

template<class T> using Arc = std::shared_ptr<T>;
template<class T> using Weak = std::weak_ptr<T>;
template<class T> using EnableArcFromThis = std::enable_shared_from_this<T>;

struct IGraphPin
{
    virtual IGraphNode &GetOwningNode() const = 0;

    virtual bool IsConnected() const = 0;
    virtual bool TryConnect(IRwLock<IGraphPin> &otherPin) = 0;
    [[nodiscard]] virtual Action Disconnect() = 0;

protected:
    IGraphPin() {}
    ~IGraphPin() {}

    IGraphPin(IGraphPin const &) = delete;
    IGraphPin(IGraphPin &&) = delete;

    IGraphPin &operator=(IGraphPin const &) = delete;
    IGraphPin &operator=(IGraphPin &&) = delete;
};

using IGraphPinPtr = Arc<IRwLock<IGraphPin>>;

/// @brief General node interface
struct IGraphNode : EnableArcFromThis<IGraphNode>
{
    virtual void ProcessForwards() const = 0;
    virtual void ProcessBackwards() const = 0;

    virtual bool HasInputs() const = 0;
    virtual bool HasOutputs() const = 0;

    virtual std::set<IGraphPinPtr> GetInputPins() const = 0;
    virtual std::set<IGraphPinPtr> GetOutputPins() const = 0;

protected:
    IGraphNode() {}
    ~IGraphNode() {}

    IGraphNode(IGraphNode const &) = delete;
    IGraphNode(IGraphNode &&) = delete;

    IGraphNode &operator= (IGraphNode const &) = delete;
    IGraphNode &operator= (IGraphNode &&) = delete;
};

using IGraphNodePtr = Arc<IGraphNode>;

/// @brief Universal pin
/// @tparam T Type of data stored in this pin
template<class T> class GraphPin : public IGraphPin
{
public:
    using base_interface = IGraphPin;

    /// @brief Obtain owning node
    IGraphNode &GetOwningNode() const override { return m_node; }

    /// @brief Set data stored in this pin
    template<class X> void SetData(X &&x) { m_data = std::forward<X>(x); }

    /// @brief Get data stored in this pin
    T const & GetData() const { return m_data; }

private:
    // Node owns this pin
    IGraphNode &m_node;
    T m_data;

protected:
    /// @brief Create pin attached to specific node
    GraphPin(IGraphNode &node): m_node(node), m_data{} {}
};

template<class T> class GraphInputPin;
template<class T> class GraphOutputPin;

template<class T> using GraphInputPinPtr = Arc<RwLock<GraphInputPin<T>>>;
template<class T> using GraphOutputPinPtr = Arc<RwLock<GraphOutputPin<T>>>;

/// @brief Input pin
/// @tparam T Type of data input
template<class T> class GraphInputPin : public GraphPin<T>, public EnableRwLockFromThis<GraphInputPin<T>>
{
public:
    GraphInputPin(IGraphNode &node): GraphPin<T>(node) {}

    /// @brief Connect output pin (implemented below to solve catch22)
    void Connect(GraphOutputPin<T> &pin);
    
    bool TryConnect(IRwLock<IGraphPin> &otherPin) override;

    /// @brief Receive data from the other connected pin into this pin
    void ReceiveData()
    {
        if (auto connectedPin = GetConnectedPin(); connectedPin)
        {
            GraphPin<T>::SetData(connectedPin->read()->GetData());
        }
    }
    
    /// @brief Propagate processing to connected node
    void PropagateBackwards() const
    {
        if (auto connectedPin = GetConnectedPin(); connectedPin)
        {
            IGraphNode &node = connectedPin->read()->GetOwningNode();
            node.ProcessBackwards();
        }
    }
    
    /// @brief Obtain connected pin
    GraphOutputPinPtr<T> GetConnectedPin() const { return m_connection.lock(); }

    /// @brief Tell if pin is connected
    /// @return 
    bool IsConnected() const override
    {
        return !m_connection.expired();
    }

    /// @brief Disconnect other pin
    Action Disconnect() override;
    
    Action OutputDisconnecting();

    GraphInputPinPtr<T> shared_from_this() const
    {
        return GraphInputPinPtr<T>(
            GraphPin<T>::GetOwningNode().shared_from_this(), 
            EnableRwLockFromThis<GraphInputPin<T>>::lock_from_this());
    }

private:
    // Must be weak, as it is the node that owns the pins
    Weak<RwLock<GraphOutputPin<T>>> m_connection;
};

/// @brief Output pin
/// @tparam T Type of data output
template<class T> class GraphOutputPin : public GraphPin<T> , public EnableRwLockFromThis<GraphOutputPin<T>>
{
public:
    GraphOutputPin(IGraphNode &node): GraphPin<T>(node) {}

    /// @brief Connect output pin
    void Connect(GraphInputPin<T> &pin)
    {
        DBG("Connect: OutputPin ==> InputPin");

        // Check if not already connected to that pin to avoid infinite loop
        if (auto const [pos, added] = m_connections.insert(pin.shared_from_this()); added)
        {
            pin.Connect(*this);
        }
    }
    
    bool TryConnect(IRwLock<IGraphPin> &otherPin) override
    {
        DBG("TryConnect: OutputPin ==> InputPin");

        if (auto inputPin = otherPin.try_write<GraphInputPin<T>>(); inputPin.has_value())
        {
            Connect(*inputPin.value());
            return true;
        }
        return false;
    }

    /// @brief Send data from this pin to the other connected pin
    void SendData() const
    {
        for (auto &connectedPin : GetConnectedPins())
        {
            connectedPin->write()->SetData(GraphPin<T>::GetData());
        }
    }

    /// @brief Propagate processing to connected node
    void PropagateForwards() const
    {
        for (auto &connectedPin : GetConnectedPins())
        {
            IGraphNode &node = connectedPin->read()->GetOwningNode();
            node.ProcessForwards();
        }
    }
    
    /// @brief Obtain connected pin
    std::set<GraphInputPinPtr<T>> GetConnectedPins() const
    {
        std::set<GraphInputPinPtr<T>> pins;
        
        for (auto &pin : m_connections)
        {
            if (!pin.expired()) { pins.insert(pin.lock()); }
        }

        return std::move(pins);
    }

    /// @brief Tell if pin is connected
    /// @return 
    bool IsConnected() const override
    {
        for (auto const &pin : m_connections)
        {
            if (!pin.expired()) { return true; }
        }
        return false;
    }

    void DisconnectPin(GraphInputPin<T> &pin)
    {
        if (auto pos = m_connections.find(pin.shared_from_this()); pos != m_connections.end())
        {
            m_connections.erase(pos);
            pin.Disconnect();
        }
    }

    /// @brief Disconnect other pin
    Action Disconnect() override
    {
        auto connections = GetConnectedPins();
        m_connections.clear();

        std::vector<Action> actions;
        std::ranges::transform(connections, std::back_inserter(actions), [](auto &pin) {
            return pin->write()->OutputDisconnecting();
        });

        return [actions]() {
            std::ranges::for_each(actions, [](auto &action) { action(); });
        };
    }
    
    GraphOutputPinPtr<T> shared_from_this() const
    {
        return GraphOutputPinPtr<T>(
            GraphPin<T>::GetOwningNode().shared_from_this(),
            EnableRwLockFromThis<GraphOutputPin<T>>::lock_from_this());
    }

private:
    // Must be weak, as it is the node that owns the pins
    std::set<Weak<RwLock<GraphInputPin<T>>>, weak_ptr_compare<RwLock<GraphInputPin<T>>>> m_connections;
};

template<class T> Action GraphInputPin<T>::Disconnect()
{
    // Since we're disconnecting input, we need to drop old data
    GraphPin<T>::SetData(T{});

    if (!m_connection.expired())
    {
        // Reset to prevent infinite loop
        auto connection = m_connection.lock();
        m_connection.reset();

        // We need to tell source to disconnect us
        connection->write()->DisconnectPin(*this);
    }
    else {
        // Source died
        m_connection.reset();
    }
    
    // The lock acquired to read owning node is now gone, and we can go ahead and tell node to propagate
    return [node = GraphPin<T>::GetOwningNode().shared_from_this()]() {
        node->ProcessForwards();
    };
}

template<class T> Action GraphInputPin<T>::OutputDisconnecting()
{
    // Since we're disconnecting input, we need to drop old data
    GraphPin<T>::SetData(T{});

    m_connection.reset();

    // The lock acquired to read owning node is now gone, and we can go ahead and tell node to propagate
    return [node = GraphPin<T>::GetOwningNode().shared_from_this()]() {
        node->ProcessForwards();
    };
}

// Implementation of InputPin::Connect
template<class T> void GraphInputPin<T>::Connect(GraphOutputPin<T> &pin)
{
    DBG("Connect: InputPin ==> OutputPin");

    // Check if not already connected to that pin to avoid infinite loop
    if (m_connection.expired() || m_connection.lock().get() != pin.shared_from_this().get())
    {
        m_connection = pin.shared_from_this();
        pin.Connect(*this);
    }
}

template<class T> bool GraphInputPin<T>::TryConnect(IRwLock<IGraphPin> &otherPin)
{
    DBG("TryConnect: InputPin ==> OutputPin");

    if (auto inputPin = otherPin.try_write<GraphOutputPin<T>>(); inputPin.has_value())
    {
        Connect(*inputPin.value());
        return true;
    }
    return false;
}

/// @brief IGraphNode that will provide data
/// @tparam T Type of source data
template<class T> class SourceNode : public IGraphNode, public IValueLoader<T>
{
public:
    ~SourceNode() { DBG("~SourceNode()"); }
    SourceNode() : m_outputPin{*this}
    {}

    void ProcessForwards() const override
    {
        DBG("Source.ProcessForwards");

        m_outputPin.read()->SendData();
        m_outputPin.read()->PropagateForwards();
    }
    
    void ProcessBackwards() const override
    {
        // This could be loading new value from somewhere
    }
    
    bool HasInputs() const override { return false; }
    bool HasOutputs() const override { return true; }

    std::set<IGraphPinPtr> GetInputPins() const override { return {}; }
    std::set<IGraphPinPtr> GetOutputPins() const override { return {m_outputPin.read()->shared_from_this() }; }

    void LoadValue(T &&value) override
    {
        DBG("Source.LoadValue");
        m_outputPin.write()->SetData(std::move(value));
    }

private:
    RwLock<GraphOutputPin<T>> m_outputPin;
};

/// @brief IGraphNode that will receive final result of all procesing
/// @tparam T Type of final result data
template<class T> class TargetNode : public IGraphNode, public IValueHolder<T>
{
public:
    ~TargetNode() { DBG("~TargetNode()"); }
    TargetNode() : m_inputPin{*this}
    {}

    void ProcessForwards() const override
    {
        // This could render result somewhere
    }

    void ProcessBackwards() const override
    {
        DBG("Target.ProcessBackwards");

        m_inputPin.read()->PropagateBackwards();
        m_inputPin.write()->ReceiveData();
    }
    
    bool HasInputs() const override { return true; }
    bool HasOutputs() const override { return false; }

    std::set<IGraphPinPtr> GetInputPins() const override { return { m_inputPin.read()->shared_from_this() }; }
    std::set<IGraphPinPtr> GetOutputPins() const override { return {}; }

    T const &GetValue() const override
    {
        return m_inputPin.read()->GetData();
    }

private:
    RwLock<GraphInputPin<T>> m_inputPin;
};

template<class X, class Y, class F> class TransformNode : public IGraphNode
{
public:
    ~TransformNode() { DBG("~TransformNode()"); }
    TransformNode(F f): m_function(std::move(f)), m_inputPin{*this}, m_outputPin{*this} {}
    
    void ProcessBackwards() const override
    {
        DBG("Transform.ProcessBackwards");
        // If we're propagating backwards, then request came from output pin
        // This can happen when we're pulling from output
        auto data = ReceiveData();
        auto result = m_function(std::move(data));
        m_outputPin.write()->SetData(std::move(result));
    }

    void ProcessForwards() const override
    {
        DBG("Transform.ProcessForwards");
        // If we're propagate forwards, then request came from input pin
        // This can happen when we're pushing new input data
        auto data = m_inputPin.read()->GetData();
        auto result = m_function(std::move(data));
        SendData(result);
        m_outputPin.read()->PropagateForwards();
    }
    
    bool HasInputs() const override { return true; }
    bool HasOutputs() const override { return true; }

    std::set<IGraphPinPtr> GetInputPins() const override { return {m_inputPin.read()->shared_from_this()}; }
    std::set<IGraphPinPtr> GetOutputPins() const override { return {m_outputPin.read()->shared_from_this()}; }

private:
    F m_function;
    RwLock<GraphInputPin<X>> m_inputPin;
    RwLock<GraphOutputPin<Y>> m_outputPin;

    X ReceiveData() const
    {
        auto pin_write = m_inputPin.write();
        pin_write->ReceiveData();
        return pin_write->GetData();
    }

    void SendData(Y result) const
    {
        auto pin_write = m_outputPin.write();
        pin_write->SetData(std::move(result));
        pin_write->SendData();
    }
};

class NodeGraph
{
public:
    ~NodeGraph() { DBG("~NodeGraph()"); }

    template<class NodeT>
    NodeGraph &AddNode(Arc<NodeT> const &nodePtr)
    {
        m_nodes.emplace(std::move(nodePtr));
        return *this;
    }

    std::set<IGraphNodePtr> GetSourceNodes() const
    {
        auto predicate = [](auto &nodePtr) { return nodePtr->HasOutputs() and not nodePtr->HasInputs(); };
        return GetMatchingNodes(std::move(predicate));
    }

    std::set<IGraphNodePtr> GetTargetNodes() const
    {
        auto predicate = [](auto &nodePtr) { return nodePtr->HasInputs() and not nodePtr->HasOutputs(); };
        return GetMatchingNodes(std::move(predicate));
    }

    std::set<IGraphNodePtr> GetTransformNodes() const
    {
        auto predicate = [](auto &nodePtr) { return nodePtr->HasInputs() and nodePtr->HasOutputs(); };
        return GetMatchingNodes(std::move(predicate));
    }

    template<class NodeType>
        std::set<Arc<NodeType>> GetNodesOfType() const
        {
            auto projection = [] (auto &nodePtr) { return std::dynamic_pointer_cast<NodeType>(nodePtr); };
            std::set<Arc<NodeType>> nodes{};
            std::ranges::transform(m_nodes, std::inserter(nodes, nodes.begin()), projection);
            nodes.erase(Arc<NodeType>{});
            return std::move(nodes);
        }

    template<class Predicate>
        std::set<IGraphNodePtr> GetMatchingNodes(Predicate &&predicate) const
        {
            std::set<IGraphNodePtr> nodes{};
            std::ranges::copy_if(m_nodes, std::inserter(nodes, nodes.begin()), predicate);
            return std::move(nodes);
        }

private:
    std::set<IGraphNodePtr> m_nodes;
};

struct ExampleDataSample
{
    ExampleDataSample(): x{}, y{} {}
    ExampleDataSample(int ix, int iy): x{ix}, y{iy} {}

    int x;
    int y;
};

struct ExampleDataResultU
{
    ExampleDataResultU(): u{} {}
    ExampleDataResultU(double iu): u{iu} {}

    double u;
};

struct ExampleDataResultV
{
    ExampleDataResultV(): v{} {}
    ExampleDataResultV(double iv): v{iv} {}

    double v;
};

using ExampleDataSamplePtr = Arc<ExampleDataSample>;
using ExampleDataResultUPtr = Arc<ExampleDataResultU>;
using ExampleDataResultVPtr = Arc<ExampleDataResultV>;

std::ostream &operator <<(std::ostream &os, ExampleDataSamplePtr const &data)
{
    if (data) {
        return os << "[ x: " << data->x << ", y: " << data->y << "]";
    }
    else {
        return os << "[ <no value> ]";
    }
}

std::ostream &operator <<(std::ostream &os, ExampleDataResultUPtr const &data)
{
    if (data) {
        return os << "[ u: " << data->u << "]";
    }
    else {
        return os << "[ <no value> ]";
    }
}

std::ostream &operator <<(std::ostream &os, ExampleDataResultVPtr const &data)
{
    if (data) {
        return os << "[ v: " << data->v << "]";
    }
    else {
        return os << "[ <no value> ]";
    }
}

void add_example_nodes_to_graph(NodeGraph &graph)
{
    // Func lives only within current scope.
    // Note that it gets consumed by TrasformNode constructor.
    auto funcU = [](ExampleDataSamplePtr const &data)
    {
        if (data) {
            return std::make_shared<ExampleDataResultU>(
                static_cast<double>(data->x * data->y));
        }
        else {
            return ExampleDataResultUPtr{};
        }
    };

    auto funcV = [](ExampleDataSamplePtr const &data)
    {
        if (data) {
            return std::make_shared<ExampleDataResultV>(
                static_cast<double>(data->x) / static_cast<double>(data->y));
        }
        else {
            return ExampleDataResultVPtr{};
        }
    };

    // Let's create node using our transformation function
    auto transformUNodePtr = std::make_shared<TransformNode<
        ExampleDataSamplePtr, ExampleDataResultUPtr,
        decltype(funcU)>>(funcU);

    auto transformVNodePtr = std::make_shared<TransformNode<
        ExampleDataSamplePtr, ExampleDataResultVPtr,
        decltype(funcV)>>(funcV);

    // Now we need some source
    auto sourceNodePtr = std::make_shared<SourceNode<ExampleDataSamplePtr>>();

    // And we also need some target
    auto targetUNodePtr = std::make_shared<TargetNode<ExampleDataResultUPtr>>();
    
    auto targetVNodePtr = std::make_shared<TargetNode<ExampleDataResultVPtr>>();

    // Move ownership of the nodes into node graph
    graph
        .AddNode(std::move(sourceNodePtr))
        .AddNode(std::move(transformUNodePtr))
        .AddNode(std::move(transformVNodePtr))
        .AddNode(std::move(targetUNodePtr))
        .AddNode(std::move(targetVNodePtr));

    // WARNING !!! At this point shared node pointers are no longer valid
}

} // end of local namespace

using namespace ns8;

void test_s8_dynamic_graph_and_lock()
{
    std::cout << "TEST: test_s8_dynamic_graph_and_rwlock" << std::endl;

    NodeGraph graph{};

    add_example_nodes_to_graph(graph);

    // We need to find the source node
    auto sourceNodes = graph.GetSourceNodes();
    assert(sourceNodes.size() == 1);

    // and we need to be able to load value into source node
    auto sourceNode = *std::begin(sourceNodes);
    auto sourceLoader = std::dynamic_pointer_cast<IValueLoader<ExampleDataSamplePtr>>(sourceNode);
    assert(nullptr != sourceLoader);

    // and then we need to get output pin from the source node
    auto sourceOutputPins = sourceNode->GetOutputPins();
    assert(sourceOutputPins.size() == 1);

    // and from that pin we need to be able to get current value in it
    auto sourcePin = *std::begin(sourceOutputPins);

    auto sourcePinGetData = [sourcePin] {
        auto sourceOutputPin = sourcePin->try_read<GraphOutputPin<ExampleDataSamplePtr>>();
        assert(sourceOutputPin.has_value());
        return sourceOutputPin.value()->GetData();
    };

    // We need to find target nodes
    auto targetUNodes = graph.GetNodesOfType<TargetNode<ExampleDataResultUPtr>>();
    auto targetVNodes = graph.GetNodesOfType<TargetNode<ExampleDataResultVPtr>>();
    assert(targetUNodes.size() == 1);
    assert(targetVNodes.size() == 1);

    // and then we need to be able to get current result from them
    auto targetUNode = *std::begin(targetUNodes);
    auto targetVNode = *std::begin(targetVNodes);
    auto targetUHolder = std::dynamic_pointer_cast<IValueHolder<ExampleDataResultUPtr>>(targetUNode);
    auto targetVHolder = std::dynamic_pointer_cast<IValueHolder<ExampleDataResultVPtr>>(targetVNode);
    assert(targetUHolder);
    assert(targetVHolder);
    
    // We need input pin from each target nodes
    auto targetUInputPins = targetUNode->GetInputPins();
    auto targetVInputPins = targetVNode->GetInputPins();
    assert(targetUInputPins.size() == 1);
    assert(targetVInputPins.size() == 1);

    auto targetUPin = *std::begin(targetUInputPins);
    auto targetVPin = *std::begin(targetVInputPins);

    // We need to find transform node first
    auto transformUNodes = graph.GetMatchingNodes([](auto const &ptr) {
        if (ptr->HasInputs()) {
            auto pins = ptr->GetOutputPins();
            return pins.end() != std::ranges::find_if(pins, [](Arc<IRwLock<IGraphPin>> const &pin) {
                return pin->try_read<GraphOutputPin<ExampleDataResultUPtr>>().has_value(); });
        }
        return false;
    });
    auto transformVNodes = graph.GetMatchingNodes([](auto const &ptr) {
        if (ptr->HasInputs()) {
            auto pins = ptr->GetOutputPins();
            return pins.end() != std::ranges::find_if(pins, [](Arc<IRwLock<IGraphPin>> const &pin) {
                return pin->try_read<GraphOutputPin<ExampleDataResultVPtr>>().has_value(); });
        }
        return false;
    });
    assert(transformUNodes.size() == 1);
    assert(transformVNodes.size() == 1);

    auto transformUNode = *std::begin(transformUNodes);
    auto transformVNode = *std::begin(transformVNodes);
    
    // We need input and output pins of transform node
    auto transformUInputPins = transformUNode->GetInputPins();
    auto transformVInputPins = transformVNode->GetInputPins();
    auto transformUOutputPins = transformUNode->GetOutputPins();
    auto transformVOutputPins = transformVNode->GetOutputPins();
    assert(transformUInputPins.size() == 1);
    assert(transformVInputPins.size() == 1);
    assert(transformUOutputPins.size() == 1);
    assert(transformVOutputPins.size() == 1);

    auto transformUInputPin = *std::begin(transformUInputPins);
    auto transformVInputPin = *std::begin(transformVInputPins);
    auto transformUOutputPin = *std::begin(transformUOutputPins);
    auto transformVOutputPin = *std::begin(transformVOutputPins);
    
    // We connect input and output of transform node
    auto connectUInputResult = transformUInputPin->write()->TryConnect(*sourcePin);
    auto connectVInputResult = transformVInputPin->write()->TryConnect(*sourcePin);
    auto connectUOutputResult = transformUOutputPin->write()->TryConnect(*targetUPin);
    auto connectVOutputResult = transformVOutputPin->write()->TryConnect(*targetVPin);

    assert(connectUInputResult);
    assert(connectVInputResult);
    assert(connectUOutputResult);
    assert(connectVOutputResult);

    std::cout << "All set" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(7, 9));
    sourceNode->ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(11, 5));
    targetUNode->ProcessBackwards();
    targetVNode->ProcessBackwards();

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
    // Let's disconnect one of the transform nodes
    auto action1 = transformVInputPin->write()->Disconnect();
    action1();

    std::cout << "After disconnecting V transform" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(5, 6));
    sourceNode->ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(3, 2));
    targetUNode->ProcessBackwards();
    targetVNode->ProcessBackwards();

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
    // Let's disconnect source
    auto action2 = sourcePin->write()->Disconnect();
    action2();
    
    std::cout << "After disconnecting source" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(1, 2));
    sourceNode->ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(9, 8));
    targetUNode->ProcessBackwards();
    targetVNode->ProcessBackwards();

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
}
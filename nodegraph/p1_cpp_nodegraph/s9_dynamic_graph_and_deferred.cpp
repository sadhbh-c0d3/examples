#include<memory>
#include<set>
#include<iostream>
#include<algorithm>
#include<ranges>
#include<assert.h>

#include "action.hpp"
#include "rwlock.hpp"
#include "weak_ptr_compare.hpp"
#include "helpers.hpp"

// local namespace
namespace ns9 {

struct IGraphNode;

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
    [[nodiscard]] virtual Action ProcessForwards() const = 0;
    [[nodiscard]] virtual Action ProcessBackwards() const = 0;

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
    [[nodiscard]] Action PropagateBackwards() const
    {
        if (auto connectedPin = GetConnectedPin(); connectedPin)
        {
            return connectedPin->unguarded_ptr()->GetOwningNode().ProcessBackwards();
        }

        return {};
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
    [[nodiscard]] Action Disconnect() override;
    
    [[nodiscard]] Action OutputDisconnecting();

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
        DBG(DBG_ID(this) << "Connect: OutputPin ==> InputPin");

        // Check if not already connected to that pin to avoid infinite loop
        if (auto const [pos, added] = m_connections.insert(pin.shared_from_this()); added)
        {
            pin.Connect(*this);
        }
    }
    
    bool TryConnect(IRwLock<IGraphPin> &otherPin) override
    {
        DBG(DBG_ID(this) << "TryConnect: OutputPin ==> InputPin");

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
    [[nodiscard]] Action PropagateForwards() const
    {
        auto actions = trasform_into_deferred_actions(GetConnectedPins(), [](auto &&connectedPin) {
            return deferred_action([connectedPin = std::move(connectedPin)] () {
                return connectedPin->unguarded_ptr()->GetOwningNode().ProcessForwards();
            });
        });
        return defer_parallel(actions);
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

    [[nodiscard]] Action DisconnectPin(GraphInputPin<T> &pin)
    {
        DBG(DBG_ID(this) << "OutputPin.DisconnectPin");

        if (auto pos = m_connections.find(pin.shared_from_this()); pos != m_connections.end())
        {
            m_connections.erase(pos);
            return pin.Disconnect();
        }

        return deferred_end();
    }

    /// @brief Disconnect other pin
    [[nodiscard]] Action Disconnect() override
    {
        DBG(DBG_ID(this) << "OutputPin.Disconnect");

        auto connections = GetConnectedPins();
        m_connections.clear();

        auto actions = trasform_into_deferred_actions(connections, [](auto &pin) {
            return pin->write()->OutputDisconnecting();
        });

        DBG(DBG_ID(this) << "OutputPin.Disconnect: defer actions...");
        return defer_parallel(actions);
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
    DBG(DBG_ID(this) << "InputPin.Disconnect");
    // Since we're disconnecting input, we need to drop old data
    GraphPin<T>::SetData(T{});

    if (!m_connection.expired())
    {
        DBG(DBG_ID(this) << "InputPin.Disconnect: Disconnecting...");

        // Reset to prevent infinite loop
        auto connection = m_connection.lock();
        m_connection.reset();

        // We need to tell source to disconnect us
        return connection->write()->DisconnectPin(*this);
    }

    // Source died
    m_connection.reset();

    DBG(DBG_ID(this) << "InputPin.Disconnect: Disconnected...");
    return deferred_action([_self = shared_from_this(), this]() {
            return GraphPin<T>::GetOwningNode().ProcessForwards();
        });
}

template<class T> Action GraphInputPin<T>::OutputDisconnecting()
{
    DBG(DBG_ID(this) << "InputPin.OutputDisconnecting");
    // Since we're disconnecting input, we need to drop old data
    GraphPin<T>::SetData(T{});

    m_connection.reset();

    DBG(DBG_ID(this) << "InputPin.OutputDisconnecting: ProcessForwards...");
    return deferred_action([_self = shared_from_this(), this]() {
            return GraphPin<T>::GetOwningNode().ProcessForwards();
        });
}

// Implementation of InputPin::Connect
template<class T> void GraphInputPin<T>::Connect(GraphOutputPin<T> &pin)
{
    DBG(DBG_ID(this) << "Connect: InputPin ==> OutputPin");

    // Check if not already connected to that pin to avoid infinite loop
    if (m_connection.expired() || m_connection.lock().get() != pin.shared_from_this().get())
    {
        m_connection = pin.shared_from_this();
        pin.Connect(*this);
    }
}

template<class T> bool GraphInputPin<T>::TryConnect(IRwLock<IGraphPin> &otherPin)
{
    DBG(DBG_ID(this) << "TryConnect: InputPin ==> OutputPin");

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
    ~SourceNode() { DBG(DBG_ID(this) << "~SourceNode()"); }
    SourceNode() : m_outputPin{*this}
    {}

    [[nodiscard]] Action ProcessForwards() const override
    {
        DBG(DBG_ID(this) << "Source.ProcessForwards");

        return deferred_action([_self = shared_from_this(), this]() {
            auto guard = m_outputPin.read();
            guard->SendData();
            return guard->PropagateForwards();
        });
    }
    
    [[nodiscard]] Action ProcessBackwards() const override
    {
        DBG(DBG_ID(this) << "Source.ProcessBackwards");

        // This could be loading new value from somewhere
        return deferred_end();
    }
    
    bool HasInputs() const override { return false; }
    bool HasOutputs() const override { return true; }

    std::set<IGraphPinPtr> GetInputPins() const override { return {}; }
    std::set<IGraphPinPtr> GetOutputPins() const override { return {m_outputPin.unguarded_ptr()->shared_from_this() }; }

    void LoadValue(T &&value) override
    {
        DBG(DBG_ID(this) << "Source.LoadValue");
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
    ~TargetNode() { DBG(DBG_ID(this) << "~TargetNode()"); }
    TargetNode() : m_inputPin{*this}
    {}

    [[nodiscard]] Action ProcessForwards() const override
    {
        DBG(DBG_ID(this) << "Target.ProcessForwards");

        // This could render result somewhere
        return {};
    }

    [[nodiscard]] Action ProcessBackwards() const override
    {
        DBG(DBG_ID(this) << "Target.ProcessBackwards");

        return defer_sequential(
            deferred_action([_self = shared_from_this(), this]() {
                return m_inputPin.read()->PropagateBackwards();
            }),
            deferred_action([_self = shared_from_this(), this]() {
                m_inputPin.write()->ReceiveData();
                return deferred_end();
            })
        );
    }
    
    bool HasInputs() const override { return true; }
    bool HasOutputs() const override { return false; }

    std::set<IGraphPinPtr> GetInputPins() const override { return { m_inputPin.unguarded_ptr()->shared_from_this() }; }
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
    ~TransformNode() { DBG(DBG_ID(this) << "~TransformNode()"); }
    TransformNode(F f): m_function(std::move(f)), m_inputPin{*this}, m_outputPin{*this} {}
    
    [[nodiscard]] Action ProcessBackwards() const override
    {
        DBG(DBG_ID(this) << "Transform.ProcessBackwards");

        return defer_sequential(
            deferred_action([_self = shared_from_this(), this] () {
                DBG(DBG_ID(this) << "Transform.ProcessBackwards: Propagate...");
                return m_inputPin.read()->PropagateBackwards();
            }),
            deferred_action([_self = shared_from_this(), this] () {
                DBG(DBG_ID(this) << "Transform.ProcessBackwards: ReceiveData");
                auto result = m_function(ReceiveData());
                DBG(DBG_ID(this) << "Transform.ProcessBackwards: SetData");
                m_outputPin.write()->SetData(std::move(result));
                return deferred_end();
            })
        );
    }

    [[nodiscard]] Action ProcessForwards() const override
    {
        DBG(DBG_ID(this) << "Transform.ProcessForwards");

        return deferred_action([_self = shared_from_this(), this] () {
            DBG(DBG_ID(this) << "Transform.ProcessForwards: GetData");
            auto data = m_inputPin.read()->GetData();
            DBG(DBG_ID(this) << "Transform.ProcessForwards: SendData");
            SendData(m_function(std::move(data)));
            DBG(DBG_ID(this) << "Transform.ProcessForwards: Propagate...");
            return m_outputPin.read()->PropagateForwards();
        });
    }
    
    bool HasInputs() const override { return true; }
    bool HasOutputs() const override { return true; }

    std::set<IGraphPinPtr> GetInputPins() const override { return {m_inputPin.unguarded_ptr()->shared_from_this()}; }
    std::set<IGraphPinPtr> GetOutputPins() const override { return {m_outputPin.unguarded_ptr()->shared_from_this()}; }

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
    ~NodeGraph() { DBG(DBG_ID(this) << "~NodeGraph()"); }

    template<class NodeT>
        NodeGraph &AddNode(Arc<NodeT> &&nodePtr)
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
            std::set<Arc<NodeType>> nodes{};
            std::ranges::transform(m_nodes, std::inserter(nodes, nodes.begin()),[] (auto &nodePtr) {
                return std::dynamic_pointer_cast<NodeType>(nodePtr); });
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

using namespace ns9;

template<class ExecutionPolicy> void s9_dynamic_graph_and_deferred(ExecutionPolicy &&policy)
{
    DBG("TEST: test_s9_dynamic_graph_and_deferred - " << policy);

    std::cout << "TEST: test_s9_dynamic_graph_and_deferred - " << policy << std::endl;

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

    DBG(DBG_ID(sourcePin->unguarded_ptr()) << "Source.Output");

    DBG(DBG_ID(transformUInputPin->unguarded_ptr()) << "Transform_u.Input");
    DBG(DBG_ID(transformUOutputPin->unguarded_ptr()) << "Transform_u.Output");

    DBG(DBG_ID(transformVInputPin->unguarded_ptr()) << "Transform_v.Input");
    DBG(DBG_ID(transformVOutputPin->unguarded_ptr()) << "Transform_v.Output");

    DBG(DBG_ID(targetUPin->unguarded_ptr()) << "Target_u.Input");
    DBG(DBG_ID(targetVPin->unguarded_ptr()) << "Target_v.Input");

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(7, 9));
    run_actions(policy, sourceNode->ProcessForwards());
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(11, 5));
    run_actions(policy, defer_parallel(
        deferred_action([targetUNode] { return targetUNode->ProcessBackwards(); }),
        deferred_action([targetVNode] { return targetVNode->ProcessBackwards(); })
    ));

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
    // Let's disconnect one of the transform nodes
    if (auto action = transformVInputPin->write()->Disconnect(); action.has_value()) {
        run_actions(policy, action);
    }

    std::cout << "After disconnecting V transform" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(5, 6));
    run_actions(policy, sourceNode->ProcessForwards());
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(3, 2));
    run_actions(policy, defer_parallel(
        deferred_action([targetUNode] { return targetUNode->ProcessBackwards(); }),
        deferred_action([targetVNode] { return targetVNode->ProcessBackwards(); })
    ));

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
    // Let's disconnect source
    if (auto action = sourcePin->write()->Disconnect(); action.has_value()) {
        run_actions(policy, action);
    }
    
    std::cout << "After disconnecting source" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(1, 2));
    run_actions(policy, sourceNode->ProcessForwards());
    
    std::cout << "Result of processing forwards: f(" << sourcePinGetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(9, 8));
    run_actions(policy, defer_parallel(
        deferred_action([targetUNode] { return targetUNode->ProcessBackwards(); }),
        deferred_action([targetVNode] { return targetVNode->ProcessBackwards(); })
    ));

    std::cout << "Result of processing backwards: f(" << sourcePinGetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
    
}

void test_s9_dynamic_graph_and_deferred()
{
    s9_dynamic_graph_and_deferred(StandardPolicy<false>{});

    s9_dynamic_graph_and_deferred(StandardPolicy<true>{});
}
#include<memory>
#include<set>
#include<iostream>
#include<assert.h>

#include "rwlock.hpp"

#define DBG(expr)
// #define DBG(expr) std::cout << "DBG> " << expr << std::endl

// local namespace
namespace {

/// @brief Compare two weak pointers by their pointer value
/// @remark Inspired by: https://stackoverflow.com/questions/32668742/a-set-of-weak-ptr
template<class T> struct weak_ptr_compare {
    bool operator() (const std::weak_ptr<T> &lhs, const std::weak_ptr<T> &rhs)const {
        auto lptr = lhs.lock(), rptr = rhs.lock();
        if (!rptr) return false; // nothing after expired pointer 
        if (!lptr) return true;  // every not expired after expired pointer
        return lptr.get() < rptr.get();
    }
};

/// @brief An interface capable of loading a value into somewhere
/// @tparam T 
template<class T> struct IValueLoader
{
    virtual void LoadValue(T &&value) = 0;

protected:
    // Only derived class can call this destructor
    ~IValueLoader() {}
};

/// @brief An interface capable of obtaining a value from somewhere
/// @tparam T 
template<class T> struct IValueHolder
{
    virtual T const &GetValue() const = 0;

protected:
    // Only derived class can call this destructor
    ~IValueHolder() {}
};

struct INode;

struct IPin
{
    virtual INode &GetOwningNode() const = 0;
    virtual bool IsConnected() const = 0;
    virtual bool TryConnect(IRwLock<IPin> &otherPin) = 0;
    virtual void Disconnect() = 0;

protected:
    // Only derived class can call this destructor
    ~IPin() {}
};

/// @brief General node interface
struct INode : std::enable_shared_from_this<INode>
{
    INode() {}
    INode(INode const &) = delete;
    INode(INode &&) = delete;

    INode &operator= (INode const &) = delete;
    INode &operator= (INode &&) = delete;

    virtual void ProcessForwards() const = 0;
    virtual void ProcessBackwards() const = 0;

    virtual bool HasInputs() const = 0;
    virtual bool HasOutputs() const = 0;

    virtual std::set<std::shared_ptr<IRwLock<IPin>>> GetInputPins() const = 0;
    virtual std::set<std::shared_ptr<IRwLock<IPin>>> GetOutputPins() const = 0;

protected:
    // Only derived class can call this destructor
    ~INode() {}
};

/// @brief Universal pin
/// @tparam T Type of data stored in this pin
template<class T> class Pin : public IPin
{
public:
    using base_interface = IPin;

    /// @brief Obtain owning node
    INode &GetOwningNode() const override { return m_node; }

    /// @brief Set data stored in this pin
    template<class X> void SetData(X &&x) { m_data = std::forward<X>(x); }

    /// @brief Get data stored in this pin
    T const & GetData() const { return m_data; }

private:
    // Node owns this pin
    INode &m_node;
    T m_data;

protected:
    /// @brief Create pin attached to specific node
    Pin(INode &node): m_node(node), m_data{} {}
};

// Forward declaration of OutputPin
template<class T> class OutputPin;

/// @brief Input pin
/// @tparam T Type of data input
template<class T> class InputPin : public Pin<T>
{
public:
    InputPin(INode &node, RwLock<InputPin<T>> &me): Pin<T>(node), m_me(me) {}

    /// @brief Connect output pin (implemented below to solve catch22)
    void Connect(OutputPin<T> &pin);
    
    bool TryConnect(IRwLock<IPin> &otherPin) override;

    /// @brief Receive data from the other connected pin into this pin
    void ReceiveData()
    {
        if (auto connectedPin = GetConnectedPin(); connectedPin)
        {
            Pin<T>::SetData(connectedPin->read()->GetData());
        }
    }
    
    /// @brief Propagate processing to connected node
    void PropagateBackwards() const
    {
        if (auto connectedPin = GetConnectedPin(); connectedPin)
        {
            INode &node = connectedPin->read()->GetOwningNode();
            node.ProcessBackwards();
        }
    }
    
    /// @brief Obtain connected pin
    std::shared_ptr<RwLock<OutputPin<T>>> GetConnectedPin() const { return m_connection.lock(); }

    /// @brief Tell if pin is connected
    /// @return 
    bool IsConnected() const override
    {
        return !m_connection.expired();
    }

    /// @brief Disconnect other pin
    void Disconnect() override;
    
    void OutputDisconnecting();

    std::shared_ptr<RwLock<InputPin<T>>> shared_from_this() const
    {
        return std::shared_ptr<RwLock<InputPin<T>>>(Pin<T>::GetOwningNode().shared_from_this(), &m_me);
    }

private:
    RwLock<InputPin<T>> &m_me;
    // Must be weak, as it is the node that owns the pins
    std::weak_ptr<RwLock<OutputPin<T>>> m_connection;
};

/// @brief Output pin
/// @tparam T Type of data output
template<class T> class OutputPin : public Pin<T>
{
public:
    OutputPin(INode &node, RwLock<OutputPin<T>> &me): Pin<T>(node), m_me(me) {}

    /// @brief Connect output pin
    void Connect(InputPin<T> &pin)
    {
        DBG("Connect: OutputPin ==> InputPin");

        // Check if not already connected to that pin to avoid infinite loop
        if (auto const [pos, added] = m_connections.insert(pin.shared_from_this()); added)
        {
            pin.Connect(*this);
        }
    }
    
    bool TryConnect(IRwLock<IPin> &otherPin) override
    {
        DBG("TryConnect: OutputPin ==> InputPin");

        if (auto inputPin = otherPin.try_write<InputPin<T>>(); inputPin.has_value())
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
            connectedPin->write()->SetData(Pin<T>::GetData());
        }
    }

    /// @brief Propagate processing to connected node
    void PropagateForwards() const
    {
        for (auto &connectedPin : GetConnectedPins())
        {
            INode &node = connectedPin->read()->GetOwningNode();
            node.ProcessForwards();
        }
    }
    
    /// @brief Obtain connected pin
    std::set<std::shared_ptr<RwLock<InputPin<T>>>> GetConnectedPins() const
    {
        std::set<std::shared_ptr<RwLock<InputPin<T>>>> pins;
        
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

    void DisconnectPin(InputPin<T> &pin)
    {
        if (auto pos = m_connections.find(pin.shared_from_this()); pos != m_connections.end())
        {
            m_connections.erase(pos);
            pin.Disconnect();
        }
    }

    /// @brief Disconnect other pin
    void Disconnect() override
    {
        auto connections = GetConnectedPins();
        m_connections.clear();

        for (auto &pin : connections) { pin->write()->OutputDisconnecting(); }
    }
    
    std::shared_ptr<RwLock<OutputPin<T>>> shared_from_this() const
    {
        return std::shared_ptr<RwLock<OutputPin<T>>>(Pin<T>::GetOwningNode().shared_from_this(), &m_me);
    }

private:
    RwLock<OutputPin<T>> &m_me;
    // Must be weak, as it is the node that owns the pins
    std::set<std::weak_ptr<RwLock<InputPin<T>>>, weak_ptr_compare<RwLock<InputPin<T>>>> m_connections;
};

template<class T> void InputPin<T>::Disconnect()
{
    // Since we're disconnecting input, we need to drop old data
    Pin<T>::SetData(T{});

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
}

template<class T> void InputPin<T>::OutputDisconnecting()
{
    // Since we're disconnecting input, we need to drop old data
    Pin<T>::SetData(T{});

    m_connection.reset();
}

// Implementation of InputPin::Connect
template<class T> void InputPin<T>::Connect(OutputPin<T> &pin)
{
    DBG("Connect: InputPin ==> OutputPin");

    // Check if not already connected to that pin to avoid infinite loop
    if (m_connection.expired() || m_connection.lock().get() != pin.shared_from_this().get())
    {
        m_connection = pin.shared_from_this();
        pin.Connect(*this);
    }
}

template<class T> bool InputPin<T>::TryConnect(IRwLock<IPin> &otherPin)
{
    DBG("TryConnect: InputPin ==> OutputPin");

    if (auto inputPin = otherPin.try_write<OutputPin<T>>(); inputPin.has_value())
    {
        Connect(*inputPin.value());
        return true;
    }
    return false;
}

/// @brief INode that will provide data
/// @tparam T Type of source data
template<class T> class SourceNode : public INode, public IValueLoader<T>
{
public:
    ~SourceNode() { DBG("~SourceNode()"); }
    SourceNode() : m_outputPin{*this, m_outputPin}
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

    std::set<std::shared_ptr<IRwLock<IPin>>> GetInputPins() const override { return {}; }
    std::set<std::shared_ptr<IRwLock<IPin>>> GetOutputPins() const override { return {m_outputPin.read()->shared_from_this() }; }

    void LoadValue(T &&value) override
    {
        DBG("Source.LoadValue");
        m_outputPin.write()->SetData(std::move(value));
    }

private:
    RwLock<OutputPin<T>> m_outputPin;
};

/// @brief INode that will receive final result of all procesing
/// @tparam T Type of final result data
template<class T> class TargetNode : public INode, public IValueHolder<T>
{
public:
    ~TargetNode() { DBG("~TargetNode()"); }
    TargetNode() : m_inputPin{*this, m_inputPin}
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

    std::set<std::shared_ptr<IRwLock<IPin>>> GetInputPins() const override { return { m_inputPin.read()->shared_from_this() }; }
    std::set<std::shared_ptr<IRwLock<IPin>>> GetOutputPins() const override { return {}; }

    T const &GetValue() const override
    {
        return m_inputPin.read()->GetData();
    }

private:
    RwLock<InputPin<T>> m_inputPin;
};

template<class X, class Y, class F> class TransformNode : public INode
{
public:
    ~TransformNode() { DBG("~TransformNode()"); }
    TransformNode(F f): m_function(std::move(f)), m_inputPin{*this, m_inputPin}, m_outputPin{*this, m_outputPin} {}
    
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

    std::set<std::shared_ptr<IRwLock<IPin>>> GetInputPins() const override { return {m_inputPin.read()->shared_from_this()}; }
    std::set<std::shared_ptr<IRwLock<IPin>>> GetOutputPins() const override { return {m_outputPin.read()->shared_from_this()}; }

private:
    F m_function;
    RwLock<InputPin<X>> m_inputPin;
    RwLock<OutputPin<Y>> m_outputPin;

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
    NodeGraph &AddNode(std::shared_ptr<NodeT> const &nodePtr)
    {
        m_nodes.emplace(std::move(nodePtr));
        return *this;
    }

    std::set<std::shared_ptr<INode>> GetSourceNodes()
    {
        std::set<std::shared_ptr<INode>> sourceNodes{};

        // One could use combination of std::transform() and std::copy_if(), or std::ranges
        for (auto &nodePtr : m_nodes)
        {
            if (not nodePtr->HasInputs()) { assert(nodePtr->HasOutputs()); sourceNodes.insert(nodePtr); }
        }
    
        return std::move(sourceNodes);
    }

    std::set<std::shared_ptr<INode>> GetTargetNodes()
    {
        std::set<std::shared_ptr<INode>> targetNodes{};

        for (auto &nodePtr : m_nodes)
        {
            if (not nodePtr->HasOutputs()) { assert(nodePtr->HasInputs()); targetNodes.insert(nodePtr); }
        }

        return std::move(targetNodes);
    }

    std::set<std::shared_ptr<INode>> GetTransformNodes()
    {
        std::set<std::shared_ptr<INode>> transformNodes{};

        // One could use combination of std::transform() and std::copy_if(), or std::ranges
        for (auto &nodePtr : m_nodes)
        {
            if (nodePtr->HasInputs() and nodePtr->HasOutputs()) { transformNodes.insert(nodePtr); }
        }
    
        return std::move(transformNodes);
    }

    template<class NodeType>
        std::set<std::shared_ptr<NodeType>> GetNodesOfType()
        {
            std::set<std::shared_ptr<NodeType>> nodes{};

            for (auto &nodePtr : m_nodes)
            {
                if (auto ptr = std::dynamic_pointer_cast<NodeType>(nodePtr); ptr) { nodes.insert(ptr); }
            }

            return std::move(nodes);
        }

    template<class Predicate>
        std::set<std::shared_ptr<INode>> GetMatchingNodes(Predicate &&predicate)
        {
            std::set<std::shared_ptr<INode>> nodes{};

            // One could use combination of std::transform() and std::copy_if(), or std::ranges
            for (auto &nodePtr : m_nodes)
            {
                if (predicate(nodePtr)) { nodes.insert(nodePtr); }
            }
        
            return std::move(nodes);
        }

private:
    std::set<std::shared_ptr<INode>> m_nodes;
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

std::ostream &operator <<(std::ostream &os, std::shared_ptr<ExampleDataSample> const &data)
{
    if (data) {
        return os << "[ x: " << data->x << ", y: " << data->y << "]";
    }
    else {
        return os << "[ <no value> ]";
    }
}

std::ostream &operator <<(std::ostream &os, std::shared_ptr<ExampleDataResultU> const &data)
{
    if (data) {
        return os << "[ u: " << data->u << "]";
    }
    else {
        return os << "[ <no value> ]";
    }
}

std::ostream &operator <<(std::ostream &os, std::shared_ptr<ExampleDataResultV> const &data)
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
    auto funcU = [](std::shared_ptr<ExampleDataSample> const &data)
    {
        if (data) {
            return std::make_shared<ExampleDataResultU>(
                static_cast<double>(data->x * data->y));
        }
        else {
            return std::shared_ptr<ExampleDataResultU>{};
        }
    };

    auto funcV = [](std::shared_ptr<ExampleDataSample> const &data)
    {
        if (data) {
            return std::make_shared<ExampleDataResultV>(
                static_cast<double>(data->x) / static_cast<double>(data->y));
        }
        else {
            return std::shared_ptr<ExampleDataResultV>{};
        }
    };

    // Let's create node using our transformation function
    auto transformUNodePtr = std::make_shared<TransformNode<
        std::shared_ptr<ExampleDataSample>,
        std::shared_ptr<ExampleDataResultU>,
        decltype(funcU)>>(funcU);

    auto transformVNodePtr = std::make_shared<TransformNode<
        std::shared_ptr<ExampleDataSample>,
        std::shared_ptr<ExampleDataResultV>,
        decltype(funcV)>>(funcV);

    // Now we need some source
    auto sourceNodePtr = std::make_shared<SourceNode<
        std::shared_ptr<ExampleDataSample>>>();

    // And we also need some target
    auto targetUNodePtr = std::make_shared<TargetNode<
        std::shared_ptr<ExampleDataResultU>>>();
    
    auto targetVNodePtr = std::make_shared<TargetNode<
        std::shared_ptr<ExampleDataResultV>>>();

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
    auto sourceLoader = std::dynamic_pointer_cast<IValueLoader<std::shared_ptr<ExampleDataSample>>>(sourceNode);
    assert(nullptr != sourceLoader);

    // and then we need to get output pin from the source node
    auto sourceOutputPins = sourceNode->GetOutputPins();
    assert(sourceOutputPins.size() == 1);

    // and from that pin we need to be able to get current value in it
    auto sourcePin = *std::begin(sourceOutputPins);

    auto sourcePinGetData = [sourcePin] {
        auto sourceOutputPin = sourcePin->try_read<OutputPin<std::shared_ptr<ExampleDataSample>>>();
        assert(sourceOutputPin.has_value());
        return sourceOutputPin.value()->GetData();
    };

    // We need to find target nodes
    auto targetUNodes = graph.GetNodesOfType<TargetNode<std::shared_ptr<ExampleDataResultU>>>();
    auto targetVNodes = graph.GetNodesOfType<TargetNode<std::shared_ptr<ExampleDataResultV>>>();
    assert(targetUNodes.size() == 1);
    assert(targetVNodes.size() == 1);

    // and then we need to be able to get current result from them
    auto targetUNode = *std::begin(targetUNodes);
    auto targetVNode = *std::begin(targetVNodes);
    auto targetUHolder = std::dynamic_pointer_cast<IValueHolder<std::shared_ptr<ExampleDataResultU>>>(targetUNode);
    auto targetVHolder = std::dynamic_pointer_cast<IValueHolder<std::shared_ptr<ExampleDataResultV>>>(targetVNode);
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
            for (std::shared_ptr<IRwLock<IPin>> const &pin : ptr->GetOutputPins()) {
                if (pin->try_read<OutputPin<std::shared_ptr<ExampleDataResultU>>>().has_value()) { return true; }
            }
        }
        return false;
    });
    auto transformVNodes = graph.GetMatchingNodes([](auto const &ptr) {
        if (ptr->HasInputs()) {
            for (std::shared_ptr<IRwLock<IPin>> const &pin : ptr->GetOutputPins()) {
                if (pin->try_read<OutputPin<std::shared_ptr<ExampleDataResultV>>>().has_value()) { return true; }
            }
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
    auto connectUInputResult = transformUInputPin->write_base()->TryConnect(*sourcePin);
    auto connectVInputResult = transformVInputPin->write_base()->TryConnect(*sourcePin);
    auto connectUOutputResult = transformUOutputPin->write_base()->TryConnect(*targetUPin);
    auto connectVOutputResult = transformVOutputPin->write_base()->TryConnect(*targetVPin);

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
    transformVInputPin->write_base()->Disconnect();
    {
        auto &node = transformVInputPin->read_base()->GetOwningNode();
        // The lock acquired to read owning node is now gone, and we can go ahead and tell node to propagate
        node.ProcessForwards();
    }

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
    sourcePin->write_base()->Disconnect();
    
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
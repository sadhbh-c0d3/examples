#include<memory>
#include<set>
#include<iostream>
#include<assert.h>

// local namespace
namespace {

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

struct IPin
{
    virtual bool IsConnected() const = 0;
    virtual bool TryConnect(IPin &otherPin) = 0;
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

    virtual void ProcessForwards() = 0;
    virtual void ProcessBackwards() = 0;

    virtual bool HasInputs() = 0;
    virtual bool HasOutputs() = 0;

    virtual std::set<std::shared_ptr<IPin>> GetInputPins() = 0;
    virtual std::set<std::shared_ptr<IPin>> GetOutputPins() = 0;

protected:
    // Only derived class can call this destructor
    ~INode() {}
};

/// @brief Universal pin
/// @tparam T Type of data stored in this pin
template<class T> class Pin : public IPin
{
public:
    /// @brief Obtain owning node
    INode &GetOwningNode() { return m_node; }

    /// @brief Obtain connected pin
    std::shared_ptr<Pin<T>> GetConnectedPin() { return m_connection.lock(); }

    /// @brief Set data stored in this pin
    template<class X> void SetData(X &&x) { m_data = std::forward<X>(x); }

    /// @brief Get data stored in this pin
    T const & GetData() const { return m_data; }

    /// @brief Tell if pin is connected
    /// @return 
    bool IsConnected() const override
    {
        return !m_connection.expired();
    }

    /// @brief Disconnect other pin
    void Disconnect() override
    {
        if (!m_connection.expired())
        {
            m_connection.lock()->m_connection.reset();
        }
        m_connection.reset();
    }

    std::shared_ptr<Pin<T>> shared_from_this()
    {
        return std::shared_ptr<Pin<T>>(m_node.shared_from_this(), this);
    }

private:
    // Node owns this pin
    INode &m_node;
    
    // Must be weak, as it is the node that owns the pins
    std::weak_ptr<Pin<T>> m_connection;

    T m_data;

protected:
    /// @brief Create pin attached to specific node
    Pin(INode &node): m_node(node), m_connection{}, m_data{} {}

    /// @brief Connect pin to another pin
    void Connect(Pin<T> &pin)
    {
        m_connection = pin.shared_from_this();
        pin.m_connection = shared_from_this();
    }
};

// Forward declaration of OutputPin
template<class T> struct OutputPin;

/// @brief Input pin
/// @tparam T Type of data input
template<class T> struct InputPin : Pin<T>
{
    InputPin(INode &node): Pin<T>(node) {}

    /// @brief Connect output pin (implemented below to solve catch22)
    void Connect(OutputPin<T> &pin);
    
    bool TryConnect(IPin &otherPin) override;

    /// @brief Receive data from the other connected pin into this pin
    void ReceiveData()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); connectedPin)
        {
            Pin<T>::SetData(connectedPin->GetData());
        }
    }
    
    /// @brief Propagate processing to connected node
    void PropagateBackwards()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); connectedPin)
        {
            connectedPin->GetOwningNode().ProcessBackwards();
        }
    }
};

/// @brief Output pin
/// @tparam T Type of data output
template<class T> struct OutputPin : Pin<T>
{
    OutputPin(INode &node): Pin<T>(node) {}

    /// @brief Connect output pin
    void Connect(InputPin<T> &pin) { Pin<T>::Connect(pin); }
    
    bool TryConnect(IPin &otherPin) override
    {
        if (auto inputPin = dynamic_cast<InputPin<T> *>(&otherPin); nullptr != inputPin)
        {
            Connect(*inputPin);
            return true;
        }
        return false;
    }

    /// @brief Send data from this pin to the other connected pin
    void SendData()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); connectedPin)
        {
            connectedPin->SetData(Pin<T>::GetData());
        }
    }

    /// @brief Propagate processing to connected node
    void PropagateForwards()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); connectedPin)
        {
            connectedPin->GetOwningNode().ProcessForwards();
        }
    }
};

// Implementation of InputPin::Connect
template<class T> void InputPin<T>::Connect(OutputPin<T> &pin) { Pin<T>::Connect(pin); }

template<class T> bool InputPin<T>::TryConnect(IPin &otherPin)
{
    if (auto outputPin = dynamic_cast<OutputPin<T> *>(&otherPin); nullptr != outputPin)
    {
        Connect(*outputPin);
        return true;
    }
    return false;
}

/// @brief INode that will provide data
/// @tparam T Type of source data
template<class T> class SourceNode : public INode, public IValueLoader<T>
{
public:
    ~SourceNode() { std::cout << "~SourceNode()" << std::endl; }
    SourceNode() : m_outputPin{*this}
    {}

    void ProcessForwards() override
    {
        GetOutputPin().SendData();
        GetOutputPin().PropagateForwards();
    }
    
    void ProcessBackwards() override
    {
        // This could be loading new value from somewhere
    }
    
    bool HasInputs() override { return false; }
    bool HasOutputs() override { return true; }

    std::set<std::shared_ptr<IPin>> GetInputPins() override { return {}; }
    std::set<std::shared_ptr<IPin>> GetOutputPins() override { return {m_outputPin.shared_from_this()}; }

    void LoadValue(T &&value) override
    {
        GetOutputPin().SetData(std::move(value));
    }

    OutputPin<T> &GetOutputPin() { return m_outputPin; }

private:
    OutputPin<T> m_outputPin;
};

/// @brief INode that will receive final result of all procesing
/// @tparam T Type of final result data
template<class T> class TargetNode : public INode, public IValueHolder<T>
{
public:
    ~TargetNode() { std::cout << "~TargetNode()" << std::endl; }
    TargetNode() : m_inputPin{*this}
    {}

    void ProcessForwards() override
    {
        // This could render result somewhere
    }

    void ProcessBackwards() override
    {
        GetInputPin().PropagateBackwards();
        GetInputPin().ReceiveData();
    }
    
    bool HasInputs() override { return true; }
    bool HasOutputs() override { return false; }

    std::set<std::shared_ptr<IPin>> GetInputPins() override { return {m_inputPin.shared_from_this()}; }
    std::set<std::shared_ptr<IPin>> GetOutputPins() override { return {}; }

    T const &GetValue() const override
    {
        // Process(); -- should we pull?
        return GetInputPin().GetData();
    }

    InputPin<T> &GetInputPin() { return m_inputPin; }
    InputPin<T> const &GetInputPin() const { return m_inputPin; }

private:
    InputPin<T> m_inputPin;
};

template<class X, class Y, class F> class TransformNode : public INode
{
public:
    ~TransformNode() { std::cout << "~TransformNode()" << std::endl; }
    TransformNode(F f): m_function(std::move(f)), m_inputPin{*this}, m_outputPin{*this} {}
    
    void ProcessBackwards() override
    {
        // If we're propagating backwards, then request came from output pin
        // This can happen when we're pulling from output
        GetInputPin().PropagateBackwards();
        GetInputPin().ReceiveData();
        GetOutputPin().SetData(m_function(GetInputPin().GetData()));
    }

    void ProcessForwards() override
    {
        // If we're propagate forwards, then request came from input pin
        // This can happen when we're pushing new input data
        GetOutputPin().SetData(m_function(GetInputPin().GetData()));
        GetOutputPin().SendData();
        GetOutputPin().PropagateForwards();
    }
    
    bool HasInputs() override { return true; }
    bool HasOutputs() override { return true; }

    std::set<std::shared_ptr<IPin>> GetInputPins() override { return {m_inputPin.shared_from_this()}; }
    std::set<std::shared_ptr<IPin>> GetOutputPins() override { return {m_outputPin.shared_from_this()}; }

    InputPin<X> &GetInputPin() { return m_inputPin; }
    OutputPin<Y> &GetOutputPin() { return m_outputPin; }

private:
    F m_function;
    InputPin<X> m_inputPin;
    OutputPin<Y> m_outputPin;
};

class NodeGraph
{
public:
    ~NodeGraph() { std::cout << "~NodeGraph()" << std::endl; }

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
        return std::make_shared<ExampleDataResultU>(
            static_cast<double>(data->x * data->y));
    };
    
    auto funcV = [](std::shared_ptr<ExampleDataSample> const &data)
    {
        return std::make_shared<ExampleDataResultV>(
            static_cast<double>(data->x) / static_cast<double>(data->y));
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

void test_s6_dynamic_graph_and_fork()
{
    std::cout << "TEST: test_s6_dynamic_graph_and_fork" << std::endl;

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
    auto sourceOutputPin = std::dynamic_pointer_cast<OutputPin<std::shared_ptr<ExampleDataSample>>>(sourcePin);
    assert(nullptr != sourceOutputPin);

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
            for (auto const &pin : ptr->GetOutputPins()) {
                if (std::dynamic_pointer_cast<OutputPin<std::shared_ptr<ExampleDataResultU>>>(pin)) { return true; }
            }
        }
        return false;
    });
    auto transformVNodes = graph.GetMatchingNodes([](auto const &ptr) {
        if (ptr->HasInputs()) {
            for (auto const &pin : ptr->GetOutputPins()) {
                if (std::dynamic_pointer_cast<OutputPin<std::shared_ptr<ExampleDataResultV>>>(pin)) { return true; }
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
    auto connectUInputResult = transformUInputPin->TryConnect(*sourcePin);
    auto connectVInputResult = transformVInputPin->TryConnect(*sourcePin);
    auto connectUOutputResult = transformUOutputPin->TryConnect(*targetUPin);
    auto connectVOutputResult = transformVOutputPin->TryConnect(*targetVPin);
    assert(connectUInputResult);
    assert(connectVInputResult);
    assert(connectUOutputResult);
    assert(connectVOutputResult);

    std::cout << "All set" << std::endl;

    // Here we will push the value from the source
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(7, 9));
    sourceNode->ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << sourceOutputPin->GetData() << ") => " 
              << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;

    // Here we will pull the value from each target
    sourceLoader->LoadValue(std::make_shared<ExampleDataSample>(11, 5));
    targetUNode->ProcessBackwards();
    targetVNode->ProcessBackwards();

    std::cout << "Result of processing backwards: f(" << sourceOutputPin->GetData() << ") => " 
               << targetUHolder->GetValue() << " and " << targetVHolder->GetValue() << std::endl;
}
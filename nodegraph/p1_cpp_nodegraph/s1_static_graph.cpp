#include<memory>
#include<list>
#include<iostream>

// local namespace
namespace {

/// @brief General node interface
class Node
{
public:
    // virtual ~Node() {} -- instead we define private dtor
    //                    -- note that Process() is still a virtual method
    //                    -- look at Pin<T> and it calls Process() using vtable
    virtual void ProcessForwards() = 0;
    virtual void ProcessBackwards() = 0;

protected:
    ~Node() {}
};

/// @brief Universal pin
/// @tparam T Type of data stored in this pin
template<class T> class Pin
{
public:
    /// @brief Obtain owning node
    Node &GetOwningNode() { return m_node; }

    /// @brief Obtain connected pin
    Pin<T> *GetConnectedPin() { return m_connection; }

    /// @brief Set data stored in this pin
    template<class X> void SetData(X &&x) { m_data = std::forward<X>(x); }

    /// @brief Get data stored in this pin
    T const & GetData() const { return m_data; }

    /// @brief Disconnect other pin
    void Disconnect()
    {
        if (m_connection)
        {
            m_connection->m_connection = nullptr;
            m_connection = nullptr;
        }
    }

private:
    Node &m_node;
    Pin<T> *m_connection;
    T m_data;

protected:
    /// @brief Create pin attached to specific node
    Pin(Node &node): m_node(node), m_connection(nullptr), m_data{} {}

    /// @brief Connect pin to another pin
    void Connect(Pin<T> &pin)
    {
        m_connection = &pin;
        pin.m_connection = this;
    }
};

// Forward declaration of OutputPin
template<class T> struct OutputPin;

/// @brief Input pin
/// @tparam T Type of data input
template<class T> struct InputPin : Pin<T>
{
    InputPin(Node &node): Pin<T>(node) {}

    /// @brief Connect output pin (implemented below to solve catch22)
    void Connect(OutputPin<T> &pin);

    /// @brief Receive data from the other connected pin into this pin
    void ReceiveData()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); nullptr != connectedPin)
        {
            Pin<T>::SetData(connectedPin->GetData());
        }
    }
    
    /// @brief Propagate processing to connected node
    void PropagateBackwards()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); nullptr != connectedPin)
        {
            connectedPin->GetOwningNode().ProcessBackwards();
        }
    }
};

/// @brief Output pin
/// @tparam T Type of data output
template<class T> struct OutputPin : Pin<T>
{
    OutputPin(Node &node): Pin<T>(node) {}

    /// @brief Connect output pin
    void Connect(InputPin<T> &pin) { Pin<T>::Connect(pin); }

    /// @brief Send data from this pin to the other connected pin
    void SendData()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); nullptr != connectedPin)
        {
            connectedPin->SetData(Pin<T>::GetData());
        }
    }

    /// @brief Propagate processing to connected node
    void PropagateForwards()
    {
        if (auto connectedPin = Pin<T>::GetConnectedPin(); nullptr != connectedPin)
        {
            connectedPin->GetOwningNode().ProcessForwards();
        }
    }
};

// Implementation of InputPin::Connect
template<class T> void InputPin<T>::Connect(OutputPin<T> &pin) { Pin<T>::Connect(pin); }

/// @brief Node that will provide data
/// @tparam T Type of source data
template<class T> class SourceNode : Node
{
public:
    ~SourceNode() {}
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

    template<class X> void LoadValue(X &&x)
    {
        GetOutputPin().SetData(std::forward<X>(x));
    }

    OutputPin<T> &GetOutputPin() { return m_outputPin; }

private:
    OutputPin<T> m_outputPin;
};

/// @brief Node that will receive final result of all procesing
/// @tparam T Type of final result data
template<class T> class TargetNode : Node
{
public:
    ~TargetNode() {}
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

    const T &GetValue() const 
    {
        // Process(); -- should we pull?
        return GetInputPin().GetData();
    }

    InputPin<T> &GetInputPin() { return m_inputPin; }
    InputPin<T> const &GetInputPin() const { return m_inputPin; }

private:
    InputPin<T> m_inputPin;
};

template<class X, class Y, class F> class TransformNode : Node
{
public:
    ~TransformNode() {}
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

    InputPin<X> &GetInputPin() { return m_inputPin; }
    OutputPin<Y> &GetOutputPin() { return m_outputPin; }

private:
    F m_function;
    InputPin<X> m_inputPin;
    OutputPin<Y> m_outputPin;
};

} // end of local namespace

void test_int_to_double()
{
    // Let's declare some transformation function: int -> double
    auto func = [](int x) { return x * 1.5; };

    // Let's create node using our transformation function
    TransformNode<int, double, decltype(func)> transformNode{func};
    
    // Now we need some source
    SourceNode<int> sourceNode;

    // And we also need some target
    TargetNode<double> targetNode;

    transformNode.GetInputPin().Connect(sourceNode.GetOutputPin());
    transformNode.GetOutputPin().Connect(targetNode.GetInputPin());

    // Here we will push the value from the source
    sourceNode.LoadValue(10.0);
    sourceNode.ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << sourceNode.GetOutputPin().GetData() << ") => " << targetNode.GetValue() << std::endl;
    
    // Here we will pull the value from the target
    sourceNode.LoadValue(50.0);
    targetNode.ProcessBackwards();
    
    std::cout << "Result of processing backwards: f(" << sourceNode.GetOutputPin().GetData() << ") => " << targetNode.GetValue() << std::endl;

    // We prevent by design storage of the node that would causing type erasure
    //Node *ptr = &transformNode; -- This is invalid as TransformNode inherits Node privately
    //delete ptr; -- This is invalid as ~Node is private
}

template<class T>
std::ostream &operator <<(std::ostream &os, std::tuple<T, T> const &x)
{
    const auto [a, b] = x;
    return os << "[" << a << ", " << b << "]";
}

void test_with_shared_ptr()
{
    auto func = [](std::shared_ptr<std::tuple<int, int>> const &x)
    {
        const auto [a, b] = *x;
        return std::make_shared<std::tuple<double, double>>(a * b, a / (double)b);
    };

    // Let's create node using our transformation function
    TransformNode<
        std::shared_ptr<std::tuple<int, int>>,
        std::shared_ptr<std::tuple<double, double>>,
        decltype(func)> transformNode{func};
    
    // Now we need some source
    SourceNode<std::shared_ptr<std::tuple<int, int>>> sourceNode;

    // And we also need some target
    TargetNode<std::shared_ptr<std::tuple<double, double>>> targetNode;

    transformNode.GetInputPin().Connect(sourceNode.GetOutputPin());
    transformNode.GetOutputPin().Connect(targetNode.GetInputPin());

    // Here we will push the value from the source
    sourceNode.LoadValue(std::make_shared<std::tuple<int, int>>(7, 9));
    sourceNode.ProcessForwards();
    
    std::cout << "Result of processing forwards: f(" << *sourceNode.GetOutputPin().GetData() << ") => " << *targetNode.GetValue() << std::endl;
    
    // Here we will pull the value from the target
    sourceNode.LoadValue(std::make_shared<std::tuple<int, int>>(11, 5));
    targetNode.ProcessBackwards();
    
    std::cout << "Result of processing backwards: f(" << *sourceNode.GetOutputPin().GetData() << ") => " << *targetNode.GetValue() << std::endl;
}

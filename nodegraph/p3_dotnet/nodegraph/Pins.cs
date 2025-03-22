namespace nodegraph
{
    public interface IValueLoader<T> where T: new()
    {
        public void LoadValue(T value);
    }

    public interface IValueHolder<T> where T: new()
    {
        public T Value { get; }
    }

    public interface IPin
    {
        public bool IsConnected { get; }
        public DeferredAction Disconnect();
        public INode OwningNode { get; }
    }

    public interface INode
    {
        public IEnumerable<IPin> InputPins { get; }
        public IEnumerable<IPin> OutputPins { get; }

        public DeferredAction ProcessForwards();
        public DeferredAction ProcessBackwards();
    }

    public abstract class Pin<T> : IPin
        where T: new()
    {
        public Pin(INode owningNode)
        {
            m_node = owningNode;
            m_data = new T();
        }

        public abstract bool IsConnected { get; }
        public abstract DeferredAction Disconnect();

        public INode OwningNode { get { return m_node; } }
        public T Data { set { m_data = value; }  get { return m_data; } }

        private readonly INode m_node;
        private T m_data;
    }

    public sealed class InputPin<T> : Pin<T>
        where T: new()
    {
        public InputPin(INode owningNode) : base(owningNode)
        {
            m_connection = null;
        }

        public DeferredAction PropagateBackwards()
        {
            return m_connection?.OwningNode.ProcessBackwards() ?? DeferredAction.NoAction;
        }

        public void ReceiveData()
        {
            var connection = m_connection;
            if (connection != null)
            {
                Data = connection.Data;
            }
        }

        public override bool IsConnected => (m_connection != null);

        public bool Connect(OutputPin<T> outputPin)
        {
            if (m_connection != null)
            {
                return (m_connection == outputPin);
            }
            
            m_connection = outputPin;

            if (false == m_connection.Connect(this))
            {
                m_connection = null;
                return false;
            }

            return true;
        }

        public override DeferredAction Disconnect()
        {
            if (m_connection != null)
            {
                var connection = m_connection;
                m_connection = null;
                connection.DisconnectPin(this);

                return connection.OwningNode.ProcessForwards();
            }

            return DeferredAction.NoAction;
        }

        private OutputPin<T> ?m_connection;
    }

    public sealed class OutputPin<T> : Pin<T>
        where T: new()
    {
        public OutputPin(INode owningNode): base(owningNode)
        { 
            m_connections = new HashSet<InputPin<T>>();
        }

        public DeferredAction PropagateForwards()
        {
            var actions = from connection in m_connections
                          let node = connection.OwningNode
                          select node.ProcessForwards();

            return new DeferredParallel(actions.ToArray());
        }

        public void SendData()
        {
            foreach (var connection in m_connections)
            {
                connection.Data = Data;
            }
        }

        public override bool IsConnected => (m_connections.Count > 0);

        public bool Connect(InputPin<T> inputPin)
        {
            if (false == m_connections.Add(inputPin))
            {
                return true;
            }

            if (false == inputPin.Connect(this))
            {
                m_connections.Remove(inputPin);
                return false;
            }

            return true;
        }

        public DeferredAction DisconnectPin(InputPin<T> pin)
        {
            if (m_connections.Remove(pin))
            {
                return pin.Disconnect();
            }

            return DeferredAction.NoAction;
        }

        public override DeferredAction Disconnect()
        {
            var connections = m_connections;
            m_connections = new HashSet<InputPin<T>>();

            var actions = from connection in connections
                          select connection.Disconnect();

            return new DeferredParallel(actions.ToArray());
        }

        private HashSet<InputPin<T>> m_connections;
    }

}

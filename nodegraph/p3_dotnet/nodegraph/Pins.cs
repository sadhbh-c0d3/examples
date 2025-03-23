namespace nodegraph
{
    public class Constants
    {
        public static TimeSpan Timeout = TimeSpan.FromSeconds(3);
    }

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
        public IEnumerable<IRwLock<IPin>> InputPins { get; }
        public IEnumerable<IRwLock<IPin>> OutputPins { get; }

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

    public sealed class InputPin<T> : Pin<T>, IRwLockFromThis<InputPin<T>>
        where T: new()
    {
        public InputPin(INode owningNode) : base(owningNode)
        {
            m_connection = null;
        }

        public DeferredAction PropagateBackwards()
        {
            return m_connection?.UnguardedData.OwningNode.ProcessBackwards() ?? DeferredAction.NoAction;
        }

        public void ReceiveData()
        {
            var connection = m_connection;
            if (connection != null)
            {
                Data = connection.WithRead(Constants.Timeout, x => x.Data);
            }
        }

        public override bool IsConnected => (m_connection != null);

        public RwLock<InputPin<T>> RwLockSelf { set => m_self = value; }

        public bool Connect(RwLock<OutputPin<T>> outputPin)
        {
            if (m_self == null) {  return false; }

            if (m_connection != null)
            {
                return (m_connection == outputPin);
            }
            
            m_connection = outputPin;

            if (false == m_connection.WithWrite(Constants.Timeout, x => x.Connect(m_self)))
            {
                m_connection = null;
                return false;
            }

            return true;
        }

        public override DeferredAction Disconnect()
        {
            if (m_connection != null && m_self != null)
            {
                var connection = m_connection;
                m_connection = null;

                connection.WithWrite(Constants.Timeout, x => x.DisconnectPin(m_self));

                return connection.UnguardedData.OwningNode.ProcessForwards();
            }

            return DeferredAction.NoAction;
        }

        private RwLock<InputPin<T>> ?m_self;
        private RwLock<OutputPin<T>> ?m_connection;
    }

    public sealed class OutputPin<T> : Pin<T>, IRwLockFromThis<OutputPin<T>>
        where T: new()
    {
        public OutputPin(INode owningNode): base(owningNode)
        { 
            m_connections = new HashSet<RwLock<InputPin<T>>>();
        }

        public DeferredAction PropagateForwards()
        {
            var actions = from connection in m_connections
                          let node = connection.UnguardedData.OwningNode
                          select node.ProcessForwards();

            return new DeferredParallel(actions.ToArray());
        }

        public void SendData()
        {
            foreach (var connection in m_connections)
            {
                connection.WithWrite(Constants.Timeout, x => x.Data = Data);
            }
        }

        public override bool IsConnected => (m_connections.Count > 0);

        public RwLock<OutputPin<T>> RwLockSelf { set => m_self = value; }

        public bool Connect(RwLock<InputPin<T>> inputPin)
        {
            if (m_self == null) {  return false; }

            if (false == m_connections.Add(inputPin))
            {
                return true;
            }

            if (false == inputPin.WithWrite(Constants.Timeout, x => x.Connect(m_self)))
            {
                m_connections.Remove(inputPin);
                return false;
            }

            return true;
        }

        public DeferredAction DisconnectPin(RwLock<InputPin<T>> pin)
        {
            if (m_connections.Remove(pin))
            {
                return pin.WithWrite(Constants.Timeout, x => x.Disconnect());
            }

            return DeferredAction.NoAction;
        }

        public override DeferredAction Disconnect()
        {
            var connections = m_connections;
            m_connections = new HashSet<RwLock<InputPin<T>>>();

            var actions = from connection in connections
                          select connection.WithWrite(Constants.Timeout, x => x.Disconnect());

            return new DeferredParallel(actions.ToArray());
        }

        private RwLock<OutputPin<T>>? m_self;
        private HashSet<RwLock<InputPin<T>>> m_connections;
    }

}

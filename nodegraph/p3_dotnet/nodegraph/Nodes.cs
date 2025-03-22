namespace nodegraph
{
    public class SourceNode<T> : INode, IValueLoader<T> where T: new()
    {
        public SourceNode()
        {
            m_outputPin = new OutputPin<T>(this);
        }

        public IEnumerable<IPin> InputPins => Enumerable.Empty<IPin>();

        public IEnumerable<IPin> OutputPins => new[] { m_outputPin };

        public DeferredAction ProcessForwards()
        {
            m_outputPin.SendData();
            return m_outputPin.PropagateForwards();
        }

        public DeferredAction ProcessBackwards()
        {
            return DeferredAction.NoAction;
        }

        public void LoadValue(T data)
        {
            m_outputPin.Data = data;
        }

        private readonly OutputPin<T> m_outputPin;
    }

    public class TargetNode<T> : INode, IValueHolder<T> where T: new()
    {
        public TargetNode()
        {
            m_inputPin = new InputPin<T>(this);
        }

        public IEnumerable<IPin> InputPins => new[] { m_inputPin };

        public IEnumerable<IPin> OutputPins => Enumerable.Empty<IPin>();

        public DeferredAction ProcessForwards()
        {
            return DeferredAction.NoAction;
        }

        public DeferredAction ProcessBackwards()
        {
            var lambda = async () =>
            {
                await m_inputPin.PropagateBackwards().Run();
                m_inputPin.ReceiveData();
            };

            return new DeferredCoroutine(lambda());
        }

        public T Value => m_inputPin.Data;

        private readonly InputPin<T> m_inputPin;
    }

    public class TransformNode<X, Y> : INode
        where X: new()
        where Y: new()
    {
        public TransformNode(Func<X, Y> func)
        {
            m_inputPin = new InputPin<X>(this);
            m_outputPin = new OutputPin<Y>(this);
            m_func = func;
        }

        public IEnumerable<IPin> InputPins => new[] { m_inputPin };

        public IEnumerable<IPin> OutputPins => new[] { m_outputPin };

        public DeferredAction ProcessForwards()
        {
            var lambda = async () =>
            {
                var result = m_func(m_inputPin.Data);
                m_outputPin.Data = result;
                m_outputPin.SendData();
                await m_outputPin.PropagateForwards().Run();
            };

            return new DeferredCoroutine(lambda());
        }

        public DeferredAction ProcessBackwards()
        {
            var lambda = async () =>
            {
                await m_inputPin.PropagateBackwards().Run();
                m_inputPin.ReceiveData();
                var result = m_func(m_inputPin.Data);
                m_outputPin.Data = result;
            };

            return new DeferredCoroutine(lambda());
        }

        private readonly InputPin<X> m_inputPin;
        private readonly OutputPin<Y> m_outputPin;
        private readonly Func<X, Y> m_func;
    }
}

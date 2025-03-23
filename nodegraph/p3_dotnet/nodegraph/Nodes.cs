namespace nodegraph
{
    public class SourceNode<T> : INode, IValueLoader<T> where T: new()
    {
        public SourceNode()
        {
            m_outputPin = new RwLock<OutputPin<T>>(new OutputPin<T>(this));
        }

        public IEnumerable<IRwLock<IPin>> InputPins => Enumerable.Empty<IRwLock<IPin>>();

        public IEnumerable<IRwLock<IPin>> OutputPins => new[] { m_outputPin };

        public DeferredAction ProcessForwards() => m_outputPin.WithRead(Constants.Timeout,
            x => {
                x.SendData();
                return x.PropagateForwards();
            });

        public DeferredAction ProcessBackwards()
        {
            return DeferredAction.NoAction;
        }

        public void LoadValue(T data)
        {
            m_outputPin.WithWrite(Constants.Timeout, x => x.Data = data);
        }

        private readonly RwLock<OutputPin<T>> m_outputPin;
    }

    public class TargetNode<T> : INode, IValueHolder<T> where T: new()
    {
        public TargetNode()
        {
            m_inputPin = new RwLock<InputPin<T>>(new InputPin<T>(this));
        }

        public IEnumerable<IRwLock<IPin>> InputPins => new[] { m_inputPin };

        public IEnumerable<IRwLock<IPin>> OutputPins => Enumerable.Empty<IRwLock<IPin>>();

        public DeferredAction ProcessForwards()
        {
            return DeferredAction.NoAction;
        }

        public DeferredAction ProcessBackwards()
        {
            var lambda = async () =>
            {
                var action = m_inputPin.WithRead(Constants.Timeout, x => x.PropagateBackwards());

                await action.Run();

                m_inputPin.WithWrite(Constants.Timeout, x => x.ReceiveData());
            };

            return new DeferredCoroutine(lambda());
        }

        public T Value => m_inputPin.WithRead(Constants.Timeout, x => x.Data);

        private readonly RwLock<InputPin<T>> m_inputPin;
    }

    public class TransformNode<X, Y> : INode
        where X: new()
        where Y: new()
    {
        public TransformNode(Func<X, Y> func)
        {
            m_inputPin = new RwLock<InputPin<X>>(new InputPin<X>(this));
            m_outputPin =  new RwLock<OutputPin<Y>>(new OutputPin<Y>(this));
            m_func = func;
        }

        public IEnumerable<IRwLock<IPin>> InputPins => new[] { m_inputPin };

        public IEnumerable<IRwLock<IPin>> OutputPins => new[] { m_outputPin };

        public DeferredAction ProcessForwards()
        {
            var lambda = async () =>
            {
                var result = m_func(m_inputPin.WithRead(Constants.Timeout, x => x.Data));
                m_outputPin.WithWrite(Constants.Timeout, x => {
                    x.Data = result;
                    x.SendData();
                });
                var action = m_outputPin.WithRead(Constants.Timeout, x => x.PropagateForwards());
                await action.Run();
            };

            return new DeferredCoroutine(lambda());
        }

        public DeferredAction ProcessBackwards()
        {
            var lambda = async () =>
            {
                var action = m_inputPin.WithRead(Constants.Timeout, x => x.PropagateBackwards());
                await action.Run();
                var data = m_inputPin.WithWrite(Constants.Timeout, x =>
                {
                    x.ReceiveData();
                    return x.Data;
                });
                var result = m_func(data);
                m_outputPin.WithWrite(Constants.Timeout, x => x.Data = result);
            };

            return new DeferredCoroutine(lambda());
        }

        private readonly RwLock<InputPin<X>> m_inputPin;
        private readonly RwLock<OutputPin<Y>> m_outputPin;
        private readonly Func<X, Y> m_func;
    }
}

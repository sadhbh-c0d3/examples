namespace nodegraph
{

    public interface ILockGuard<out T> : IDisposable
    {
        public T Data { get; }
    }

    public class ReadLockGuard<T> : ILockGuard<T>
    {
        internal ReadLockGuard(ReaderWriterLock @lock, T data, TimeSpan timeout)
        {
            m_lock = @lock;
            m_data = data;
            m_lock.AcquireReaderLock(timeout);
        }

        public void Dispose()
        {
            m_lock.ReleaseReaderLock();
        }

        public T Data => m_data;

        private readonly ReaderWriterLock m_lock;
        private readonly T m_data;
    }
    
    public class WriteLockGuard<T> : ILockGuard<T>
    {
        internal WriteLockGuard(ReaderWriterLock @lock, T data, TimeSpan timeout)
        {
            m_lock = @lock;
            m_data = data;
            m_lock.AcquireWriterLock(timeout);
        }

        public void Dispose()
        {
            m_lock.ReleaseWriterLock();
        }
       
        public T Data => m_data;

        private readonly ReaderWriterLock m_lock;
        private readonly T m_data;
    }

    public interface IRwLock<out T>
    {
        public ILockGuard<T> Write(TimeSpan timeout);
        public ILockGuard<T> Read(TimeSpan timeout);

        public T UnguardedData { get; }
    }

    public interface IRwLockFromThis<T>
    {
        public RwLock<T> RwLockSelf { set; }
    }

    public class RwLock<T> : IRwLock<T>
    {
        public RwLock(T data)
        {
            m_lock = new ReaderWriterLock();
            m_data = data;

            var with_self = m_data as IRwLockFromThis<T>;
            if (with_self != null)
            {
                with_self.RwLockSelf = this;
            }
        }

        public WriteLockGuard<T> Write(TimeSpan timeout)
        {
            return new WriteLockGuard<T>(m_lock, m_data, timeout);
        }

        public ReadLockGuard<T> Read(TimeSpan timeout)
        {
            return new ReadLockGuard<T>(m_lock, m_data, timeout);
        }

        ILockGuard<T> IRwLock<T>.Write(TimeSpan timeout) => Write(timeout);

        ILockGuard<T> IRwLock<T>.Read(TimeSpan timeout) => Read(timeout);

        public void WithWrite(TimeSpan timeout, Action<T> write)
        {
            using (var guard = Write(timeout))
            {
                write(guard.Data);
            }
        }
        
        public void WithRead(TimeSpan timeout, Action<T> read)
        {
            using (var guard = Read(timeout))
            {
                read(guard.Data);
            }
        }

        public Z WithWrite<Z>(TimeSpan timeout, Func<T, Z> write)
        {
            using (var guard = Write(timeout))
            {
                return write(guard.Data);
            }
        }

        public Z WithRead<Z>(TimeSpan timeout, Func<T, Z> read)
        {
            using (var guard = Read(timeout))
            {
                return read(guard.Data);
            }
        }

        public T UnguardedData => m_data;

        private readonly ReaderWriterLock m_lock;
        private readonly T m_data;
    }

}

namespace nodegraph
{
    public abstract class DeferredAction
    {
        public abstract Task Run();

        public static NoDeferredAction NoAction { get; } = new NoDeferredAction();
    }

    public sealed class NoDeferredAction : DeferredAction
    {
        public override Task Run() { return Task.CompletedTask; }
    }

    public sealed class DeferredSingle : DeferredAction
    {
        public DeferredSingle(Func<DeferredAction> action)
        {
            this.action = action;
        }

        public override Task Run()
        {
            action();

            return Task.CompletedTask;
        }

        public readonly Func<DeferredAction> action;
    }

    public sealed class DeferredSequential : DeferredAction
    {
        public DeferredSequential(DeferredAction[] actions)
        {
            this.actions = actions;
        }

        public override Task Run()
        {
            var lambda = async () =>
            {
                foreach (var action in actions)
                {
                    await action.Run();
                }
            };

            return lambda();
        }

        public readonly DeferredAction[] actions;
    }

    public sealed class DeferredParallel : DeferredAction
    {
        public DeferredParallel(DeferredAction[] actions)
        {
            this.actions = actions;
        }

        public override Task Run()
        {
            var tasks = from action in actions
                        select action.Run();

            return Task.WhenAll(tasks);
        }

        public readonly DeferredAction[] actions;
    }

    public sealed class DeferredCoroutine : DeferredAction
    {
        public DeferredCoroutine(Task task)
        {
            this.task = task;
        }

        public override Task Run()
        {
            return task;
        }

        public readonly Task task;
    }

}

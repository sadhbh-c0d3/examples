namespace nodegraph
{
    public class NodeGraph
    {
        public NodeGraph AddNode(INode node)
        {
            m_nodes.Add(node);

            return this;
        }

        public IEnumerable<INode> FindNodes(Func<INode, bool> pred)
        {
            return m_nodes.Where(pred);
        }

        private readonly HashSet<INode> m_nodes = new HashSet<INode>();
    }
}

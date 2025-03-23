using nodegraph;
using System.Diagnostics;

public class Program
{
	private NodeGraph m_graph = new NodeGraph();

	private (INode, INode, INode) BuildGraph()
	{
		var sourceNode = new SourceNode<int>();
		var targetNode1 = new TargetNode<float>();
		var targetNode2 = new TargetNode<float>();
        var transformNode1 = new TransformNode<int, float>(x => x / 2.0f);
        var transformNode2 = new TransformNode<int, float>(x => x * 1.5f);

		m_graph.AddNode(sourceNode)
			.AddNode(targetNode1)
			.AddNode(targetNode2)
			.AddNode(transformNode1)
			.AddNode(transformNode2);

		var sourceOutPin = sourceNode.OutputPins.Single() as RwLock<OutputPin<int>>;
		var target1InPin = targetNode1.InputPins.Single() as RwLock<InputPin<float>>;
		var target2InPin = targetNode2.InputPins.Single() as RwLock<InputPin<float>>;
		var tranform1InPin = transformNode1.InputPins.Single() as RwLock<InputPin<int>>;
		var tranform1OutPin = transformNode1.OutputPins.Single() as RwLock<OutputPin<float>>;
		var tranform2InPin = transformNode2.InputPins.Single() as RwLock<InputPin<int>>;
		var tranform2OutPin = transformNode2.OutputPins.Single() as RwLock<OutputPin<float>>;

		Debug.Assert(sourceOutPin != null);
		Debug.Assert(target1InPin != null);
		Debug.Assert(target2InPin != null);
		Debug.Assert(tranform1InPin != null);
		Debug.Assert(tranform1OutPin != null);
		Debug.Assert(tranform2InPin != null);
		Debug.Assert(tranform2OutPin != null);

		sourceOutPin.WithWrite(Constants.Timeout, x => x.Connect(tranform1InPin));
		sourceOutPin.WithWrite(Constants.Timeout, x => x.Connect(tranform2InPin));
		target1InPin.WithWrite(Constants.Timeout, x => x.Connect(tranform1OutPin));
		target2InPin.WithWrite(Constants.Timeout, x => x.Connect(tranform2OutPin));

		return (sourceNode, targetNode1, targetNode2);
    }

	private void Run()
	{
		var (src, dst1, dst2) = BuildGraph();

		var sourceLoader = src as IValueLoader<int>;
		var targetHolder1 = dst1 as IValueHolder<float>;
		var targetHolder2 = dst2 as IValueHolder<float>;
		var sourceOutputPin = (src.OutputPins.Single() as RwLock<OutputPin<int>>);

		Debug.Assert(sourceLoader != null);
		Debug.Assert(targetHolder1 != null);
		Debug.Assert(targetHolder2 != null);
		Debug.Assert(sourceOutputPin != null);
		
		sourceLoader.LoadValue(5);
		Task.WaitAll(src.ProcessForwards().Run());

		Console.WriteLine("Result of processing forwards: " +
			$"{sourceOutputPin.WithRead(Constants.Timeout, x => x.Data)} => {targetHolder1.Value}, {targetHolder2.Value}");

		sourceLoader.LoadValue(7);
		Task.WaitAll(
			dst1.ProcessBackwards().Run(),
			dst2.ProcessBackwards().Run());

		Console.WriteLine("Result of processing backwards: " + 
			$"{sourceOutputPin.WithRead(Constants.Timeout, x => x.Data)} => {targetHolder1.Value}, {targetHolder2.Value}");

	}

    public static void Main()
	{
		var program = new Program();
		program.Run();
	}
}

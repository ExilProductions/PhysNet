using BenchmarkDotNet.Running;

namespace PhysNet.Benchmarks
{
    internal class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run(new[]
            {
                typeof(Benchmarks.BroadphaseBenchmarks),
                typeof(Benchmarks.NarrowphaseBenchmarks),
                typeof(Benchmarks.WorldStepBenchmarks)
            });
        }
    }
}

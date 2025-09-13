using System.Numerics;
using BenchmarkDotNet.Attributes;
using PhysNet;
using PhysNet.Dynamics;
using PhysNet.World;

namespace PhysNet.Benchmarks.Benchmarks
{
    [MemoryDiagnoser]
    public class WorldStepBenchmarks
    {
        private PhysicsWorld _world = null!;

        [GlobalSetup]
        public void Setup()
        {
            _world = new PhysicsWorld();
            _world.SolverSettings.Iterations = 6;
            var ground = Physics.CreateStaticBox(new Vector3(100, 1, 100), new Vector3(0, -1, 0));
            _world.AddBody(ground);
            for (int i = 0; i < 500; i++)
            {
                var b = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(i % 25, 5 + i / 25, (i / 5) % 25));
                _world.AddBody(b);
            }
        }

        [Benchmark]
        public void StepMany()
        {
            for (int i = 0; i < 10; i++)
            {
                _world.Step(1f / 60f);
            }
        }
    }
}

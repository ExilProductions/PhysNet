using System.Numerics;
using BenchmarkDotNet.Attributes;
using PhysNet.Collision.Narrowphase;
using PhysNet.Collision.Shapes;
using PhysNet.Math;

namespace PhysNet.Benchmarks.Benchmarks
{
    [MemoryDiagnoser]
    public class NarrowphaseBenchmarks
    {
        private SphereShape _s1 = null!;
        private SphereShape _s2 = null!;
        private BoxShape _b1 = null!;
        private BoxShape _b2 = null!;
        private Transform _t1, _t2;

        [GlobalSetup]
        public void Setup()
        {
            _s1 = new SphereShape(0.5f);
            _s2 = new SphereShape(0.5f);
            _b1 = new BoxShape(new Vector3(1));
            _b2 = new BoxShape(new Vector3(1));
            _t1 = new Transform(new Vector3(0, 0, 0), Quaternion.Identity);
            _t2 = new Transform(new Vector3(0.75f, 0, 0), Quaternion.Identity);
        }

        [Benchmark]
        public bool SphereSphere()
        {
            return CollidePrimitives.Collide(_s1, _t1, _s2, _t2, out _);
        }

        [Benchmark]
        public bool GjkBoxBox()
        {
            return GjkEpa.Intersect(_b1, _t1, _b2, _t2, out _, out _);
        }
    }
}

using System.Collections.Generic;
using System.Numerics;
using BenchmarkDotNet.Attributes;
using PhysNet.Collision.Broadphase;

namespace PhysNet.Benchmarks.Benchmarks
{
    [MemoryDiagnoser]
    public class BroadphaseBenchmarks
    {
        private DynamicAabbTree<int> _tree = null!;
        private List<int> _results = null!;
        private Aabb[] _aabbs = null!;

        [GlobalSetup]
        public void Setup()
        {
            _tree = new DynamicAabbTree<int>();
            _results = new List<int>(1024);
            _aabbs = new Aabb[1000];
            var rnd = new System.Random(1);
            for (int i = 0; i < _aabbs.Length; i++)
            {
                var center = new Vector3((float)rnd.NextDouble() * 100f, (float)rnd.NextDouble() * 100f, (float)rnd.NextDouble() * 100f);
                var ext = new Vector3(0.5f, 0.5f, 0.5f);
                _aabbs[i] = Aabb.FromCenterExtents(center, ext);
                _tree.Insert(_aabbs[i], i);
            }
        }

        [Benchmark]
        public int QueryMany()
        {
            _results.Clear();
            for (int i = 0; i < _aabbs.Length; i++)
            {
                _tree.Query(_aabbs[i], _results);
            }
            return _results.Count;
        }

        [Benchmark]
        public bool RayCastMany()
        {
            bool cont = true;
            for (int i = 0; i < 100; i++)
            {
                cont &= _tree.RayCast(Vector3.Zero, Vector3.Normalize(new Vector3(1, 0.1f, 1)), 1000f, (item, box) => true);
            }
            return cont;
        }
    }
}

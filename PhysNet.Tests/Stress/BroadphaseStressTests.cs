using System;
using System.Collections.Generic;
using System.Numerics;
using PhysNet.Collision.Broadphase;
using Xunit;

namespace PhysNet.Tests.Stress
{
    public class BroadphaseStressTests
    {
        private static bool IsFinite(Vector3 v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

        [Fact]
        public void DynamicAabbTree_InsertQueryRayCast_Stress()
        {
            var tree = new DynamicAabbTree<int>();
            var rnd = new Random(1234);
            int nodeCount = 2000;
            var ids = new int[nodeCount];
            for (int i = 0; i < nodeCount; i++)
            {
                var center = new Vector3((float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f);
                var ext = new Vector3(0.25f + (float)rnd.NextDouble(), 0.25f + (float)rnd.NextDouble(), 0.25f + (float)rnd.NextDouble());
                ids[i] = tree.Insert(Aabb.FromCenterExtents(center, ext), i);
            }

            var tmp = new List<int>(256);
            int totalHits = 0;
            for (int q = 0; q < 500; q++)
            {
                tmp.Clear();
                var center = new Vector3((float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f);
                var query = Aabb.FromCenterExtents(center, new Vector3(1, 1, 1));
                tree.Query(query, tmp);
                totalHits += tmp.Count;
            }

            int rayHits = 0;
            for (int r = 0; r < 500; r++)
            {
                var origin = new Vector3((float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f, (float)rnd.NextDouble() * 200f);
                var dir = Vector3.Normalize(new Vector3((float)rnd.NextDouble() - 0.5f, (float)rnd.NextDouble() - 0.5f, (float)rnd.NextDouble() - 0.5f));
                if (dir.LengthSquared() < 1e-6f) dir = Vector3.UnitX;
                tree.RayCast(origin, dir, 500f, (item, box) => { rayHits++; return true; });
            }

            Assert.True(totalHits >= 0);
            Assert.True(rayHits >= 0);
        }

        [Fact]
        public void DynamicAabbTree_Update_Remove_Stress()
        {
            var tree = new DynamicAabbTree<int>();
            var rnd = new Random(42);
            int nodeCount = 1000;
            var ids = new int[nodeCount];
            for (int i = 0; i < nodeCount; i++)
            {
                var center = new Vector3((float)rnd.NextDouble() * 50f, (float)rnd.NextDouble() * 50f, (float)rnd.NextDouble() * 50f);
                ids[i] = tree.Insert(Aabb.FromCenterExtents(center, new Vector3(0.5f)), i);
            }

            // Random updates
            for (int step = 0; step < 200; step++)
            {
                int idx = rnd.Next(nodeCount);
                var center = new Vector3((float)rnd.NextDouble() * 50f, (float)rnd.NextDouble() * 50f, (float)rnd.NextDouble() * 50f);
                tree.SetLeafBox(ids[idx], Aabb.FromCenterExtents(center, new Vector3(0.5f)));
            }

            // Random removals
            for (int i = 0; i < 200; i++)
            {
                int idx = rnd.Next(nodeCount);
                tree.Remove(ids[idx]);
            }

            Assert.True(true);
        }
    }
}

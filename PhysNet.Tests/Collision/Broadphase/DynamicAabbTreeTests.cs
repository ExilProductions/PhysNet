using System;
using System.Collections.Generic;
using System.Numerics;
using PhysNet.Collision.Broadphase;
using Xunit;

namespace PhysNet.Tests.Collision.Broadphase
{
    public class DynamicAabbTreeTests
    {
        [Fact]
        public void InsertQuery_FindOverlaps()
        {
            var tree = new DynamicAabbTree<int>();
            var idA = tree.Insert(new Aabb(new Vector3(0,0,0), new Vector3(1,1,1)), 0);
            var idB = tree.Insert(new Aabb(new Vector3(0.5f,0.5f,0.5f), new Vector3(1.5f,1.5f,1.5f)), 1);
            var idC = tree.Insert(new Aabb(new Vector3(3,3,3), new Vector3(4,4,4)), 2);

            var query = new Aabb(new Vector3(0.25f), new Vector3(1.25f));
            var results = new List<int>();
            tree.Query(query, results);
            var items = new HashSet<int>();
            foreach (var leaf in results)
            {
                var (_, item) = tree.GetLeaf(leaf);
                items.Add(item);
            }
            Assert.Contains(0, items);
            Assert.Contains(1, items);
            Assert.DoesNotContain(2, items);
        }

        [Fact]
        public void RayCast_VisitsExpected()
        {
            var tree = new DynamicAabbTree<int>();
            tree.Insert(new Aabb(new Vector3(0,0,0), new Vector3(1,1,1)), 0);
            tree.Insert(new Aabb(new Vector3(5,5,5), new Vector3(6,6,6)), 1);
            int count = 0;
            tree.RayCast(Vector3.Zero, Vector3.Normalize(new Vector3(1,1,1)), 10f, (item, box) => { count++; return true; });
            Assert.Equal(2, count);
        }

        [Fact]
        public void RayCast_VisitsExpected_AndHandlesZeroComponents()
        {
            var tree = new DynamicAabbTree<int>();
            tree.Insert(new Aabb(new Vector3(0,0,0), new Vector3(1,1,1)), 0);
            tree.Insert(new Aabb(new Vector3(5,5,5), new Vector3(6,6,6)), 1);
            int count = 0;
            // include a zero component direction to test robustness
            tree.RayCast(Vector3.Zero, new Vector3(1,0,1), 10f, (item, box) => { count++; return true; });
            // Only the first AABB at y in [0,1] should be hit since the ray stays at y=0 plane
            Assert.Equal(1, count);
        }
    }
}

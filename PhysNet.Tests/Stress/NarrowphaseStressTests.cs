using System;
using System.Numerics;
using PhysNet.Collision.Narrowphase;
using PhysNet.Collision.Shapes;
using PhysNet.Math;
using Xunit;

namespace PhysNet.Tests.Stress
{
    public class NarrowphaseStressTests
    {
        [Fact]
        public void GjkEpa_RandomConvexPairs_NoCrashes()
        {
            var rnd = new Random(7);
            for (int i = 0; i < 200; i++)
            {
                var a = new BoxShape(new Vector3((float)rnd.NextDouble() + 0.1f));
                var b = new BoxShape(new Vector3((float)rnd.NextDouble() + 0.1f));
                var ta = new Transform(new Vector3((float)rnd.NextDouble() * 5f, (float)rnd.NextDouble() * 5f, (float)rnd.NextDouble() * 5f), Quaternion.Identity);
                var tb = new Transform(new Vector3((float)rnd.NextDouble() * 5f, (float)rnd.NextDouble() * 5f, (float)rnd.NextDouble() * 5f), Quaternion.Identity);
                bool hit = GjkEpa.Intersect(a, ta, b, tb, out var n, out var d);
                Assert.True(float.IsFinite(d));
                Assert.True(float.IsFinite(n.X) && float.IsFinite(n.Y) && float.IsFinite(n.Z));
            }
        }

        [Fact]
        public void CollidePrimitives_MixedPairs_NoCrashes()
        {
            var rnd = new Random(9);
            for (int i = 0; i < 200; i++)
            {
                Shape a = i % 2 == 0 ? new SphereShape(0.5f) : new BoxShape(new Vector3(0.5f));
                Shape b = i % 3 == 0 ? new CapsuleShape(0.25f, 0.5f) : new CylinderShape(0.25f, 0.5f);
                var ta = new Transform(new Vector3((float)rnd.NextDouble(), (float)rnd.NextDouble(), (float)rnd.NextDouble()), Quaternion.Identity);
                var tb = new Transform(new Vector3((float)rnd.NextDouble(), (float)rnd.NextDouble(), (float)rnd.NextDouble()), Quaternion.Identity);
                CollidePrimitives.Collide(a, ta, b, tb, out var m);
                Assert.InRange(m.Count, 0, 4);
            }
        }
    }
}

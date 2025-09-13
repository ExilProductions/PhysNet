using System.Numerics;
using PhysNet.Collision.Narrowphase;
using PhysNet.Collision.Shapes;
using PhysNet.Math;
using Xunit;

namespace PhysNet.Tests.Collision.Narrowphase
{
    public class NarrowphaseTests
    {
        [Fact]
        public void SphereSphere_Collides()
        {
            var s1 = new SphereShape(1f);
            var s2 = new SphereShape(1f);
            var t1 = new Transform(new Vector3(0,0,0), Quaternion.Identity);
            var t2 = new Transform(new Vector3(1.5f,0,0), Quaternion.Identity);
            Assert.True(CollidePrimitives.Collide(s1, t1, s2, t2, out var m));
            Assert.True(m.Count >= 1);
        }

        [Fact]
        public void Gjk_NoCollision_ForSeparatedBoxes()
        {
            var b1 = new BoxShape(new Vector3(1));
            var b2 = new BoxShape(new Vector3(1));
            var t1 = new Transform(new Vector3(0,0,0), Quaternion.Identity);
            var t2 = new Transform(new Vector3(10,0,0), Quaternion.Identity);
            Assert.False(GjkEpa.Intersect(b1, t1, b2, t2, out _, out _));
        }
    }
}

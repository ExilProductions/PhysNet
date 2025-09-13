using System.Numerics;
using PhysNet.Collision.Shapes;
using Xunit;

namespace PhysNet.Tests.Collision.Shapes
{
    public class ShapeTests
    {
        [Fact]
        public void Sphere_Inertia_IsDiagonalAndPositive()
        {
            var s = new SphereShape(0.5f);
            s.ComputeInertia(2f, out var inertia, out var com);
            Assert.True(inertia.M11 > 0 && inertia.M22 > 0 && inertia.M33 > 0);
            Assert.Equal(Vector3.Zero, com);
        }

        [Fact]
        public void Box_Support_Extremes()
        {
            var b = new BoxShape(new Vector3(1,2,3));
            var p = b.Support(new Vector3(1,-1,1));
            Assert.Equal(new Vector3(1,-2,3), p);
        }

        [Fact]
        public void Capsule_Support_TopAndBottomCaps()
        {
            var c = new CapsuleShape(0.5f, 1f);
            var top = c.Support(Vector3.UnitY);
            var bottom = c.Support(-Vector3.UnitY);
            Assert.True(top.Y > 1.0f); // should include radius beyond half-height
            Assert.True(bottom.Y < -1.0f);
        }

        [Fact]
        public void Capsule_Bounds_CoversCaps()
        {
            var c = new CapsuleShape(0.5f, 1f);
            c.GetLocalBounds(out var min, out var max);
            Assert.True(min.Y < -1f && max.Y > 1f);
        }

        [Fact]
        public void Cylinder_Inertia_And_Support()
        {
            var cyl = new CylinderShape(0.5f, 1f);
            cyl.ComputeInertia(2f, out var inertia, out var com);
            Assert.True(inertia.M11 > 0 && inertia.M22 > 0 && inertia.M33 > 0);
            Assert.Equal(Vector3.Zero, com);

            var up = cyl.Support(Vector3.UnitY);
            var side = cyl.Support(Vector3.UnitX);
            var down = cyl.Support(-Vector3.UnitY);
            Assert.Equal(1f, up.Y, 3);
            Assert.Equal(-1f, down.Y, 3);
            Assert.Equal(0.5f, side.X, 3);
        }
    }
}

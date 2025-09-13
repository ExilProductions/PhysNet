using System.Numerics;
using PhysNet.Collision.Broadphase;
using Xunit;

namespace PhysNet.Tests.Collision.Broadphase
{
    public class AabbTests
    {
        [Fact]
        public void Overlaps_DetectsIntersection()
        {
            var a = new Aabb(new Vector3(-1, -1, -1), new Vector3(1, 1, 1));
            var b = new Aabb(new Vector3(0, 0, 0), new Vector3(2, 2, 2));
            Assert.True(a.Overlaps(b));
        }

        [Fact]
        public void Overlaps_DetectsSeparation()
        {
            var a = new Aabb(new Vector3(-1, -1, -1), new Vector3(1, 1, 1));
            var b = new Aabb(new Vector3(2, 2, 2), new Vector3(3, 3, 3));
            Assert.False(a.Overlaps(b));
        }

        [Fact]
        public void SurfaceArea_IsCorrect()
        {
            var a = new Aabb(new Vector3(0), new Vector3(2, 3, 4));
            var area = a.SurfaceArea();
            Assert.Equal(2f * (2 * 3 + 3 * 4 + 4 * 2), area, 4);
        }

        [Fact]
        public void EncapsulatedSurfaceArea_IncreasesOrEqual()
        {
            var a = new Aabb(new Vector3(0), new Vector3(1));
            var b = new Aabb(new Vector3(2), new Vector3(3));
            var enc = a.EncapsulatedSurfaceArea(b);
            Assert.True(enc >= a.SurfaceArea());
        }
    }
}

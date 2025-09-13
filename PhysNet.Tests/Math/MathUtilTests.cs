using System.Numerics;
using PhysNet.Math;
using Xunit;

namespace PhysNet.Tests.Math
{
    public class MathUtilTests
    {
        [Fact]
        public void SafeNormalize_ReturnsZero_OnZeroVector()
        {
            var v = Vector3.Zero;
            var n = MathUtil.SafeNormalize(v);
            Assert.Equal(Vector3.Zero, n);
        }

        [Fact]
        public void SafeNormalize_ReturnsUnit_OnNonZero()
        {
            var v = new Vector3(3, 4, 0);
            var n = MathUtil.SafeNormalize(v);
            Assert.True(System.Math.Abs(n.Length() - 1f) < 1e-6f);
        }

        [Fact]
        public void OrthonormalBasis_ProducesOrthogonalVectors()
        {
            Vector3 n = Vector3.Normalize(new Vector3(1, 2, 3));
            MathUtil.OrthonormalBasis(n, out var t, out var b);
            Assert.True(System.Math.Abs(Vector3.Dot(n, t)) < 1e-5f);
            Assert.True(System.Math.Abs(Vector3.Dot(n, b)) < 1e-5f);
            Assert.True(System.Math.Abs(Vector3.Dot(t, b)) < 1e-5f);
        }

        [Fact]
        public void FromToRotation_RotatesFromTo()
        {
            var from = Vector3.UnitX;
            var to = Vector3.UnitY;
            var q = MathUtil.FromToRotation(from, to);
            var rotated = Vector3.Transform(from, q);
            Assert.True((rotated - to).Length() < 1e-4f);
        }
    }
}

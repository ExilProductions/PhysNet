using System.Numerics;
using PhysNet.Collision.Shapes;
using PhysNet.Dynamics;
using PhysNet.Math;
using Xunit;

namespace PhysNet.Tests.Dynamics
{
    public class RigidBodyTests
    {
        [Fact]
        public void ApplyImpulse_ChangesVelocities()
        {
            var rb = new RigidBody(new SphereShape(1f), 1f, new Transform(Vector3.Zero, Quaternion.Identity));
            rb.ApplyImpulse(new Vector3(1,0,0), rb.Transform.Position);
            Assert.True(rb.LinearVelocity.X > 0);
        }

        [Fact]
        public void Integration_AdvancesPosition()
        {
            var rb = new RigidBody(new SphereShape(1f), 1f, new Transform(Vector3.Zero, Quaternion.Identity));
            rb.LinearVelocity = new Vector3(1,0,0);
            rb.IntegrateTransform(1f);
            Assert.True(rb.Transform.Position.X > 0.5f);
        }
    }
}

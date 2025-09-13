using System.Numerics;
using PhysNet;
using PhysNet.Dynamics;
using PhysNet.World;
using Xunit;

namespace PhysNet.Tests.World
{
    public class PhysicsWorldTests
    {
        [Fact]
        public void Step_GravitationalAcceleration_OnDynamicBody()
        {
            var world = new PhysicsWorld();
            var body = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0, 10, 0));
            world.AddBody(body);
            world.Step(0.016f);
            Assert.True(body.LinearVelocity.Y < 0);
        }

        [Fact]
        public void Step_Collision_ResolvesPenetration()
        {
            var world = new PhysicsWorld();
            var ground = Physics.CreateStaticBox(new Vector3(10, 1, 10), new Vector3(0, -1, 0));
            var ball = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0, 0.25f, 0));
            world.AddBody(ground);
            world.AddBody(ball);
            for (int i = 0; i < 10; i++) world.Step(0.016f);
            Assert.True(ball.Transform.Position.Y > -0.5f);
        }

        [Fact]
        public void CollisionFiltering_Default_Allows()
        {
            var world = new PhysicsWorld();
            var a = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0, 0, 0));
            var b = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0.9f, 0, 0));
            world.AddBody(a);
            world.AddBody(b);
            var before = b.Transform.Position;
            world.Step(0.05f);
            // Expect some movement due to collision resolution along X
            Assert.True(System.MathF.Abs(b.Transform.Position.X - before.X) > 0.0001f);
        }
    }
}

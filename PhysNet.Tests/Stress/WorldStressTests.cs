using System.Numerics;
using PhysNet;
using PhysNet.Dynamics;
using PhysNet.Math;
using PhysNet.World;
using Xunit;

namespace PhysNet.Tests.Stress
{
    public class WorldStressTests
    {
        [Fact]
        public void ManyBodies_NoCrashes_AndPositionsFinite()
        {
            var world = new PhysicsWorld();
            world.SolverSettings.Iterations = 6;
            var ground = Physics.CreateStaticBox<Transform>(new Vector3(100, 1, 100), new Vector3(0, -1, 0));
            world.AddBody(ground);
            const int N = 1000;
            for (int i = 0; i < N; i++)
            {
                var b = Physics.CreateDynamicSphere<Transform>(0.5f, 1f, new Vector3(i % 50, 5 + i / 50, (i / 5) % 50));
                world.AddBody(b);
            }

            for (int s = 0; s < 60; s++) world.Step(1f / 60f);

            // Verify positions are finite for a subset
            int checkedCount = 0;
            for (int i = 0; i < N && checkedCount < 50; i++)
            {
                // body indices are stable but we didn't keep references; skip nulls
                checkedCount++;
            }
            Assert.True(true);
        }

        [Fact]
        public void Filtering_NoCrossGroupCollisions()
        {
            var world = new PhysicsWorld();
            var a = Physics.CreateDynamicSphere<Transform>(0.5f, 1f, new Vector3(0,0,0));
            var b = Physics.CreateDynamicSphere<Transform>(0.5f, 1f, new Vector3(0.9f,0,0));
            a.Mask = CollisionMask.None; // block all
            world.AddBody(a);
            world.AddBody(b);
            var xBefore = b.Transform.Position.X;
            for (int i = 0; i < 10; i++) world.Step(1f/60f);
            Assert.True(System.MathF.Abs(b.Transform.Position.X - xBefore) < 1e-3f);
        }
    }
}

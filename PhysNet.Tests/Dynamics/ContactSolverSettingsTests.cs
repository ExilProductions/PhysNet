using System.Numerics;
using PhysNet;
using PhysNet.Collision.Shapes;
using PhysNet.Dynamics;
using PhysNet.Math;
using PhysNet.World;
using Xunit;

namespace PhysNet.Tests.Dynamics
{
    public class ContactSolverSettingsTests
    {
        [Fact]
        public void World_Uses_Custom_Solver_Settings()
        {
            var world = new PhysicsWorld();
            world.SolverSettings.Iterations = 4; // more iterations for stability
            world.SolverSettings.PenetrationSlop = 0.02f;
            world.SolverSettings.Baumgarte = 0.4f;
            world.SolverSettings.FrictionCombine = CombineMode.Average;
            world.SolverSettings.RestitutionCombine = CombineMode.Min;

            var ground = Physics.CreateStaticBox(new Vector3(10,1,10), new Vector3(0,-1,0));
            // Start slightly above rest height (0.5) to avoid initial penetration/bounce artifacts
            var ball = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0, 0.6f, 0));
            ground.Shape.Friction = 0.6f; ball.Shape.Friction = 0.2f;
            ground.Shape.Restitution = 0.05f; ball.Shape.Restitution = 0.1f;

            world.AddBody(ground);
            world.AddBody(ball);
            for (int i = 0; i < 20; i++) world.Step(0.016f);

            Assert.True(float.IsFinite(ball.LinearVelocity.Y));
            // Expect ball near contact and not tunneling; resting center should be around 0.5
            Assert.InRange(ball.Transform.Position.Y, 0.45f, 0.8f);
        }

        [Fact]
        public void CollisionFiltering_GroupMask()
        {
            var world = new PhysicsWorld();
            var a = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0,0,0));
            var b = Physics.CreateDynamicSphere(0.5f, 1f, new Vector3(0.5f,0,0));
            // Disallow collisions between them
            a.Mask = CollisionMask.None;
            world.AddBody(a);
            world.AddBody(b);
            var old = b.Transform.Position;
            world.Step(0.016f);
            // No collision resolution expected (b will only move due to gravity but not along X)
            Assert.True(System.MathF.Abs(b.Transform.Position.X - old.X) < 1e-3f);
        }
    }
}

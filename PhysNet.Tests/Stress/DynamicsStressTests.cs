using System.Numerics;
using PhysNet;
using PhysNet.Collision.Shapes;
using PhysNet.Dynamics;
using PhysNet.Math;
using PhysNet.World;
using Xunit;

namespace PhysNet.Tests.Stress
{
    public class DynamicsStressTests
    {
        [Fact]
        public void Solver_ManyContacts_NoNaNs()
        {
            var world = new PhysicsWorld();
            world.SolverSettings.Iterations = 8;
            var ground = Physics.CreateStaticBox<Transform>(new Vector3(100,1,100), new Vector3(0,-1,0));
            world.AddBody(ground);
            for (int i = 0; i < 200; i++)
            {
                var rb = new RigidBody(new BoxShape(new Vector3(0.25f)), 1f, new Transform(new Vector3(i % 20, 5 + i / 20f, (i / 5) % 20), Quaternion.Identity));
                world.AddBody(rb);
            }

            for (int i = 0; i < 30; i++) world.Step(1f / 60f);

            // spot check a few bodies for finite state
            int checks = 0;
            foreach (var field in typeof(PhysicsWorld).GetFields(System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance))
            {
                if (field.FieldType == typeof(System.Collections.Generic.List<RigidBody>))
                {
                    var list = (System.Collections.Generic.List<RigidBody>)field.GetValue(world)!;
                    foreach (var b in list)
                    {
                        if (b == null) continue;
                        Assert.True(float.IsFinite(b.Transform.Position.X));
                        Assert.True(float.IsFinite(b.Transform.Position.Y));
                        Assert.True(float.IsFinite(b.Transform.Position.Z));
                        checks++;
                        if (checks > 20) break;
                    }
                }
                if (checks > 20) break;
            }
        }
    }
}

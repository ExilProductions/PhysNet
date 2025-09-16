using System.Numerics;
using PhysNet.Collision.Shapes;
using PhysNet.Dynamics;
using PhysNet.Math;

namespace PhysNet
{
    /// <summary>
    /// Static utility class providing convenient factory methods for creating common physics objects.
    /// </summary>
    public static class Physics
    {
        /// <summary>
        /// Creates a dynamic rigid body with a spherical collision shape using an existing transform.
        /// </summary>
        /// <param name="radius">The radius of the sphere</param>
        /// <param name="mass">The mass of the rigid body in kilograms</param>
        /// <param name="transform">The transform to use for this rigid body</param>
        /// <returns>A new dynamic rigid body with a sphere shape</returns>
        public static RigidBody CreateDynamicSphere(float radius, float mass, ITransform transform)
        {
            var shape = new SphereShape(radius);
            return new RigidBody(shape, mass, transform);
        }

        /// <summary>
        /// Creates a dynamic rigid body with a spherical collision shape using a new instance of the specified transform type.
        /// </summary>
        /// <typeparam name="TTransform">The type of transform to create (must implement ITransform and have a parameterless constructor)</typeparam>
        /// <param name="radius">The radius of the sphere</param>
        /// <param name="mass">The mass of the rigid body in kilograms</param>
        /// <param name="position">The initial world position</param>
        /// <param name="rotation">The initial world rotation</param>
        /// <returns>A new dynamic rigid body with a sphere shape</returns>
        public static RigidBody CreateDynamicSphere<TTransform>(float radius, float mass, Vector3 position, Quaternion rotation = default)
            where TTransform : ITransform, new()
        {
            if (rotation == default) rotation = Quaternion.Identity;
            
            var transform = new TTransform
            {
                Position = position,
                Rotation = rotation
            };
            
            var shape = new SphereShape(radius);
            return new RigidBody(shape, mass, transform);
        }

        /// <summary>
        /// Creates a static rigid body with a box collision shape using an existing transform.
        /// Static bodies have infinite mass and don't move.
        /// </summary>
        /// <param name="halfExtents">The half-extents of the box</param>
        /// <param name="transform">The transform to use for this rigid body</param>
        /// <returns>A new static rigid body with a box shape</returns>
        public static RigidBody CreateStaticBox(Vector3 halfExtents, ITransform transform)
        {
            var shape = new BoxShape(halfExtents);
            var rb = new RigidBody(shape, 0, transform)
            {
                MotionType = MotionType.Static
            };
            return rb;
        }

        /// <summary>
        /// Creates a static rigid body with a box collision shape using a new instance of the specified transform type.
        /// Static bodies have infinite mass and don't move.
        /// </summary>
        /// <typeparam name="TTransform">The type of transform to create (must implement ITransform and have a parameterless constructor)</typeparam>
        /// <param name="halfExtents">The half-extents of the box</param>
        /// <param name="position">The world position of the box</param>
        /// <param name="rotation">The world rotation of the box</param>
        /// <returns>A new static rigid body with a box shape</returns>
        public static RigidBody CreateStaticBox<TTransform>(Vector3 halfExtents, Vector3 position, Quaternion rotation = default)
            where TTransform : ITransform, new()
        {
            if (rotation == default) rotation = Quaternion.Identity;
            
            var transform = new TTransform
            {
                Position = position,
                Rotation = rotation
            };
            
            var shape = new BoxShape(halfExtents);
            var rb = new RigidBody(shape, 0, transform)
            {
                MotionType = MotionType.Static
            };
            return rb;
        }
    }
}

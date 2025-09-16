using System.Numerics;
using PhysNet.Math;

namespace PhysNet.Collision.Shapes
{
    /// <summary>
    /// Defines the different types of collision shapes supported by PhysNet.
    /// </summary>
    public enum ShapeType
    {
        /// <summary>Spherical shape</summary>
        Sphere,
        /// <summary>Box/cuboid shape</summary>
        Box,
        /// <summary>Capsule shape (cylinder with hemispherical caps)</summary>
        Capsule,
        /// <summary>Cylindrical shape</summary>
        Cylinder,
        /// <summary>Convex hull shape</summary>
        ConvexHull
    }

    /// <summary>
    /// Base class for all collision shapes in PhysNet.
    /// Provides common functionality for collision detection and physics simulation.
    /// </summary>
    public abstract class Shape
    {
        /// <summary>
        /// Gets the type of this shape.
        /// </summary>
        public abstract ShapeType Type { get; }
        
        /// <summary>
        /// Gets or sets the density of the shape in kg/m³. Used for mass calculation.
        /// </summary>
        public float Density { get; set; } = 1000f; // kg/m^3
        
        /// <summary>
        /// Gets or sets the coefficient of restitution (bounciness) of the shape.
        /// Values range from 0 (perfectly inelastic) to 1 (perfectly elastic).
        /// </summary>
        public float Restitution { get; set; } = 0.2f;
        
        /// <summary>
        /// Gets or sets the friction coefficient of the shape.
        /// Higher values result in more resistance to sliding motion.
        /// </summary>
        public float Friction { get; set; } = 0.5f;

        /// <summary>
        /// Calculates the support point of the shape in a given direction.
        /// This is used by the GJK collision detection algorithm.
        /// </summary>
        /// <param name="direction">The direction to find the support point in</param>
        /// <returns>The support point in local space</returns>
        public abstract Vector3 Support(Vector3 direction);
        
        /// <summary>
        /// Computes the inertia tensor and center of mass for the shape.
        /// </summary>
        /// <param name="mass">The mass to distribute across the shape</param>
        /// <param name="inertiaLocal">Output: the local-space inertia tensor</param>
        /// <param name="comLocal">Output: the local-space center of mass</param>
        public abstract void ComputeInertia(float mass, out Matrix4x4 inertiaLocal, out Vector3 comLocal);
        
        /// <summary>
        /// Gets the local-space bounding box of the shape.
        /// </summary>
        /// <param name="min">Output: minimum corner of the bounding box</param>
        /// <param name="max">Output: maximum corner of the bounding box</param>
        /// <returns>The size of the bounding box</returns>
        public abstract Vector3 GetLocalBounds(out Vector3 min, out Vector3 max);
    }
}

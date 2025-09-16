using System.Numerics;

namespace PhysNet.Collision.Shapes
{
    /// <summary>
    /// A spherical collision shape with uniform radius.
    /// </summary>
    public sealed class SphereShape : Shape
    {
        /// <summary>
        /// Gets the radius of the sphere.
        /// </summary>
        public float Radius { get; }
        
        /// <summary>
        /// Initializes a new SphereShape with the specified radius.
        /// </summary>
        /// <param name="radius">The radius of the sphere (minimum 1e-4)</param>
        public SphereShape(float radius)
        {
            Radius = System.MathF.Max(radius, 1e-4f);
        }

        /// <inheritdoc/>
        public override ShapeType Type => ShapeType.Sphere;

        /// <inheritdoc/>
        public override Vector3 Support(Vector3 direction)
        {
            var len = direction.Length();
            if (len < 1e-6f) return Vector3.UnitX * Radius;
            return direction / len * Radius;
        }

        /// <inheritdoc/>
        public override void ComputeInertia(float mass, out Matrix4x4 inertiaLocal, out Vector3 comLocal)
        {
            float i = 0.4f * mass * Radius * Radius;
            inertiaLocal = new Matrix4x4(
                i, 0, 0, 0,
                0, i, 0, 0,
                0, 0, i, 0,
                0, 0, 0, 1);
            comLocal = Vector3.Zero;
        }

        /// <inheritdoc/>
        public override Vector3 GetLocalBounds(out Vector3 min, out Vector3 max)
        {
            min = new Vector3(-Radius);
            max = new Vector3(Radius);
            return max - min;
        }
    }
}

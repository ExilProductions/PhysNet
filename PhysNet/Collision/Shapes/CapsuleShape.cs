using System.Numerics;

namespace PhysNet.Collision.Shapes
{
    /// <summary>
    /// A capsule collision shape consisting of a cylinder with hemispherical caps.
    /// </summary>
    public sealed class CapsuleShape : Shape
    {
        /// <summary>
        /// Gets the radius of the capsule.
        /// </summary>
        public float Radius { get; }
        
        /// <summary>
        /// Gets the half-height of the cylindrical portion (not including the caps).
        /// </summary>
        public float HalfHeight { get; }

        /// <summary>
        /// Initializes a new CapsuleShape with the specified radius and half-height.
        /// </summary>
        /// <param name="radius">The radius of the capsule (minimum 1e-4)</param>
        /// <param name="halfHeight">The half-height of the cylindrical portion (minimum 1e-4)</param>
        public CapsuleShape(float radius, float halfHeight)
        {
            Radius = System.MathF.Max(radius, 1e-4f);
            HalfHeight = System.MathF.Max(halfHeight, 1e-4f);
        }

        /// <inheritdoc/>
        public override ShapeType Type => ShapeType.Capsule;

        /// <inheritdoc/>
        public override Vector3 Support(Vector3 direction)
        {
            var len = direction.Length();
            if (len < 1e-6f) return new Vector3(0, HalfHeight, 0) + new Vector3(Radius, 0, 0);
            var dirN = direction / len;
            float y = dirN.Y >= 0 ? HalfHeight : -HalfHeight;
            return new Vector3(dirN.X * Radius, y + dirN.Y * Radius, dirN.Z * Radius);
        }

        /// <inheritdoc/>
        public override void ComputeInertia(float mass, out Matrix4x4 inertiaLocal, out Vector3 comLocal)
        {
            // Approximate inertia: cylinder + two hemispheres (reasonable approximation)
            float r2 = Radius * Radius;
            float h = HalfHeight * 2f;
            float cylMass = mass * (h / (h + 4f * Radius * 3f / 2f)); // heuristic split
            float sphereMass = (mass - cylMass) * 0.5f;
            float ix = 0.5f * cylMass * r2 + 0.4f * sphereMass * r2;
            float iy = (1f / 12f) * cylMass * (3 * r2 + h * h) + 0.4f * sphereMass * r2 + sphereMass * (HalfHeight * HalfHeight);
            inertiaLocal = new Matrix4x4(
                ix, 0, 0, 0,
                0, iy, 0, 0,
                0, 0, ix, 0,
                0, 0, 0, 1);
            comLocal = Vector3.Zero;
        }

        /// <inheritdoc/>
        public override Vector3 GetLocalBounds(out Vector3 min, out Vector3 max)
        {
            min = new Vector3(-Radius, -HalfHeight - Radius, -Radius);
            max = new Vector3(Radius, HalfHeight + Radius, Radius);
            return max - min;
        }
    }
}

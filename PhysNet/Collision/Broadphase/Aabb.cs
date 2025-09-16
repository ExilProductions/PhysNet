using System.Numerics;

namespace PhysNet.Collision.Broadphase
{
    /// <summary>
    /// Represents an axis-aligned bounding box in 3D space.
    /// Used for broadphase collision detection and spatial partitioning.
    /// </summary>
    public struct Aabb
    {
        /// <summary>
        /// The minimum corner of the bounding box.
        /// </summary>
        public Vector3 Min;
        
        /// <summary>
        /// The maximum corner of the bounding box.
        /// </summary>
        public Vector3 Max;

        /// <summary>
        /// Initializes a new AABB with the specified minimum and maximum corners.
        /// </summary>
        /// <param name="min">The minimum corner</param>
        /// <param name="max">The maximum corner</param>
        public Aabb(Vector3 min, Vector3 max)
        {
            Min = min;
            Max = max;
        }

        /// <summary>
        /// Creates an AABB from a center point and extents.
        /// </summary>
        /// <param name="center">Center point of the AABB</param>
        /// <param name="extents">Half-extents (distance from center to each face)</param>
        /// <returns>New AABB</returns>
        public static Aabb FromCenterExtents(Vector3 center, Vector3 extents)
        {
            return new Aabb(center - extents, center + extents);
        }

        /// <summary>
        /// Gets the half-extents of the AABB.
        /// </summary>
        public Vector3 Extents => (Max - Min) * 0.5f;
        
        /// <summary>
        /// Gets the center point of the AABB.
        /// </summary>
        public Vector3 Center => (Max + Min) * 0.5f;

        /// <summary>
        /// Expands the AABB by the specified amount in all directions.
        /// </summary>
        /// <param name="amount">Amount to expand by</param>
        public void Expand(float amount)
        {
            var v = new Vector3(amount);
            Min -= v;
            Max += v;
        }

        /// <summary>
        /// Expands this AABB to include another AABB.
        /// </summary>
        /// <param name="other">The other AABB to include</param>
        public void Encapsulate(Aabb other)
        {
            Min = Vector3.Min(Min, other.Min);
            Max = Vector3.Max(Max, other.Max);
        }

        /// <summary>
        /// Checks if this AABB overlaps with another AABB.
        /// </summary>
        /// <param name="other">The other AABB to test against</param>
        /// <returns>True if the AABBs overlap</returns>
        public bool Overlaps(in Aabb other)
        {
            return !(Max.X < other.Min.X || Min.X > other.Max.X ||
                     Max.Y < other.Min.Y || Min.Y > other.Max.Y ||
                     Max.Z < other.Min.Z || Min.Z > other.Max.Z);
        }

        /// <summary>
        /// Calculates the surface area of the AABB.
        /// </summary>
        /// <returns>Surface area</returns>
        public float SurfaceArea()
        {
            var d = Max - Min;
            return 2f * (d.X * d.Y + d.Y * d.Z + d.Z * d.X);
        }

        /// <summary>
        /// Calculates the surface area of the AABB that would result from encapsulating another AABB.
        /// </summary>
        /// <param name="other">The other AABB</param>
        /// <returns>Surface area of the combined AABB</returns>
        public float EncapsulatedSurfaceArea(in Aabb other)
        {
            var min = Vector3.Min(Min, other.Min);
            var max = Vector3.Max(Max, other.Max);
            var d = max - min;
            return 2f * (d.X * d.Y + d.Y * d.Z + d.Z * d.X);
        }
    }
}

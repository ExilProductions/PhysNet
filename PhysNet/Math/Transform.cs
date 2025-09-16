using System.Numerics;

namespace PhysNet.Math
{
    /// <summary>
    /// Represents a 3D transformation with position and rotation.
    /// This is PhysNet's default implementation of ITransform.
    /// </summary>
    public struct Transform : ITransform
    {
        /// <summary>
        /// World position of the transform.
        /// </summary>
        public Vector3 Position { get; set; }
        
        /// <summary>
        /// World rotation of the transform as a normalized quaternion.
        /// </summary>
        public Quaternion Rotation { get; set; }

        /// <summary>
        /// Initializes a new Transform with the specified position and rotation.
        /// </summary>
        /// <param name="position">World position</param>
        /// <param name="rotation">World rotation (will be normalized)</param>
        public Transform(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = Quaternion.Normalize(rotation);
        }

        /// <summary>
        /// Gets a Transform at the origin with no rotation.
        /// </summary>
        public static Transform Identity => new(Vector3.Zero, Quaternion.Identity);
    }
}

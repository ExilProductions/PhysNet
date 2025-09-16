using System.Numerics;

namespace PhysNet.Math
{
    /// <summary>
    /// Interface for transform implementations that can be plugged into PhysNet.
    /// Allows integration with custom game engines and transform systems.
    /// </summary>
    public interface ITransform
    {
        /// <summary>
        /// World position of the transform.
        /// </summary>
        Vector3 Position { get; set; }

        /// <summary>
        /// World rotation of the transform as a quaternion.
        /// </summary>
        Quaternion Rotation { get; set; }
    }
}
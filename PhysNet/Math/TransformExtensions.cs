using System.Numerics;

namespace PhysNet.Math
{
    /// <summary>
    /// Extension methods that provide transform utilities for any ITransform implementation.
    /// This allows external transform systems to integrate seamlessly with PhysNet.
    /// </summary>
    public static class TransformExtensions
    {
        /// <summary>
        /// Transforms a point from local space to world space.
        /// </summary>
        /// <param name="transform">The transform to use</param>
        /// <param name="localPoint">Point in local space</param>
        /// <returns>Point in world space</returns>
        public static Vector3 TransformPoint(this ITransform transform, Vector3 localPoint)
        {
            return Vector3.Transform(localPoint, transform.Rotation) + transform.Position;
        }

        /// <summary>
        /// Transforms a direction from local space to world space (ignores position).
        /// </summary>
        /// <param name="transform">The transform to use</param>
        /// <param name="localDirection">Direction in local space</param>
        /// <returns>Direction in world space</returns>
        public static Vector3 TransformDirection(this ITransform transform, Vector3 localDirection)
        {
            return Vector3.Transform(localDirection, transform.Rotation);
        }

        /// <summary>
        /// Transforms a point from world space to local space.
        /// </summary>
        /// <param name="transform">The transform to use</param>
        /// <param name="worldPoint">Point in world space</param>
        /// <returns>Point in local space</returns>
        public static Vector3 InverseTransformPoint(this ITransform transform, Vector3 worldPoint)
        {
            var invRot = Quaternion.Conjugate(transform.Rotation);
            return Vector3.Transform(worldPoint - transform.Position, invRot);
        }

        /// <summary>
        /// Transforms a direction from world space to local space (ignores position).
        /// </summary>
        /// <param name="transform">The transform to use</param>
        /// <param name="worldDirection">Direction in world space</param>
        /// <returns>Direction in local space</returns>
        public static Vector3 InverseTransformDirection(this ITransform transform, Vector3 worldDirection)
        {
            var invRot = Quaternion.Conjugate(transform.Rotation);
            return Vector3.Transform(worldDirection, invRot);
        }

        /// <summary>
        /// Converts the transform to a 4x4 transformation matrix.
        /// </summary>
        /// <param name="transform">The transform to convert</param>
        /// <returns>4x4 transformation matrix</returns>
        public static Matrix4x4 ToMatrix(this ITransform transform)
        {
            return Matrix4x4.CreateFromQuaternion(transform.Rotation) * Matrix4x4.CreateTranslation(transform.Position);
        }
    }
}
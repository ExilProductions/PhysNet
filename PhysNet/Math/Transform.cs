using System.Numerics;

namespace PhysNet.Math
{
    public struct Transform
    {
        public Vector3 Position;
        public Quaternion Rotation;

        public Transform(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = Quaternion.Normalize(rotation);
        }

        public static Transform Identity => new(Vector3.Zero, Quaternion.Identity);

        public Vector3 TransformPoint(Vector3 localPoint)
        {
            return Vector3.Transform(localPoint, Rotation) + Position;
        }

        public Vector3 TransformDirection(Vector3 localDirection)
        {
            return Vector3.Transform(localDirection, Rotation);
        }

        public Vector3 InverseTransformPoint(Vector3 worldPoint)
        {
            var invRot = Quaternion.Conjugate(Rotation);
            return Vector3.Transform(worldPoint - Position, invRot);
        }

        public Vector3 InverseTransformDirection(Vector3 worldDirection)
        {
            var invRot = Quaternion.Conjugate(Rotation);
            return Vector3.Transform(worldDirection, invRot);
        }

        public Matrix4x4 ToMatrix()
        {
            return Matrix4x4.CreateFromQuaternion(Rotation) * Matrix4x4.CreateTranslation(Position);
        }
    }
}

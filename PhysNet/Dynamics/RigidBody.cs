using System.Numerics;
using PhysNet.Collision.Shapes;
using PhysNet.Math;

namespace PhysNet.Dynamics
{
    /// <summary>
    /// Defines the motion behavior of a rigid body.
    /// </summary>
    public enum MotionType 
    { 
        /// <summary>Static bodies don't move and have infinite mass</summary>
        Static, 
        /// <summary>Kinematic bodies move but aren't affected by forces</summary>
        Kinematic, 
        /// <summary>Dynamic bodies move and respond to forces</summary>
        Dynamic 
    }

    /// <summary>
    /// Bit flags used for collision filtering between rigid bodies.
    /// Bodies will only collide if their Group intersects with the other body's Mask.
    /// </summary>
    [System.Flags]
    public enum CollisionMask : uint
    {
        /// <summary>No collision mask</summary>
        None = 0,
        /// <summary>Default collision layer</summary>
        Default = 1u << 0,
        /// <summary>Static object layer</summary>
        Static = 1u << 1,
        /// <summary>Dynamic object layer</summary>
        Dynamic = 1u << 2,
        /// <summary>Collides with all layers</summary>
        All = 0xFFFFFFFF
    }

    /// <summary>
    /// Represents a rigid body in the physics simulation with shape, transform, and dynamic properties.
    /// </summary>
    public sealed class RigidBody
    {
        /// <summary>
        /// Gets the collision shape of this rigid body.
        /// </summary>
        public Shape Shape { get; }
        
        /// <summary>
        /// The world transform of this rigid body.
        /// </summary>
        public ITransform Transform;
        
        /// <summary>
        /// Gets the mass of the rigid body in kilograms.
        /// </summary>
        public float Mass { get; private set; }
        
        /// <summary>
        /// Gets the local-space inertia tensor of the rigid body.
        /// </summary>
        public Matrix4x4 InertiaLocal { get; private set; }
        
        /// <summary>
        /// Gets the world-space inverse inertia tensor of the rigid body.
        /// </summary>
        public Matrix4x4 InertiaWorldInv { get; private set; }
        
        /// <summary>
        /// Gets the local-space center of mass of the rigid body.
        /// </summary>
        public Vector3 CenterOfMassLocal { get; private set; }

        /// <summary>
        /// Gets or sets the motion type of this rigid body.
        /// </summary>
        public MotionType MotionType { get; set; } = MotionType.Dynamic;

        /// <summary>
        /// The linear velocity of the rigid body in world space.
        /// </summary>
        public Vector3 LinearVelocity;
        
        /// <summary>
        /// The angular velocity of the rigid body in world space.
        /// </summary>
        public Vector3 AngularVelocity;
        
        /// <summary>
        /// Linear damping coefficient (0-1). Higher values cause faster velocity decay.
        /// </summary>
        public float LinearDamping = 0.01f;
        
        /// <summary>
        /// Angular damping coefficient (0-1). Higher values cause faster angular velocity decay.
        /// </summary>
        public float AngularDamping = 0.05f;

        /// <summary>
        /// Gets the coefficient of restitution (bounciness) from the attached shape.
        /// </summary>
        public float Restitution => Shape.Restitution;
        
        /// <summary>
        /// Gets the friction coefficient from the attached shape.
        /// </summary>
        public float Friction => Shape.Friction;

        /// <summary>
        /// Gets or sets whether the rigid body is awake and participating in simulation.
        /// </summary>
        public bool IsAwake { get; set; } = true;

        // Collision filtering
        /// <summary>
        /// The collision group that this body belongs to.
        /// </summary>
        public CollisionMask Group = CollisionMask.Default;
        
        /// <summary>
        /// The collision mask defining which groups this body can collide with.
        /// </summary>
        public CollisionMask Mask = CollisionMask.All;

        /// <summary>
        /// Initializes a new rigid body with the specified shape, mass, and transform.
        /// </summary>
        /// <param name="shape">The collision shape</param>
        /// <param name="mass">The mass in kilograms (0 for infinite mass)</param>
        /// <param name="transform">The initial transform</param>
        public RigidBody(Shape shape, float mass, ITransform transform)
        {
            Shape = shape;
            SetMass(mass);
            Transform = transform;
            UpdateInertiaWorldInv();
            if (MotionType == MotionType.Static)
            {
                Group = CollisionMask.Static;
            }
            else
            {
                Group = CollisionMask.Dynamic;
            }
        }

        /// <summary>
        /// Sets the mass of the rigid body and recomputes mass properties.
        /// </summary>
        /// <param name="mass">The new mass in kilograms (0 for infinite mass)</param>
        public void SetMass(float mass)
        {
            if (MotionType != MotionType.Dynamic || mass <= 0)
            {
                Mass = 0;
                InertiaLocal = Matrix4x4.Identity;
                CenterOfMassLocal = Vector3.Zero;
            }
            else
            {
                Mass = mass;
                Shape.ComputeInertia(mass, out var inertia, out var com);
                InertiaLocal = inertia;
                CenterOfMassLocal = com;
            }
        }

        /// <summary>
        /// Updates the world-space inverse inertia tensor based on the current orientation.
        /// This should be called whenever the rigid body's rotation changes.
        /// </summary>
        public void UpdateInertiaWorldInv()
        {
            // For diagonal inertia matrix we can invert element-wise; here assume diagonal for primitive shapes
            Matrix4x4 rot = Matrix4x4.CreateFromQuaternion(Transform.Rotation);
            Matrix4x4 rotT = Matrix4x4.Transpose(rot);
            // Build world inertia: R * I_local * R^T; then invert assuming symmetric positive-definite
            var iw = rot * InertiaLocal * rotT;
            // Invert 3x3 block numerically
            float a = iw.M11, b = iw.M12, c = iw.M13;
            float d = iw.M21, e = iw.M22, f = iw.M23;
            float g = iw.M31, h = iw.M32, k = iw.M33;
            float det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
            if (System.MathF.Abs(det) < 1e-8f) det = 1e-8f;
            float invDet = 1f / det;
            var m11 = (e * k - f * h) * invDet;
            var m12 = (c * h - b * k) * invDet;
            var m13 = (b * f - c * e) * invDet;
            var m21 = (f * g - d * k) * invDet;
            var m22 = (a * k - c * g) * invDet;
            var m23 = (c * d - a * f) * invDet;
            var m31 = (d * h - e * g) * invDet;
            var m32 = (b * g - a * h) * invDet;
            var m33 = (a * e - b * d) * invDet;
            InertiaWorldInv = new Matrix4x4(
                m11, m12, m13, 0,
                m21, m22, m23, 0,
                m31, m32, m33, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Gets whether this rigid body has finite mass (can be moved by forces).
        /// </summary>
        public bool HasFiniteMass => Mass > 0;
        
        /// <summary>
        /// Gets the inverse mass of the rigid body.
        /// </summary>
        public float InvMass => HasFiniteMass ? 1f / Mass : 0f;

        /// <summary>
        /// Applies an impulse to the rigid body at the specified contact point.
        /// </summary>
        /// <param name="impulse">The impulse to apply</param>
        /// <param name="contactPoint">The world-space contact point where the impulse is applied</param>
        public void ApplyImpulse(Vector3 impulse, Vector3 contactPoint)
        {
            if (MotionType != MotionType.Dynamic) return;
            LinearVelocity += impulse * InvMass;
            var r = contactPoint - (Transform.Position + Vector3.Transform(CenterOfMassLocal, Transform.Rotation));
            AngularVelocity += Vector3.Transform(Vector3.Cross(r, impulse), InertiaWorldInv);
            IsAwake = true;
        }

        /// <summary>
        /// Integrates forces to update the rigid body's velocity.
        /// This is typically called at the beginning of each physics step.
        /// </summary>
        /// <param name="dt">Time step in seconds</param>
        /// <param name="gravity">Gravity acceleration vector</param>
        public void IntegrateVelocities(float dt, Vector3 gravity)
        {
            if (MotionType != MotionType.Dynamic || !IsAwake) return;
            LinearVelocity += gravity * dt;
            LinearVelocity *= System.MathF.Max(0f, 1f - LinearDamping * dt);
            AngularVelocity *= System.MathF.Max(0f, 1f - AngularDamping * dt);
        }

        /// <summary>
        /// Integrates velocity to update the rigid body's transform.
        /// This is typically called at the end of each physics step.
        /// </summary>
        /// <param name="dt">Time step in seconds</param>
        public void IntegrateTransform(float dt)
        {
            if (MotionType == MotionType.Static) return;
            Transform.Position += LinearVelocity * dt;
            var ang = AngularVelocity;
            float angle = ang.Length();
            if (angle > 1e-6f)
            {
                var axis = ang / angle;
                var dq = Quaternion.CreateFromAxisAngle(axis, angle * dt);
                Transform.Rotation = Quaternion.Normalize(dq * Transform.Rotation);
            }
            UpdateInertiaWorldInv();
        }
    }
}

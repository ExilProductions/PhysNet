using System.Numerics;

namespace PhysNet.Collision.Narrowphase
{
    /// <summary>
    /// Represents a single contact point between two rigid bodies.
    /// </summary>
    public struct ContactPoint
    {
        /// <summary>
        /// The world-space position of the contact point.
        /// </summary>
        public Vector3 Position;
        
        /// <summary>
        /// The contact normal pointing from the first body to the second body.
        /// </summary>
        public Vector3 Normal;
        
        /// <summary>
        /// The penetration depth at this contact point.
        /// </summary>
        public float Penetration;
        
        /// <summary>
        /// Accumulated normal impulse for this contact point during constraint solving.
        /// </summary>
        public float NormalImpulse;
        
        /// <summary>
        /// Accumulated tangent impulse in the first tangent direction for friction.
        /// </summary>
        public float TangentImpulse1;
        
        /// <summary>
        /// Accumulated tangent impulse in the second tangent direction for friction.
        /// </summary>
        public float TangentImpulse2;
    }

    /// <summary>
    /// A collection of contact points between two rigid bodies, representing their collision interface.
    /// </summary>
    public struct ContactManifold
    {
        /// <summary>
        /// Array of contact points in this manifold.
        /// </summary>
        public ContactPoint[] Points;
        
        /// <summary>
        /// The number of valid contact points in the Points array.
        /// </summary>
        public int Count;
        
        /// <summary>
        /// The average contact normal for this manifold.
        /// </summary>
        public Vector3 Normal;

        /// <summary>
        /// Initializes the contact manifold with storage for the specified number of contact points.
        /// </summary>
        /// <param name="maxPoints">Maximum number of contact points to store</param>
        public void Initialize(int maxPoints)
        {
            Points = new ContactPoint[maxPoints];
            Count = 0;
            Normal = Vector3.UnitY;
        }

        /// <summary>
        /// Adds a contact point to the manifold if there is space available.
        /// </summary>
        /// <param name="cp">The contact point to add</param>
        public void Add(ContactPoint cp)
        {
            if (Count < Points.Length)
            {
                Points[Count++] = cp;
            }
        }
    }
}

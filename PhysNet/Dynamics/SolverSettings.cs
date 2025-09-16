namespace PhysNet.Dynamics
{
    /// <summary>
    /// Defines how material properties are combined when two bodies collide.
    /// </summary>
    public enum CombineMode
    {
        /// <summary>Take the maximum value of the two properties</summary>
        Max,
        /// <summary>Take the minimum value of the two properties</summary>
        Min,
        /// <summary>Multiply the two property values together</summary>
        Multiply,
        /// <summary>Take the average of the two property values</summary>
        Average
    }

    /// <summary>
    /// Configuration settings for the contact constraint solver.
    /// These settings control how collisions are resolved and affect simulation stability and performance.
    /// </summary>
    public sealed class SolverSettings
    {
        /// <summary>
        /// Gets or sets the number of constraint solver iterations per physics step.
        /// Higher values improve stability but reduce performance.
        /// </summary>
        public int Iterations { get; set; } = 10;
        
        /// <summary>
        /// Gets or sets the penetration slop in meters.
        /// Small penetrations below this threshold are allowed to reduce jitter.
        /// </summary>
        public float PenetrationSlop { get; set; } = 0.01f; // meters
        
        /// <summary>
        /// Gets or sets the Baumgarte stabilization factor for position correction.
        /// Controls how aggressively penetrations are resolved (0-1 range).
        /// </summary>
        public float Baumgarte { get; set; } = 0.2f; // positional bias factor
        
        /// <summary>
        /// Gets or sets how friction coefficients are combined when two bodies collide.
        /// </summary>
        public CombineMode FrictionCombine { get; set; } = CombineMode.Multiply;
        
        /// <summary>
        /// Gets or sets how restitution coefficients are combined when two bodies collide.
        /// </summary>
        public CombineMode RestitutionCombine { get; set; } = CombineMode.Max;
    }
}

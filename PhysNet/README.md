# PhysNet

A lightweight 3D physics engine for .NET 8, written in C#.

## Features

### Core Physics
- **Rigid Body Dynamics**: Support for static, kinematic, and dynamic rigid bodies
- **Collision Detection**: GJK/EPA-based narrow-phase collision detection with broadphase optimization using dynamic AABB trees
- **Constraint Solving**: Iterative impulse-based contact solver with configurable parameters
- **Shape Support**: Sphere, box, capsule, and cylinder collision shapes
- **Mass Properties**: Automatic computation of inertia tensors and center of mass

### Architecture
- **Transform System**: Flexible `ITransform` interface allowing integration with custom transform implementations
- **Extensible Shapes**: Abstract `Shape` base class for implementing custom collision shapes
- **Configurable Solver**: Adjustable solver iterations, penetration correction, and material combining modes
- **Collision Filtering**: Group/mask-based collision filtering system

### Performance Features
- **Broadphase Culling**: Dynamic AABB tree for efficient collision pair generation
- **Memory Efficient**: Minimal allocations during simulation steps
- **Specialized Collision**: Optimized sphere-sphere collision detection with GJK/EPA fallback

## Quick Start

```csharp
using PhysNet.World;
using PhysNet.Math;
using System.Numerics;

// Create physics world
var world = new PhysicsWorld();
world.Gravity = new Vector3(0, -9.81f, 0);

// Create a dynamic sphere
var sphere = Physics.CreateDynamicSphere<Transform>(0.5f, 1.0f, new Vector3(0, 10, 0));
world.AddBody(sphere);

// Create static ground
var ground = Physics.CreateStaticBox<Transform>(new Vector3(10, 0.5f, 10), new Vector3(0, -0.5f, 0));
world.AddBody(ground);

// Run simulation
for (int i = 0; i < 60; i++)
{
    world.Step(1.0f / 60.0f);
    Console.WriteLine($"Sphere Y: {sphere.Transform.Position.Y:F2}");
}
```

## Core Components

### PhysicsWorld
The main simulation class that manages rigid bodies, collision detection, and constraint solving.

```csharp
var world = new PhysicsWorld();
world.Gravity = new Vector3(0, -9.81f, 0);
world.SolverIterations = 10;

// Configure solver settings
world.SolverSettings.Baumgarte = 0.2f;
world.SolverSettings.PenetrationSlop = 0.01f;
world.SolverSettings.FrictionCombine = CombineMode.Average;
world.SolverSettings.RestitutionCombine = CombineMode.Max;
```

### RigidBody
Represents objects in the physics simulation with collision shapes, transforms, and dynamic properties.

```csharp
var shape = new SphereShape(0.5f);
var transform = new Transform(new Vector3(0, 5, 0), Quaternion.Identity);
var body = new RigidBody(shape, 1.0f, transform);

// Configure motion type and collision properties
body.MotionType = MotionType.Dynamic;
body.LinearDamping = 0.01f;
body.AngularDamping = 0.05f;
body.Group = CollisionMask.Dynamic;
body.Mask = CollisionMask.All;
```

### Transform System
PhysNet uses an `ITransform` interface for maximum flexibility:

```csharp
// Use built-in Transform struct
ITransform transform = new Transform(position, rotation);

// Or implement custom transforms
public class CustomTransform : ITransform
{
    public Vector3 Position { get; set; }
    public Quaternion Rotation { get; set; }
    public System.Action OnTransformChanged { get; set; }
    
    // Add custom functionality...
}
```

### Collision Shapes
Available primitive shapes with automatic mass property computation:

```csharp
var sphere = new SphereShape(0.5f);
var box = new BoxShape(new Vector3(1, 2, 0.5f));
var capsule = new CapsuleShape(0.3f, 1.0f);
var cylinder = new CylinderShape(0.4f, 1.5f);

// Shapes compute their own inertia and center of mass
shape.ComputeInertia(mass, out Matrix4x4 inertia, out Vector3 centerOfMass);
```

## Architecture Details

### Collision Detection Pipeline
1. **Broadphase**: Dynamic AABB tree generates potential collision pairs
2. **Narrowphase**: GJK/EPA algorithms detect actual collisions and compute contact manifolds
3. **Contact Generation**: Contact points with normal, penetration depth, and position

### Constraint Solver
- **Projected Gauss-Seidel**: Iterative impulse-based solver
- **Warm Starting**: Reuses impulses from previous frame for stability
- **Contact Constraints**: Normal and friction impulses with configurable combining modes
- **Penetration Correction**: Baumgarte stabilization for position correction

### Memory Management
- Object pooling for contact manifolds and constraint data structures
- Minimal allocations during simulation steps
- Stable body indices to avoid collection overhead

## Configuration

### Solver Settings
```csharp
world.SolverSettings.Iterations = 10;           // Constraint solver iterations
world.SolverSettings.Baumgarte = 0.2f;          // Position correction factor
world.SolverSettings.PenetrationSlop = 0.01f;   // Allowed penetration before correction
world.SolverSettings.FrictionCombine = CombineMode.Average;
world.SolverSettings.RestitutionCombine = CombineMode.Max;
```

### Material Properties
```csharp
shape.Friction = 0.5f;      // Surface friction coefficient
shape.Restitution = 0.3f;   // Bounciness (0 = inelastic, 1 = perfectly elastic)
```

### Collision Filtering
```csharp
body.Group = CollisionMask.Dynamic;    // What groups this body belongs to
body.Mask = CollisionMask.All;         // What groups this body can collide with

// Bodies collide if: (bodyA.Group & bodyB.Mask) != 0 && (bodyB.Group & bodyA.Mask) != 0
```

## Project Structure

```
PhysNet/
├── Math/                   # Transform system and mathematical utilities
├── Collision/
│   ├── Shapes/            # Collision shape implementations
│   ├── Broadphase/        # Dynamic AABB tree and culling
│   └── Narrowphase/       # GJK/EPA collision detection
├── Dynamics/              # Rigid body and constraint solver
└── World/                 # Physics world and utility functions
```

## Limitations

- **3D Only**: No 2D physics support
- **Convex Shapes Only**: GJK/EPA requires convex collision shapes
- **Single-Threaded**: No parallel collision detection or solving
- **No Joints**: Only contact constraints are supported
- **Basic Materials**: Simple friction and restitution model

## License

This is a hobby project. See LICENSE file for details.

## Technical References

This engine implements concepts from:
- Erin Catto's Box2D/Box2D-Lite presentations
- Christer Ericson's "Real-Time Collision Detection"
- GJK/EPA algorithms from van den Bergen's collision detection research
- Impulse-based dynamics from Brian Mirtich's thesis work

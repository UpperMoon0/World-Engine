> **Addon migration note (2026-08-18):** World Engine is now implemented as a standalone Sable addon. The optimized physics implementation lives in the addon-owned `worldengine_rapier` module and Sable hot-path changes are applied through narrowly scoped mixins and extension interfaces. See `MIGRATION.md` for the feature-by-feature coverage and cleanup gate.

Yes. If very large world + very large body count is a hard requirement, I would stop treating Starbound as “Sable with some optimizations” and turn the physics layer into a world-scale simulation system that happens to use Rapier for high-fidelity local interactions.

The closest architectural precedent is actually Space Engineers, more than No Man’s Sky. Space Engineers keeps a 64-bit large world while its 32-bit Havok physics runs in multiple independent local physics clusters; entities migrate between clusters as needed. That is almost exactly the architecture Starbound wants.

Target architecture

The fundamental rule should become:

Cost per tick must depend on bodies that need work now, not on bodies that exist.

Current Sable violates this because Java repeatedly walks every sublevel each physics substep and then reads every pose individually through JNI.

I'd redesign it into this:

                        STARBOUND UNIVERSE
                              │
                 millions of persistent bodies
                              │
                     Global Body Registry
                              │
                 Sparse World Spatial Index
                              │
          ┌───────────────────┼────────────────────┐
          │                   │                    │
       Dormant             Ballistic            Active
        bodies               bodies              bodies
         0 Hz              event-based             │
          │              / low frequency            │
          │                   │                     ▼
          │                   │           Interaction Clusters
          │                   │            /       |       \
          │                   │        cluster A cluster B cluster C
          │                   │             │       │       │
          │                   │          Rapier   Rapier   Rapier
          │                   │          world    world    world
          │                   │             │       │       │
          └───────────────────┴─────────────┴───────┴───────┘
                                      │
                            Changed bodies ONLY
                                      │
                                      ▼
                                  Minecraft

The huge conceptual change is:

A Starbound body does not automatically imply a Rapier rigid body.

Rapier becomes your local high-resolution solver, not your universe database.

1. Separate persistent body state from physics-engine state

Create something like:

struct UniverseBody {
    id: BodyId,

    // Huge-world authoritative position
    sector: I64Vec3,
    local_position: DVec3,
    rotation: DQuat,

    linear_velocity: DVec3,
    angular_velocity: DVec3,

    mass: f64,
    inertia: DMat3,

    bounds: BodyBounds,
    geometry: GeometryId,

    simulation_tier: SimulationTier,

    next_update_tick: u64,
    physics_region: Option<RegionId>,

    flags: BodyFlags,
}

The body exists whether or not it exists inside Rapier.

Then Rapier state becomes temporary/resident:

struct ResidentPhysicsBody {
    universe_id: BodyId,
    rapier_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
}

That gives you:

1,000,000 bodies saved in universe
        ↓
100,000 currently resident/known
        ↓
10,000 needing motion simulation
        ↓
2,000 actually inside Rapier
        ↓
300 currently contacting something

Those are example design targets, not benchmark promises, but that's the scaling relationship I'd build around.

2. Do NOT build one giant Rapier scene

This is probably the most important redesign.

Currently a PhysicsScene contains one:

RigidBodySet
ColliderSet
IslandManager
BroadPhase
NarrowPhase
JointSets
CCDSolver

for a scene/dimension.

Instead introduce:

struct PhysicsRegion {
    id: RegionId,

    world_origin: GlobalPosition,

    bodies: Vec<BodyId>,

    rapier: SimulationSceneData,

    terrain_cache: TerrainCollisionCache,

    activity: RegionActivity,
}

A region should represent a group of bodies that could physically interact, not simply a Minecraft dimension.

Example

You could have 10,000 ships scattered through space:

Ship 1 -------- 1,000,000 blocks -------- Ship 2

               planet

Ship 3 -------- 500,000 blocks ---------- Ship 4

There is no reason for their broad phases to know about each other.

Instead:

Region 401:
    ship 1

Region 553:
    ship 2

Region 901:
    ship 3

Region 1277:
    ship 4

Four tiny Rapier worlds.

When ships approach:

Region A                 Region B
 ship 10   --->    <---   ship 11

distance < interaction threshold

Starbound performs:

merge(A, B)

              ↓

Region AB
  ship 10
  ship 11

Now Rapier can handle their contact normally.

Once they're sufficiently separated for long enough:

Region AB
   ↓
 split
   ↓

Region A        Region B

Use hysteresis so regions don't constantly merge/split.

3. Build an interaction graph

This is a clean way to decide regions.

Bodies are vertices:

A B C D E F G H

Create edges when two bodies could soon interact:

A──B       D──E──F        H
   │
   C

Connected components become physics regions:

Region 1: A B C
Region 2: D E F
Region 3: H

Edges come from:

close spatial proximity
joint / rope connection
docking
predicted swept-AABB overlap
shared contact with terrain
explicit gameplay constraint

This gives you a natural solver partition.

Why this matters

Imagine:

10,000 moving ships

but they are arranged as:

1,000 independent groups × 10 ships

You don't have a 10,000-body problem.

You have:

1,000 × 10-body problems

which can be distributed across cores.

Space Engineers uses essentially this broader concept—multiple local Havok worlds/clusters inside a much larger coordinate space.

4. Use local coordinate systems inside each physics region

This also fixes huge-world precision.

Starbound currently depends on rapier3d with SIMD and parallel features. Regardless of whether you eventually choose double-precision physics, letting a solver operate at enormous absolute Minecraft coordinates is undesirable.

Store the authoritative position as something like:

GlobalPosition
    sectorX: i64
    sectorY: i64
    sectorZ: i64

    localX: f64
    localY: f64
    localZ: f64

For example:

sector size = 4096 blocks

A position might be:

sector: (5,192,812, -13, 91,028)

local:
    (123.419,
     -982.11,
      58.2)

Rapier never gets:

21,269,758,075.419

It gets:

123.419

relative to its physics region.

Large-world engines commonly solve precision this way or with double precision/origin shifting; Godot's large-world documentation explicitly discusses the precision problem and origin-shifting approach, while noting that full double-precision physics has performance/memory costs.

This architecture allows your logical universe to become effectively enormous without degrading local physics accuracy.

5. Add four simulation tiers

I would replace your current:

slow?
    1 substep
else
    2 substeps

system entirely.

Your current mixin scans every sublevel and globally changes the substep count depending on whether any body exceeds a velocity threshold.

That's useful today but fundamentally wrong for a massive simulation.

Use:

Tier	Body state	Update
Dormant	stationary and isolated	event-only / 0 Hz
Ballistic	moving but safely isolated	analytical/event-scheduled
Active	nearby interaction possible	Rapier 20 Hz
Critical	collision/joints/high speed	Rapier 20 Hz + CCD/extra accuracy
Dormant

A ship sitting in space:

velocity = 0
angular velocity = 0
no active forces
no nearby body
no player controlling it

should require essentially:

0 CPU/tick

It shouldn't even be iterated.

Rapier itself already implements sleeping bodies and exposes only bodies that were active during the previous step.

Starbound should extend that concept outside Rapier.

6. Ballistic bodies should not use Rapier at all

This is where you can scale to very large numbers of moving bodies.

Suppose a spaceship is:

300,000 blocks from anything
velocity: 80 blocks/sec
angular velocity: constant
thrusters unchanged

You don't need to run collision solving 20 times per second.

Its state is predictable.

You can integrate:

position += velocity * dt
rotation = integrate(rotation, angularVelocity, dt)

or evaluate its state directly from:

start position
start velocity
start rotation
start time

Then schedule when it must next receive attention.

Something like:

next_event = min(
    next_control_change,
    next_spatial_cell_crossing,
    predicted_neighbor_approach,
    predicted_terrain_approach,
    max_ballistic_interval
);

Instead of:

Are we near anything?
No.

Are we near anything?
No.

Are we near anything?
No.

20 times every second × 50,000 ships.

you get:

Ship 841 next requires simulation at tick 98231.

This is directly analogous to Factorio's robot optimization: instead of updating movement every tick, Factorio stores the movement intention and schedules the next meaningful update, with moving robots going up to 20 ticks and stationary ones up to 60 ticks without normal updates. That produced around 10–25% improvements in robot-heavy saves.

For Starbound, the gain could be much larger because isolated spaceflight is especially predictable.

7. Use swept-volume wake detection

The obvious danger of coarse simulation is:

what if two coarse bodies collide between updates?

Solve it conservatively.

For each ballistic body calculate a swept bound:

current AABB
   +
velocity × horizon
   +
rotation safety margin

So:

    current body
       ███

      movement
        ↓

       ███
       ███
       ███
       ███
       ███

= swept AABB / swept sphere

Search the global spatial index against that envelope.

If empty:

safe to remain ballistic

If another body's envelope overlaps:

wake both
create/merge physics region
insert into Rapier

Do this before exact collision is possible.

You aren't approximating collisions.

You're approximating motion only when you've mathematically established that collision can't happen during the approximation interval.

8. Build a proper world-scale spatial index

This should become a first-class subsystem:

struct WorldSpatialIndex {
    cells: HashMap<MacroCell, CellData>,
}

For example:

macrocell size = 256 or 512 blocks

Each macrocell stores body IDs:

Cell (502, 12, -992)
    ship 91
    ship 182
    asteroid 29

Factorio solved a very similar scaling issue for thousands of robots by indexing them according to map chunks and searching outward from the relevant chunk rather than scanning all robots.

You want two levels:

WORLD
  ↓
sparse macrocell hash
  ↓
cell-local BVH / body list
  ↓
body AABB

Large ships spanning many cells require either:

multi-cell registration

or a separate large-object BVH.

I'd probably use:

small/normal objects → hashed loose grid
large objects        → dynamic BVH
9. Immediately kill Sable's O(N) spatial queries

This becomes Phase 1 even before the big redesign.

Sable currently explicitly says:

// Brute force check all of them
return container.queryIntersecting(bounds);

when ticket-based queries aren't active.

That cannot remain.

Likewise your native changeBlock() currently does:

for (_, sable_body) in level_colliders.iter_mut() {
    if sable_body.contains(x, y, z) {
        ...
        break;
    }
}

so a block change can scan every body.

Change the API.

Instead of:

changeBlock(x, y, z, block);

use:

changeWorldBlock(x, y, z, block);

and:

changeSubLevelBlock(
    bodyId,
    localX,
    localY,
    localZ,
    block
);

When Minecraft already knows the sublevel being changed, searching for it again is pointless.

That turns:

O(number of ships)

into:

O(1)

for the common case.

10. Completely redesign Java ↔ Rust communication

Current Sable is doing something especially bad for high counts.

Every substep:

Rapier step
      ↓
for every sublevel:
    JNI getPose()

updateAllPoses() iterates all sublevels and readPose() invokes Rapier3D.getPose() separately.

That needs to disappear.

Create one command buffer:

NativeInputBuffer

ApplyForce
ApplyForce
SetKinematicPose
WakeBody
ChangeBlock
ChangeBlock
CreateBody
DestroyBody
...

and one output buffer:

NativeOutputBuffer

MovedBody
MovedBody
SleepEvent
WakeEvent
CollisionEvent
CollisionEvent
...

Then:

Rapier3D.stepWorld(
    scene,
    inputDirectBuffer,
    outputDirectBuffer
);

One JNI call per physics update.

Use a direct ByteBuffer or equivalent contiguous native memory.

No:

new double[]
JNI call
new Vector3d
JNI call
new double[]
JNI call
...
11. Return active bodies only

Rapier already gives you:

island_manager.active_bodies()

which is specifically intended to let applications update only bodies that moved.

So native stepping becomes:

pipeline.step(...);

for handle in island_manager.active_bodies() {
    let rb = &bodies[*handle];

    output.push(BodyState {
        id,
        position,
        rotation,
        linear_velocity,
        angular_velocity,
    });
}

Then Java does:

10,000 bodies
9,800 asleep

Rust sends:
200 body updates

not:

10,000 JNI getPose() calls

Your current profiler even labels:

let active_bodies = sim.rigid_body_set.len();

as active bodies, even though that is total body count.

Change that immediately to actual Rapier active bodies.

12. Stop universally enabling CCD

Every Starbound Sable sublevel currently gets:

.ccd_enabled(true)

and your integration config uses:

max_ccd_substeps: 3

Rapier explicitly says CCD has additional computational cost and is useless on bodies expected to move slowly; its normal default is off, with a default maximum of one CCD substep.

Make CCD per-body.

Dormant
    CCD OFF

slow Active ship
    CCD OFF

ordinary ship
    CCD OFF unless collision risk

fast small body
    CCD ON

high-speed imminent interaction
    CCD ON

critical projectile
    CCD ON

Use something like:

ccdRisk =
    travelDistanceThisTick /
    smallestRelevantColliderDimension

and hysteresis.

This is far better than saying:

all ships use expensive collision safeguards forever
13. Default to one physics step

Once selective CCD exists, I'd target:

1 Rapier step / Minecraft tick

as the normal active simulation.

Extra solver work becomes region-specific.

Region A
3 cruising ships
→ 1 step

Region B
stationary station
→ sleeping

Region C
20 ships smashing together
→ higher solver effort

Region D
one fast projectile
→ CCD

Do not let one difficult body double simulation work for the entire dimension.

Your current global adaptive substep mixin should eventually be deleted.

14. Make voxel geometry persistent and incremental

The current native collider setup can rebuild an octree by walking chunk sections and all 16 × 16 × 16 blocks when bounds change.

For huge ships, geometry needs to become an independent persistent object:

struct ShipGeometry {
    sections: HashMap<SectionPos, CollisionSection>,
    hierarchy: SectionBVH,
    bounds: Aabb,
    mass_cache: MassProperties,
    version: u64,
}

Each section might have:

4096-bit occupancy mask
block collider IDs
surface/exterior mask
local BVH/octree
version

Block update:

change block
    ↓
dirty ONE section
    ↓
update occupancy
    ↓
update section collision tree
    ↓
update parent bounds

Not:

change bounds
    ↓
rescan entire ship

Keep Sable's custom voxel collider approach; it is conceptually better for Minecraft than making thousands of Rapier colliders.

But make it hierarchical:

body AABB
   ↓
section BVH
   ↓
section occupancy
   ↓
exact block shape
15. Share geometry between universe and Rapier regions

This matters when bodies migrate between clusters.

Don't recreate a giant ship collider when it moves from:

PhysicsRegion 7

to:

PhysicsRegion 8

Use:

Arc<ShipGeometry>

Then the region collider references the same immutable geometry.

Dynamic block changes create/version updated geometry.

Think:

UniverseBody
   │
   └── GeometryId 77
              │
              ▼
         Arc<ShipGeometry>
          /      |       \
region A ref   query ref   save ref

Migration becomes cheap.

16. Aggregate fixed assemblies

This is another major Factorio-style gain.

Factorio optimized belts by internally treating sequences of adjacent belt pieces as larger continuous structures rather than independent repeatedly updated entities.

Do the physics equivalent.

Suppose:

Ship A
   │ fixed docking joint
Ship B
   │ fixed docking joint
Ship C
   │
Ship D

If these are effectively rigid:

4 bodies
3 joints
multiple solver constraints

can become:

1 compound physical body
4 logical Minecraft sublevels

Logical identity remains separate.

Physics identity becomes unified.

When a joint unlocks:

compound assembly
     ↓
recompute mass/inertia
     ↓
split body

For giant modular stations this could be one of the largest optimizations available.

Factorio's old optimization work repeatedly used this general strategy—grouping many logically distinct elements into a cheaper internal representation and using sleep/wakeup instead of updating everything.

17. Collision events need to stay native until they matter

Current Starbound truncates the native collision list at:

max_collisions = 100;

and serializes 15 doubles per collision into Java.

That won't scale.

Aggregate native contacts by body pair:

ship A ↔ ship B

137 contact points

                 ↓

one event:
    maximum impulse
    accumulated impulse
    representative normal
    strongest contact point

Only emit Java gameplay events when:

collision started
collision ended
damage threshold crossed
fragile-block threshold crossed
special gameplay trigger occurred

A ship resting on terrain shouldn't generate streams of effectively redundant Java events.

Use a fixed/native ring buffer rather than allocating arrays every tick.

18. Replace small-data structures that become bad at scale

Current Java has:

Int2ObjectArrayMap<ServerSubLevel>

for active sublevels.

Replace with:

Int2ObjectOpenHashMap

or preferably a dense indexed structure if runtime IDs permit:

ServerSubLevel[] bodiesByRuntimeId;

Native currently has many:

HashMap<LevelColliderID, ...>
HashMap<i64, ChunkSection>

Some are appropriate because world chunks are sparse.

But runtime body IDs should probably become:

slotmap
slab
Vec<Option<T>>
generational arena

rather than hashing in hot loops.

Your target layout should be data-oriented:

positions[]
rotations[]
velocities[]
bounds[]
tiers[]
regionIds[]
flags[]

where performance-sensitive passes can process contiguous memory.

Cache locality is another principle Factorio's optimization work repeatedly emphasizes.

19. Move the scheduler to Rust

Eventually Java should not orchestrate individual bodies at all.

Today:

Java
 for body
 for body
 JNI
 for body
 JNI
 Rapier
 for body
 JNI

Target:

JAVA MAIN THREAD

collect Minecraft changes
        │
        ▼
one native command buffer
        │
        ▼

RUST PHYSICS UNIVERSE
        │
        ├─ process wake events
        ├─ process scheduled ballistic bodies
        ├─ update global spatial index
        ├─ merge/split interaction regions
        ├─ step regions in parallel
        ├─ process collision events
        └─ construct output buffer
        │
        ▼

JAVA MAIN THREAD

apply changed poses/events

This removes massive amounts of JVM/native chatter.

20. Parallelize regions, not arbitrary bodies

Your Rapier build already enables:

"simd-nightly"
"parallel"

Rapier itself warns that parallelism only pays off for sufficiently complex scenes because threading overhead can otherwise hurt.

So make your larger parallel unit:

PhysicsRegion

For example:

Core 1 → region 101
Core 2 → region 338
Core 3 → regions 91 / 92
Core 4 → region 880
...

The ideal native entry point becomes:

physics_universe.step_all_regions(dt);

with Rayon distributing regions.

For a huge contact-heavy region, Rapier's own internal parallelism can still help.

Since both would live in the same Rayon ecosystem, you can benchmark nested scheduling rather than spawning separate Java physics pools.

21. Refactor away locks from the hot path

This becomes necessary for region-level concurrency.

Current PhysicsScene contains:

RwLock<SimulationSceneData>
Arc<RwLock<SableSceneData>>

and the global physics configuration lives behind another RwLock.

The global integration parameters are even modified before every step().

Instead use ownership:

one PhysicsRegion
    owned by
one worker during step

Therefore:

struct PhysicsRegion {
    sim: SimulationSceneData,
    data: RegionData,
}

No locks needed internally while a region is being stepped.

Global shared structures should mostly be immutable:

Arc<ColliderRegistry>
Arc<BlockShapeRegistry>

Commands cross region boundaries through queues.

Also fix this before serious multithreading:

ReportedCollisionBuffer(RefCell<Vec<_>>)

unsafe impl Sync for ReportedCollisionBuffer {}

which exists in the current scene code.

Don't build the future concurrent architecture on manually asserting that a RefCell is thread-safe.

22. Introduce a hard per-tick work budget

This is another direct Factorio lesson.

Factorio explicitly limits potentially expensive work per update instead of letting an unbounded operation stall an entire tick.

Starbound should have:

Physics budget: e.g. X ms/tick

Critical collision regions
    highest priority

Player-near regions
    high priority

Normal active regions
    normal priority

Ballistic scheduling
    cheap

Collider rebuilds
    background budget

Spatial maintenance
    background budget

Important distinction:

Do not skip correctness-critical contact physics because the budget ran out.

Budget things that can safely be deferred:

collider rebuilding
cleanup
remote ballistic refresh
region splitting
profiling
spatial compaction
noncritical maintenance
23. Separate physics activity from Minecraft logic activity

This matters a lot for Starbound.

A stationary ship might physically be:

Dormant

but have:

Create machines
furnaces
block entities
redstone
players

running inside it.

So maintain independent states:

PhysicsActivity
LogicActivity
NetworkActivity
ChunkActivity

Example:

Factory ship

physics:
    Dormant

block simulation:
    Active

network:
    NearPlayer

Rapier cost:
    ~0

Don't tie "physics sleeping" to "stop ticking Minecraft."

Later you can optimize remote ship logic separately, but keep that out of the first physics redesign because compatibility risks are much higher.

24. Terrain must be streamed into physics regions

No gigantic physics scene should contain every Minecraft chunk.

Each active region computes:

region body swept bounds
        +
safety margin

Then terrain collision cache contains only overlapping world sections.

Physics region
      │
      ▼
required terrain cells

[-2,-2] [-1,-2] [0,-2]
[-2,-1] [-1,-1] [0,-1]
[-2, 0] [-1, 0] [0, 0]

When the region moves:

drop old collision cells
load new collision cells

No Man's Sky is useful here as an architectural reference: its massive world depends on continuous generation/population/simulation pipelines rather than materializing the entire universe as one conventional world state.

For Starbound:

NMS lesson = world locality/streaming.

Factorio lesson = scheduling/sleep/aggregation.

Space Engineers lesson = multiple local physics worlds.

Those three together are much more useful than copying any one game.

25. The target complexity

Current Sable behaves too much like:

tick cost ≈ O(total loaded bodies)

because Java repeatedly scans sublevels and reads every pose.

The redesigned target should be:

tick cost ≈

O(
    scheduled_events
  + active_bodies
  + nearby_candidate_pairs
  + actual_contacts
  + dirty_geometry
)

not

O(total universe bodies)

That distinction is the entire architecture.

What I would implement, in order

This would be my actual roadmap.

Phase	Work	Why
[x] 0 — Benchmark foundation	Build synthetic body benchmarks and proper profiling	Prevent optimizing guesses
[x] 1 — Remove obvious scaling failures	O(N) queries, ArrayMap, universal CCD, profiler overhead	Cheap wins
[x] 2 — Batched native interface	One JNI input/output buffer; active poses only	Remove Java/JNI O(N) cost
[x] 3A — Native body registry scaffolding	Separate UniverseBody from RapierBody	Establishes the residency boundary
[x] 3B — Registry stabilization	Persistent dynamics, zero O(N) registry sync, safe reconstruction and buffers	Makes sleeping/eviction safe
[x] 4 — Simulation tiers	Dormant / Ballistic / Active / Critical	Native tiers plus Java active/continuous/dirty scheduling
[x] 5 — Global spatial index	Sparse macrocell grid + swept queries	Enables wake-on-proximity
[x] 6 — Local physics regions	Multiple Rapier worlds with local origins	Huge-world + concurrency solution
[x] 7 — Region merge/split	Interaction graph and dynamic clustering	Incremental cell, edge, component, and migration maintenance
[x] 8 — Voxel geometry redesign	Persistent section hierarchy + incremental dirty updates	Section trees plus an incrementally growable top-level octree
[~] 9 — Region parallelism	Active/dirty/due region scheduler + bounded thread pool	Dormant regions are skipped; hot-path lock cleanup remains
[ ] 10 — Assembly aggregation	Merge fixed/docked ships into compound physical bodies	Identification exists; true aggregation remains
[x] 11 — Terrain collision streaming	Region-local world collider cache	Per-body footprints and section refcounts are incremental
[~] 12 — Network/activity LOD	Send changed bodies only, extrapolate remote motion	Feature exists; scale validation remains

Phase 7–12 implementation notes (2026-08-10)

- Phase 4 uses Java active/next-active/continuous sets for actor, mass, force, ticket, and pose work. Dormant bodies no longer participate in per-tick physics-side `getAllSubLevels()` passes. Ballistic integration consistently applies `gravity_scale`, and a critical body no longer multiplies the whole region step count.
- Phase 7 incrementally updates swept-AABB cell memberships and interaction edges only for moved/expired bodies, recomputes only affected components, and checks migration only for that same dirty set. It retains the oversized-body fallback and delayed splitting.
- Phase 8 keeps one small persistent octree per body section beneath a dirty-versioned top-level octree. A one-block edit updates its section tree and the top-level accelerator; root growth wraps the existing tree in O(number of doublings), including negative-axis growth, rather than rebuilding every block when bounds change.
- Phase 9 flushes commands on the server thread and steps callback-free regions through one bounded worker pool. Regions containing Java-backed contact callbacks stay serialized for thread safety. An active/dirty/due scheduler now avoids physics, pose, terrain, and collision passes over every region; skipped scenes receive elapsed scheduler ticks when they wake.
- Phase 10 records fixed-joint connected components as native assemblies with stable roots and sizes. Component rebuilding is event-driven and visits only assembly participants, not all universe bodies. True one-rigid-body compound aggregation is still open.
- Phase 11 stores a swept terrain footprint per body and maintains region section refcounts by old/new footprint differences. Only moved, added, removed, or migrated bodies update the working set; arbitrary default-region objects use one reserved footprint owner. A reverse section-to-region index sends world chunk and block changes only to interested regions.
- Phase 12 retains changed-only snapshots, applies distance-based send intervals, carries velocity through the packet path, and performs client-side extrapolation capped at five ticks.

Phase 0 benchmark foundation (2026-08-10)

- `phase0_scaling` covers the complete A–R workload matrix: sleeping, isolated moving, clustered, contact-heavy, huge-voxel, rapid-edit, fixed-joint, and extreme-coordinate cases.
- Benchmark-only Rapier profiler instrumentation reports step, broadphase, narrowphase, solver, and CCD time without adding timer overhead to production native builds.
- Each physics workload reports Rapier bodies, awake bodies, true island-manager active bodies, colliders, candidate pairs, manifolds, CCD bodies, and joints. Criterion stores comparable timing baselines under `target/criterion`.
- `worldengine_rapier/BENCHMARKING.md` documents reproducible full, quick, filtered, JFR, allocation-profiler, JNI-byte, and native-stage measurement workflows.
- `registry_scaling` is the fixed-active-set acceptance matrix: 10,000 / 100,000 / 1,000,000 real `UniverseBody` records in `SableSceneData` and `WorldSpatialIndex`, with exactly 1,000 resident and 100 awake bodies. It runs the production Rapier scene, tier scheduler, and active-pose export working set rather than an unrelated vector.
- Scheduled-body constraint checks use Rapier's per-body joint adjacency index instead of scanning the complete impulse-joint set.
Phase 0 is implemented by the reproducible matrix below.

Your benchmark should generate at least these cases:

A: 100 bodies sleeping
B: 1,000 bodies sleeping
C: 10,000 bodies sleeping
D: 50,000 bodies sleeping

E: 1,000 moving isolated
F: 10,000 moving isolated

G: 1,000 bodies in 100 clusters
H: 1,000 bodies in 10 clusters

I: 100 bodies actively colliding
J: 500 actively colliding
K: 1,000 actively colliding

L: one huge voxel ship
M: 100 huge voxel ships

N: rapid block editing
O: many docking joints

P: coordinates around 1 million
Q: 100 million
R: billions+

Measure:

total MSPT

Java prephysics
Java actor ticks
JNI time
Rust scheduling
Rapier broadphase
Rapier narrowphase
solver
CCD
voxel narrowphase
collider rebuild
collision export
pose export

persistent bodies
resident bodies
Rapier bodies
awake bodies
active islands
candidate pairs
contact manifolds
CCD bodies

bytes Java→native
bytes native→Java
JNI calls/tick

Java allocations/tick
native allocations/tick
memory/body

Your existing profiler needs correction first because it currently reports total rigid bodies as active_bodies.

Changes I'd make to the repository first
MixinSubLevelPhysicsSystem.java

Current:

scan every ship
set global substeps 1/2

Eventually delete it.

Temporary replacement:

always 1 normal substep

and rely on selective CCD while the larger scheduler is built.

SubLevelPhysicsSystem.java

The current repeated:

for (ServerSubLevel ...)

passes during every substep need to be replaced with activity sets.

Eventually:

activeForceBodies
activeMassDirtyBodies
activePhysicsBodies
activeActors

instead of:

all sublevels
RapierPhysicsPipeline.java

Replace:

Int2ObjectArrayMap

and:

readPose(body)

per body.

Introduce:

stepBatch(...)

and:

applyActiveBodyStates(...)
rapier/src/lib.rs

First changes:

CCD off by default
max_ccd_substeps = 1
correct active-body profiling
batch commands
batch outputs
aggregate collisions
remove level_colliders linear scan

Current universal CCD and three CCD substeps are directly visible here.

Then turn it into:

PhysicsUniverse
  ├─ DimensionSpace
  │    ├─ WorldSpatialIndex
  │    ├─ BodyRegistry
  │    └─ PhysicsRegion[]
  └─ Scheduler
scene.rs

This file needs the largest architectural rewrite.

Current:

one SimulationSceneData
HashMaps
RwLocks
one scene

Target:

struct PhysicsUniverse {}

struct DimensionPhysicsSpace {}

struct PhysicsRegion {}

struct BodyRegistry {}

struct WorldSpatialIndex {}

struct GeometryRegistry {}

and avoid shared locks inside the stepping hot path.

algo.rs

Do the micro-optimization work after the architecture.

Your current hot voxel lookup calculates a logarithm:

simd_ln()

and allocates temporary Vecs while querying overlapping nodes.

Eventually:

derive octree level from integer operations
SmallVec / scratch buffers
thread-local workspace
visitor callbacks
no transient allocations

But this is Phase 8+, not Phase 1.

Saving 20 ns inside the voxel query won't matter if Java is unnecessarily touching 50,000 sleeping ships.

What scale should you design for?

I'd explicitly build the architecture against four separate numbers:

N = total persistent bodies
R = resident bodies
A = actively simulated bodies
C = bodies in active contact clusters

I'd design toward something like:

N: 1,000,000+
R: 100,000+
A: 10,000+
C: hundreds–low thousands

Then benchmark and adjust.

A million persistent ships is not inherently difficult because most are just records.

Ten thousand moving isolated ships can potentially be handled because ballistic/event-driven simulation is cheap.

Ten thousand active Rapier bodies scattered across independent regions is much more tractable than a single enormous interacting scene.

But:

10,000–100,000 bodies all simultaneously colliding with each other is a fundamentally different problem.

No amount of sleeping, spatial indexing or JNI batching eliminates the solver/contact workload when every body genuinely matters.

For that case you'd need increasingly aggressive approximations such as:

resting-cluster freezing
body aggregation
simplified collision proxies
debris particle physics
lower solver accuracy
GPU-oriented specialized simulation

Modern Havok makes this same distinction: its lightweight “Physics Particles” can outperform normal rigid bodies by roughly 4–6× in a 10,000-object example specifically by accepting limitations compared with full gameplay rigid bodies.

So don't promise that every object receives identical maximum-fidelity physics forever.

Design Starbound so fidelity follows interaction importance.

The final architecture I'd aim for
══════════════════════════════════════════════════════════
                    MINECRAFT SERVER
══════════════════════════════════════════════════════════

  blocks / players / Create / sublevels / commands
                         │
                         │ batched changes
                         ▼

══════════════════════════════════════════════════════════
                STARBOUND PHYSICS UNIVERSE
══════════════════════════════════════════════════════════

                   Body Registry
             global 64-bit/sector space
                         │
                         ▼
                World Spatial Index
                  sparse macro grid
                         │
             ┌───────────┼───────────┐
             ▼           ▼           ▼

          Dormant     Ballistic     Candidate
            0Hz       scheduled     interaction
                         │              │
                         └──────┬───────┘
                                ▼
                       Interaction Graph
                                │
                       connected components
                                │
          ┌─────────────────────┼─────────────────────┐
          ▼                     ▼                     ▼
       Region A              Region B              Region C
      local origin          local origin          local origin
          │                     │                     │
       Rapier                Rapier                Rapier
          │                     │                     │
          └─────────────────────┼─────────────────────┘
                                │
                      parallel native step
                                │
                                ▼
                         Output Ring Buffer
                                │
                      active bodies/events only
                                ▼

══════════════════════════════════════════════════════════
                    MINECRAFT SERVER
══════════════════════════════════════════════════════════

                    apply changed state

That is the architecture I would commit to.

Factorio gives you the “don't update things unnecessarily” philosophy. Its optimizations repeatedly use sleeping/wakeup, spatial indexing, scheduled updates and aggregation.

No Man's Sky gives you the “an enormous logical world does not need to be fully materialized at once” philosophy.

And most importantly, Space Engineers demonstrates the exact large-space physics trick Starbound needs: a huge 64-bit logical world split into multiple smaller local physics clusters using a conventional 32-bit physics engine.

If this were my project, I would begin Phase 0 → Phase 2 immediately, then make the PhysicsUniverse + regional-cluster redesign the next major Starbound version rather than continuing to pile mixins onto Sable's current all-sublevel tick architecture. That gives you a foundation whose performance can scale by orders of magnitude instead of continually fighting O(N) costs.

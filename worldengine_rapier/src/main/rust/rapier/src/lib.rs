pub mod algo;
mod boxes;
mod buoyancy;
mod collider;
mod config;
mod contraptions;
mod dispatcher;
mod event_handler;
mod groups;
mod hooks;
mod joints;
mod rope;
pub mod scene;
mod voxel_collider;

use jni::JNIEnv;
use jni::objects::{JClass, JDoubleArray, JIntArray};
use jni::sys::{jboolean, jdouble, jint, jlong};
use rapier3d::glamx::{DVec3, Quat};
use std::collections::{BinaryHeap, HashMap};
use std::sync::{Arc, OnceLock, RwLock};

use fern::colors::{Color, ColoredLevelConfig};
use log::info;

use crate::buoyancy::compute_buoyancy;
use crate::collider::{LevelCollider, update_collider_aabb};
use crate::dispatcher::SableDispatcher;
use crate::event_handler::SableEventHandler;
use crate::glamx::IVec3;
use crate::groups::LEVEL_GROUP;
use crate::joints::SableJointSet;
use crate::rope::RopeMap;
use crate::scene::{
    ChunkAccess, ChunkMap, SableManifoldInfoMap, SableSceneData, SimulationSceneData,
    pack_section_pos,
};
use crate::voxel_collider::VoxelColliderMap;
use hooks::SablePhysicsHooks;
use marten::Real;
use marten::level::VoxelPhysicsState::Interior;
use marten::level::{
    ALL_VOXEL_PHYSICS_STATES, BlockState, CHUNK_SHIFT, ChunkSection, OCTREE_CHUNK_SHIFT,
    OCTREE_CHUNK_SIZE, OctreeChunkSection, VoxelPhysicsState,
};
use marten::octree::SubLevelOctree;
use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use rapier3d::prelude::*;
use scene::{LevelColliderID, PhysicsScene, ReportedCollisionBuffer};

#[derive(Debug)]
pub struct ActiveLevelColliderInfo {
    pub collider: Option<ColliderHandle>,
    pub static_mount: Option<RigidBodyHandle>,
    pub fake_velocities: Option<RigidBodyVelocity<Real>>,
    pub local_bounds_min: Option<IVec3>,
    pub local_bounds_max: Option<IVec3>,
    pub octree_origin: Option<IVec3>,
    pub center_of_mass: Option<DVec3>,
    pub octree: Option<SubLevelOctree>,
    pub section_octrees: HashMap<i64, SubLevelOctree>,
    pub chunk_map: Option<ChunkMap>,
    pub geometry_version: u64,
    pub dirty_sections: std::collections::HashSet<i64>,
}

impl ChunkAccess for ActiveLevelColliderInfo {
    fn get_chunk_mut(&mut self, x: i32, y: i32, z: i32) -> Option<&mut ChunkSection> {
        self.chunk_map
            .as_mut()
            .unwrap()
            .get_mut(&pack_section_pos(x, y, z))
    }

    fn get_chunk(&self, x: i32, y: i32, z: i32) -> Option<&ChunkSection> {
        self.chunk_map
            .as_ref()
            .unwrap()
            .get(&pack_section_pos(x, y, z))
    }
}

impl ActiveLevelColliderInfo {
    /// Creates a new handle for a sable object with rigidbody and collider handles
    #[must_use]
    pub fn new(collider: Option<ColliderHandle>) -> Self {
        Self {
            collider,
            static_mount: None,
            fake_velocities: None,
            chunk_map: None,
            local_bounds_min: None,
            local_bounds_max: None,
            octree_origin: None,
            center_of_mass: None,
            octree: None,
            section_octrees: HashMap::new(),
            geometry_version: 0,
            dirty_sections: std::collections::HashSet::new(),
        }
    }

    pub fn has_own_chunks(&self) -> bool {
        self.chunk_map.is_some()
    }

    fn mark_section_dirty(&mut self, x: i32, y: i32, z: i32) {
        self.geometry_version = self.geometry_version.wrapping_add(1);
        self.dirty_sections.insert(pack_section_pos(x, y, z));
    }

    /// Sets the local bounds for the object
    pub fn set_local_bounds(
        &mut self,
        min: IVec3,
        max: IVec3,
        level_chunks: &ChunkMap,
        collider_map: &VoxelColliderMap,
    ) {
        if self.octree.is_none() {
            let max_axis = (max - min).max_element() as u32 + 1;
            let smallest_pow_2_above = max_axis.next_power_of_two();
            let chunk_min = min >> CHUNK_SHIFT;
            let chunk_max = max >> CHUNK_SHIFT;
            self.octree = Some(SubLevelOctree::new(
                smallest_pow_2_above.trailing_zeros() as i32
            ));
            self.octree_origin = Some(min);
            let has_own_chunks = self.has_own_chunks();
            for cx in chunk_min.x..=chunk_max.x {
                for cy in chunk_min.y..=chunk_max.y {
                    for cz in chunk_min.z..=chunk_max.z {
                        let chunk = if has_own_chunks {
                            self.chunk_map
                                .as_ref()
                                .unwrap()
                                .get(&pack_section_pos(cx, cy, cz))
                        } else {
                            level_chunks.get(&pack_section_pos(cx, cy, cz))
                        };

                        if let Some(chunk_section) = chunk {
                            for x in 0..16 {
                                for y in 0..16 {
                                    for z in 0..16 {
                                        let block_owned = chunk_section.get_block(x, y, z);
                                        if block_owned.1 == VoxelPhysicsState::Empty {
                                            continue;
                                        }

                                        insert_block_octree(
                                            collider_map,
                                            self.octree.as_mut().unwrap(),
                                            &block_owned,
                                            false,
                                            (x + (cx << CHUNK_SHIFT))
                                                - self.octree_origin.unwrap().x,
                                            (y + (cy << CHUNK_SHIFT))
                                                - self.octree_origin.unwrap().y,
                                            (z + (cz << CHUNK_SHIFT))
                                                - self.octree_origin.unwrap().z,
                                        );
                                        insert_block_octree(
                                            collider_map,
                                            self.section_octrees
                                                .entry(pack_section_pos(cx, cy, cz))
                                                .or_insert_with(|| {
                                                    SubLevelOctree::new(CHUNK_SHIFT.into())
                                                }),
                                            &block_owned,
                                            false,
                                            x,
                                            y,
                                            z,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // Expand by wrapping the old root. Existing leaves retain their
            // coordinates, so changing bounds never walks the whole body.
            loop {
                let origin = self.octree_origin.unwrap();
                let side = 1_i64 << self.octree.as_ref().unwrap().log_size;
                let covered = min.x as i64 >= origin.x as i64
                    && min.y as i64 >= origin.y as i64
                    && min.z as i64 >= origin.z as i64
                    && (max.x as i64) < origin.x as i64 + side
                    && (max.y as i64) < origin.y as i64 + side
                    && (max.z as i64) < origin.z as i64 + side;
                if covered {
                    break;
                }

                let negative_x = (min.x as i64) < origin.x as i64;
                let negative_y = (min.y as i64) < origin.y as i64;
                let negative_z = (min.z as i64) < origin.z as i64;
                let old_octant =
                    (negative_x as i32) | ((negative_y as i32) << 1) | ((negative_z as i32) << 2);
                self.octree.as_mut().unwrap().grow(old_octant);
                self.octree_origin = Some(IVec3::new(
                    origin.x - if negative_x { side as i32 } else { 0 },
                    origin.y - if negative_y { side as i32 } else { 0 },
                    origin.z - if negative_z { side as i32 } else { 0 },
                ));
            }
        }
        self.local_bounds_min = Some(min);
        self.local_bounds_max = Some(max);
    }

    fn insert_chunk(
        &mut self,
        chunk_section: &ChunkSection,
        cx: i32,
        cy: i32,
        cz: i32,
        collider_map: &VoxelColliderMap,
    ) {
        for x in 0..16 {
            for y in 0..16 {
                for z in 0..16 {
                    self.insert_block(
                        x + (cx << CHUNK_SHIFT),
                        y + (cy << CHUNK_SHIFT),
                        z + (cz << CHUNK_SHIFT),
                        &chunk_section.get_block(x, y, z),
                        false,
                        collider_map,
                    );
                }
            }
        }
    }

    fn insert_block(
        &mut self,
        x: i32,
        y: i32,
        z: i32,
        state: &BlockState,
        remove: bool,
        collider_map: &VoxelColliderMap,
    ) {
        let section_key = pack_section_pos(x >> CHUNK_SHIFT, y >> CHUNK_SHIFT, z >> CHUNK_SHIFT);
        let section = self
            .section_octrees
            .entry(section_key)
            .or_insert_with(|| SubLevelOctree::new(CHUNK_SHIFT.into()));
        insert_block_octree(collider_map, section, state, remove, x & 15, y & 15, z & 15);
        if section.is_empty() {
            self.section_octrees.remove(&section_key);
        }

        let local_min = self.octree_origin.unwrap();
        let x = x - local_min.x;
        let y = y - local_min.y;
        let z = z - local_min.z;

        let Some(octree) = &mut self.octree else {
            panic!("No octree!");
        };
        insert_block_octree(collider_map, octree, state, remove, x, y, z);
    }

    fn contains(&self, x: i32, y: i32, z: i32) -> bool {
        if self.local_bounds_min.is_none() || self.local_bounds_max.is_none() {
            return false;
        }

        let local_min = self.local_bounds_min.unwrap();
        let local_max = self.local_bounds_max.unwrap();

        x >= local_min.x
            && x <= local_max.x
            && y >= local_min.y
            && y <= local_max.y
            && z >= local_min.z
            && z <= local_max.z
    }
}

/// Global physics engine state shared across all scenes.
pub struct PhysicsState {
    /// An array of i32 IDs -> block collider entries
    voxel_collider_map: VoxelColliderMap,
}

fn default_integration_parameters() -> IntegrationParameters {
    IntegrationParameters {
        dt: 1.0 / 20.0,
        max_ccd_substeps: 1,
        normalized_prediction_distance: 0.005,
        contact_softness: SpringCoefficients {
            natural_frequency: 30.0,
            damping_ratio: 5.0,
        },
        normalized_max_corrective_velocity: 50.0,
        normalized_allowed_linear_error: 0.0025,
        ..IntegrationParameters::default()
    }
}

/// A collision to report to the Java side.
#[derive(Debug, Clone)]
pub struct ReportedCollision {
    body_a: Option<LevelColliderID>,
    body_b: Option<LevelColliderID>,
    local_point_a: DVec3,
    local_point_b: DVec3,
    local_normal_a: DVec3,
    local_normal_b: DVec3,
    force_amount: f64,
}

pub static PHYSICS_STATE: OnceLock<RwLock<PhysicsState>> = OnceLock::new();

const COMMAND_MAGIC: i32 = 0x5341424C;
const COMMAND_PROTOCOL_VERSION: i16 = 1;
const COMMAND_HEADER_SIZE: usize = 14;

fn validate_command_header(data: &[u8]) -> Result<usize, &'static str> {
    if data.len() < COMMAND_HEADER_SIZE {
        return Err("buffer is shorter than the command header");
    }
    let magic = i32::from_ne_bytes(data[0..4].try_into().unwrap());
    let version = i16::from_ne_bytes(data[4..6].try_into().unwrap());
    let command_count = i32::from_ne_bytes(data[6..10].try_into().unwrap());
    let declared_length = i32::from_ne_bytes(data[10..14].try_into().unwrap());
    if magic != COMMAND_MAGIC {
        return Err("command magic does not match");
    }
    if version != COMMAND_PROTOCOL_VERSION {
        return Err("command protocol version does not match");
    }
    if command_count < 0 {
        return Err("command count is negative");
    }
    if declared_length < 0 || declared_length as usize != data.len() {
        return Err("declared command length does not match the buffer");
    }
    Ok(command_count as usize)
}

pub fn with_handle<F, R>(handle: jlong, f: F) -> R
where
    F: FnOnce(&PhysicsScene) -> R,
{
    assert!(handle != 0, "null scene handle");
    unsafe { f(&*(handle as *const PhysicsScene)) }
}

#[inline(always)]
pub fn ensure_physics_state() {
    PHYSICS_STATE.get_or_init(|| {
        RwLock::new(PhysicsState {
            voxel_collider_map: VoxelColliderMap::new(),
        })
    });
}

#[inline(always)]
pub fn get_physics_state() -> std::sync::RwLockReadGuard<'static, PhysicsState> {
    ensure_physics_state();
    PHYSICS_STATE.get().unwrap().read().unwrap()
}

#[inline(always)]
pub fn get_physics_state_mut() -> std::sync::RwLockWriteGuard<'static, PhysicsState> {
    ensure_physics_state();
    PHYSICS_STATE.get().unwrap().write().unwrap()
}

#[inline(always)]
pub fn get_rigid_body_mut<'a>(
    sim: &'a mut SimulationSceneData,
    sable_data: &SableSceneData,
    id: LevelColliderID,
) -> &'a mut RigidBody {
    let handle = sable_data
        .rigid_bodies
        .get(&id)
        .expect("No rigid body for id");
    &mut sim.rigid_body_set[*handle]
}

#[inline(always)]
pub fn get_rigid_body<'a>(
    sim: &'a SimulationSceneData,
    sable_data: &SableSceneData,
    id: LevelColliderID,
) -> &'a RigidBody {
    let handle = sable_data
        .rigid_bodies
        .get(&id)
        .expect("No rigid body for id");
    &sim.rigid_body_set[*handle]
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_createUniverse<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    _dimension_id: jint,
) -> jlong {
    let universe = Arc::new(RwLock::new(crate::scene::DimensionUniverse::default()));
    Arc::into_raw(universe) as jlong
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_destroyUniverse<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
) {
    if handle != 0 {
        unsafe {
            let _ = Arc::from_raw(handle as *const RwLock<crate::scene::DimensionUniverse>);
        }
    }
}

pub fn with_universe<F, R>(universe_handle: jlong, f: F) -> R
where
    F: FnOnce(&mut crate::scene::DimensionUniverse) -> R,
{
    assert!(universe_handle != 0, "null universe handle");
    let rwlock = unsafe { &*(universe_handle as *const RwLock<crate::scene::DimensionUniverse>) };
    let mut universe = rwlock.write().unwrap();
    f(&mut *universe)
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_initialize<'local>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    universe_handle: jlong,
    gx: jdouble,
    gy: jdouble,
    gz: jdouble,
    universal_drag: jdouble,
    world_origin_x: jdouble,
    world_origin_y: jdouble,
    world_origin_z: jdouble,
) -> jlong {
    let rwlock = unsafe { Arc::from_raw(universe_handle as *const RwLock<crate::scene::DimensionUniverse>) };
    let universe_clone = rwlock.clone();
    let _ = Arc::into_raw(rwlock);

    PHYSICS_STATE.get_or_init(|| {
        let colors = ColoredLevelConfig::new()
            .info(Color::Green)
            .error(Color::Red)
            .debug(Color::Blue);

        let _ = fern::Dispatch::new()
            .format(move |out, message, record| {
                out.finish(format_args!(
                    "[{}] [{}] ({}) {}",
                    humantime::format_rfc3339(std::time::SystemTime::now()),
                    colors.color(record.level()),
                    record.target(),
                    message
                ))
            })
            .level(log::LevelFilter::Info)
            .level_for("jni", log::LevelFilter::Error)
            .chain(std::io::stdout())
            .apply();

        RwLock::new(PhysicsState {
            voxel_collider_map: VoxelColliderMap::new(),
        })
    });

    let ground = RigidBodyBuilder::fixed();

    let collider = ColliderBuilder::new(SharedShape::new(LevelCollider::new(None, true)))
        .collision_groups(LEVEL_GROUP)
        .build();

    let sable_data = Arc::new(RwLock::new(SableSceneData {
        scene_handle: 0,
        main_level_chunks: HashMap::<i64, ChunkSection>::new(),
        octree_chunks: HashMap::<i64, OctreeChunkSection>::new(),
        joint_set: SableJointSet::new(),
        rope_map: RopeMap::default(),
        level_colliders: HashMap::<LevelColliderID, ActiveLevelColliderInfo>::new(),
        rigid_bodies: HashMap::<LevelColliderID, RigidBodyHandle>::new(),
        terrain_serial_callback_sections: 0,
        sublevel_serial_callback_sections: 0,
    }));
    let manifold_info_map = Arc::new(SableManifoldInfoMap::default());
    let reported_collisions = Arc::new(ReportedCollisionBuffer::new());
    let current_step_vm = Some(Arc::new(env.get_java_vm().unwrap()));

    let dispatcher = SableDispatcher {
        sable_data: Arc::clone(&sable_data),
        manifold_info_map: Arc::clone(&manifold_info_map),
    };

    let mut scene = PhysicsScene {
        universe: universe_clone,
        sim_data: RwLock::new(SimulationSceneData {
            integration_parameters: default_integration_parameters(),
            pipeline: PhysicsPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::with_query_dispatcher(
                dispatcher.chain(DefaultQueryDispatcher),
            ),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: SablePhysicsHooks {
                sable_data: Arc::clone(&sable_data),
                manifold_info_map: Arc::clone(&manifold_info_map),
                current_step_vm: current_step_vm.clone(),
            },
            event_handler: SableEventHandler {
                reported_collisions: Arc::clone(&reported_collisions),
            },
        }),
        sable_data,
        ground_handle: None,
        reported_collisions,
        current_step_vm,
        gravity: Vec3::new(gx as Real, gy as Real, gz as Real),
        world_origin: RwLock::new(crate::scene::DVec3::new(world_origin_x as f64, world_origin_y as f64, world_origin_z as f64)),
        universal_drag: universal_drag as Real,
        manifold_info_map,
    };

    {
        let mut sim_data = scene.sim_data.write().unwrap();
        sim_data.collider_set.insert(collider);

        scene.ground_handle = Some(sim_data.rigid_body_set.insert(ground));
    }

    info!("Rapier scene initialized");
    let scene_arc = Arc::new(scene);
    let scene_ptr = Arc::into_raw(scene_arc.clone()) as jlong;
    {
        let mut sable = scene_arc.sable_data.write().unwrap();
        sable.scene_handle = scene_ptr;
    }
    scene_ptr
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_dispose<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
) {
    if handle != 0 {
        unsafe {
            drop(Arc::from_raw(handle as *const PhysicsScene));
        }
    }
}

/// Extracts a message from a caught panic payload
fn panic_message(payload: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        s.to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".to_string()
    }
}

/// Catches a panic and throws a JVM RuntimeException with the panic message
fn throw_on_panic(env: &mut JNIEnv, result: Result<(), Box<dyn std::any::Any + Send>>) {
    if let Err(payload) = result {
        let msg = format!("Rapier native panic: {}", panic_message(&payload));
        let _ = env.throw_new("java/lang/RuntimeException", &msg);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_tick<'local>(
    mut env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    _time_step: jdouble,
) {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        with_handle(handle, |scene| {
            rope::tick(scene);
            joints::tick(scene);
            compute_buoyancy(scene);
        });
    }));

    throw_on_panic(&mut env, result);
}

/// Steps physics
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_step<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    handle: jni::sys::jlong,
    time_step: jni::sys::jdouble,
    elapsed_ticks: jni::sys::jint,
) {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        with_handle(handle, |scene| {
            crate::rope::tick(scene);
            crate::joints::tick(scene);

            scene.manifold_info_map.clear();

            let gravity = scene.gravity;
            {
                // Rapier callbacks read sable_data. Never hold its write lock
                // while stepping or the callback path self-deadlocks.
                let mut sim = scene.sim_data.write().unwrap();
                let sim = &mut *sim;
                // Read the live Rapier state after queued commands have been
                // applied. The registry snapshot is intentionally updated
                // after stepping and may still contain the previous velocity.
                let critical_handles = sim
                    .rigid_body_set
                    .iter()
                    .filter_map(|(handle, body)| {
                        (body.is_ccd_enabled() || body.linvel().length() * time_step as Real > 0.25)
                            .then_some(handle)
                    })
                    .collect::<Vec<_>>();
                // Rapier CCD is selective per body. Substepping the entire
                // region because one body is critical multiplies unrelated
                // solver work, so keep one region step and retain the voxel
                // occupancy guard below for critical bodies.
                let solver_substeps = 1;
                sim.integration_parameters.dt =
                    time_step as marten::Real / solver_substeps as marten::Real;
                for _ in 0..solver_substeps {
                    let critical_previous_positions = critical_handles
                        .iter()
                        .filter_map(|handle| {
                            sim.rigid_body_set.get(*handle).map(|body| {
                                (
                                    *handle,
                                    body.user_data as LevelColliderID,
                                    *body.position(),
                                    body.translation(),
                                )
                            })
                        })
                        .collect::<Vec<_>>();
                    let params = sim.integration_parameters.clone();
                    sim.pipeline.step(
                        gravity,
                        &params,
                        &mut sim.island_manager,
                        &mut sim.broad_phase,
                        &mut sim.narrow_phase,
                        &mut sim.rigid_body_set,
                        &mut sim.collider_set,
                        &mut sim.impulse_joint_set,
                        &mut sim.multibody_joint_set,
                        &mut sim.ccd_solver,
                        &sim.physics_hooks,
                        &sim.event_handler,
                    );
                    // Custom voxel colliders do not provide a Rapier shape-cast
                    // implementation. Critical bodies therefore get a conservative
                    // per-substep occupancy guard in addition to CCD, preventing a
                    // fast body's bounds from entering occupied terrain.
                    if !critical_previous_positions.is_empty() {
                        let sable = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
                        for (handle, id, previous, previous_translation) in
                            &critical_previous_positions
                        {
                            let Some(body) = sim.rigid_body_set.get_mut(*handle) else {
                                continue;
                            };
                            let Some(universe_body) = universe.universe_bodies.get(id) else {
                                continue;
                            };
                            let previous_bounds =
                                recentered_bounds(universe_body.bounds, scene.local_to_global(*previous_translation));
                            let current_bounds =
                                recentered_bounds(universe_body.bounds, scene.local_to_global(body.translation().clone()));
                            if !terrain_overlaps_bounds(&sable, previous_bounds)
                                && terrain_overlaps_bounds(&sable, current_bounds)
                            {
                                body.set_position(*previous, true);
                                body.set_linvel(Vec3::ZERO, true);
                            }
                        }
                    }
                }
                sim.integration_parameters.dt = time_step as marten::Real;
            }
            let mut sable = scene.sable_data.write().unwrap();
            let mut universe = scene.universe.write().unwrap();
            let mut sim = scene.sim_data.write().unwrap();
            let world_origin = *scene.world_origin.read().unwrap();
            sync_active_scene_bodies(&mut sim, &mut sable, &mut universe, world_origin);
            check_scene_evictions(&mut sim, &mut sable, &mut universe, world_origin, scene.gravity);
        });
    }));

    throw_on_panic(&mut env, result);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_hasActiveBodies(
    _env: JNIEnv,
    _class: JClass,
    handle: jlong,
) -> jboolean {
    with_handle(handle, |scene| {
        let sim = scene.sim_data.read().unwrap();
        sim.island_manager.active_bodies().next().is_some() as jboolean
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_ticksUntilNextScheduledBody(
    _env: JNIEnv,
    _class: JClass,
    _handle: jlong,
) -> jlong {
    // Regions are driven by active Rapier simulation islands and universe wake/materialization events.
    // A sleeping region has no self-scheduled wake deadline independent of universe events.
    jlong::MAX
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getPose<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    handle: jni::sys::jlong,
    id: jni::sys::jint,
    store: jni::objects::JDoubleArray<'local>,
) {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let ubody = universe.universe_bodies
            .get(&(id as crate::scene::LevelColliderID))
            .unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let (translation, rotation) = if let Some(resident) = &ubody.resident {
            let rb = &sim_data.rigid_body_set[resident.rigid_body];
            (&scene.local_to_global(rb.translation().clone()), *rb.rotation())
        } else {
            (&ubody.translation, ubody.rotation)
        };

        let mut pose: [jni::sys::jdouble; 7] = [0.0; 7];

        let translation = *translation;
        pose[0] = translation.x as f64;
        pose[1] = translation.y as f64;
        pose[2] = translation.z as f64;

        pose[3] = rotation.x as f64;
        pose[4] = rotation.y as f64;
        pose[5] = rotation.z as f64;
        pose[6] = rotation.w as f64;

        env.set_double_array_region(store, 0, &pose).unwrap();
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setCenterOfMass<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    x: jdouble,
    y: jdouble,
    z: jdouble,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let info = sable_data
            .level_colliders
            .get_mut(&(id as LevelColliderID))
            .unwrap();
        info.center_of_mass = Some(DVec3::new(x, y, z));
        let mut sim_data = scene.sim_data.write().unwrap();
        update_collider_aabb(&mut sim_data, info);
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setLocalBounds<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    min_x: jint,
    min_y: jint,
    min_z: jint,
    max_x: jint,
    max_y: jint,
    max_z: jint,
) {
    with_handle(handle, |scene| {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let SableSceneData {
            level_colliders,
            main_level_chunks,
            ..
        } = &mut *sable_data;

        let info = level_colliders.get_mut(&(id as LevelColliderID)).unwrap();
        info.set_local_bounds(
            IVec3::new(min_x, min_y, min_z),
            IVec3::new(max_x, max_y, max_z),
            main_level_chunks,
            collider_map,
        );
        let mut sim_data = scene.sim_data.write().unwrap();
        update_collider_aabb(&mut sim_data, info);
        let half_extents = Vec3::new(
            (max_x - min_x + 1) as Real * 0.5,
            (max_y - min_y + 1) as Real * 0.5,
            (max_z - min_z + 1) as Real * 0.5,
        );
        if let Some(body) = universe.universe_bodies.get_mut(&(id as LevelColliderID)) {
            // A sphere-derived AABB remains conservative under arbitrary rotation.
            let radius = half_extents.length().max(0.5);
            body.bounds = crate::scene::UniverseAabb::around(body.translation, crate::scene::DVec3::new(radius as f64, radius as f64, radius as f64));
            let bounds = body.bounds;
            universe.spatial_index
                .update(id as LevelColliderID, bounds);
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_canStepInParallel(
    _env: jni::JNIEnv,
    _class: jni::objects::JClass,
    handle: jni::sys::jlong,
) -> jni::sys::jboolean {
    with_handle(handle, |scene| {
        scene.sable_data.read().unwrap().can_parallel_step() as jni::sys::jboolean
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_migrateBody<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    source_handle: jlong,
    destination_handle: jlong,
    id: jint,
) -> jboolean {
    if source_handle == destination_handle {
        return 1;
    }
    if source_handle == 0 || destination_handle == 0 {
        return 0;
    }
    let source = unsafe { &*(source_handle as *const PhysicsScene) };
    let destination = unsafe { &*(destination_handle as *const PhysicsScene) };
    let id = id as LevelColliderID;

    // Both scenes share the same universe Arc<RwLock<DimensionUniverse>>
    let mut universe = source.universe.write().unwrap();
    if !universe.universe_bodies.contains_key(&id) {
        return 0;
    }

    let is_resident_in_source = universe.universe_bodies.get(&id)
        .and_then(|b| b.resident.as_ref())
        .is_some_and(|r| r.scene_handle == source.sable_data.read().unwrap().scene_handle);
    if !is_resident_in_source {
        return 0;
    }

    let source_origin = *source.world_origin.read().unwrap();
    let collider = {
        let mut source_data = source.sable_data.write().unwrap();
        let mut source_sim = source.sim_data.write().unwrap();
        if !evict_rapier_body(&mut source_sim, &mut source_data, &mut universe, source_origin, id, false, false) {
            return 0;
        }
        source_data.level_colliders.remove(&id)
    };

    let destination_origin = *destination.world_origin.read().unwrap();
    let mut destination_data = destination.sable_data.write().unwrap();
    let mut destination_sim = destination.sim_data.write().unwrap();

    if let Some(mut col) = collider {
        col.collider = None;
        destination_data.level_colliders.insert(id, col);
    }

    if let Some(body) = universe.universe_bodies.get_mut(&id) {
        body.simulation_tier = crate::scene::SimulationTier::Active;
        body.schedule_generation = 0;
    }

    instantiate_rapier_body(&mut destination_sim, &mut destination_data, &mut *universe, destination_origin, id);
    let tick = universe.current_tick + ACTIVE_RECHECK_INTERVAL;
    universe.schedule_body(id, tick);
    1
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_mergeScenes<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    source_handle: jni::sys::jlong,
    destination_handle: jni::sys::jlong,
) -> jboolean {
    if source_handle == destination_handle {
        return 1;
    }
    if source_handle == 0 || destination_handle == 0 {
        return 0;
    }

    let source = unsafe { &*(source_handle as *const PhysicsScene) };
    let destination = unsafe { &*(destination_handle as *const PhysicsScene) };
    let source_origin = *source.world_origin.read().unwrap();
    let destination_origin = *destination.world_origin.read().unwrap();

    let mut universe = source.universe.write().unwrap();
    let mut source_data = source.sable_data.write().unwrap();
    let mut source_sim = source.sim_data.write().unwrap();
    let mut destination_data = destination.sable_data.write().unwrap();
    let mut destination_sim = destination.sim_data.write().unwrap();

    if !source_data.rope_map.is_empty()
        || source_data
            .level_colliders
            .values()
            .any(|collider| collider.static_mount.is_some())
    {
        return 0;
    }

    let migrating_joints = source_data.joint_set.take_for_scene_merge(&source_sim);
    let resident_ids: Vec<LevelColliderID> = source_data.rigid_bodies.keys().copied().collect();

    let mut colliders = Vec::with_capacity(resident_ids.len());
    for id in &resident_ids {
        if !evict_rapier_body(&mut source_sim, &mut source_data, &mut universe, source_origin, *id, true, false) {
            return 0;
        }
        if let Some(mut collider) = source_data.level_colliders.remove(id) {
            collider.collider = None;
            colliders.push((*id, collider));
        }
    }

    destination_data.sublevel_serial_callback_sections = destination_data
        .sublevel_serial_callback_sections
        .saturating_add(source_data.sublevel_serial_callback_sections);
    source_data.sublevel_serial_callback_sections = 0;

    for (id, collider) in colliders {
        destination_data.level_colliders.insert(id, collider);
    }

    let destination_wake_tick = universe.current_tick + 1;
    for id in &resident_ids {
        if let Some(body) = universe.universe_bodies.get_mut(id) {
            body.simulation_tier = crate::scene::SimulationTier::Active;
            body.schedule_generation = 0;
        }
        instantiate_rapier_body(&mut destination_sim, &mut destination_data, &mut *universe, destination_origin, *id);
        universe.schedule_body(*id, destination_wake_tick);
    }

    let destination_handles = destination_data.rigid_bodies.clone();
    let destination_ground = destination.ground_handle.unwrap();
    destination_data.joint_set.restore_after_scene_merge(
        migrating_joints,
        &mut destination_sim,
        &destination_handles,
        destination_ground,
    );

    1
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_rebaseRegionOrigin(
    _env: jni::JNIEnv,
    _class: jni::objects::JClass,
    handle: jni::sys::jlong,
    new_x: jni::sys::jdouble,
    new_y: jni::sys::jdouble,
    new_z: jni::sys::jdouble,
) {
    with_handle(handle, |scene| {
        let new_origin = crate::scene::DVec3::new(
            new_x as f64,
            new_y as f64,
            new_z as f64,
        );
        let mut sim = scene.sim_data.write().unwrap();
        let mut sable = scene.sable_data.write().unwrap();
        let old_origin = *scene.world_origin.read().unwrap();
        let delta = new_origin - old_origin;
        if delta.norm_squared() == 0.0 {
            return;
        }
        *scene.world_origin.write().unwrap() = new_origin;

        sable.main_level_chunks.clear();
        sable.octree_chunks.clear();
        sable.terrain_serial_callback_sections = 0;

        let delta_f32 = Vec3::new(delta.x as f32, delta.y as f32, delta.z as f32);
        for (_id, handle) in &sable.rigid_bodies {
            if let Some(rb) = sim.rigid_body_set.get_mut(*handle) {
                let mut pos = rb.translation().clone();
                pos -= delta_f32;
                rb.set_translation(pos, true);
            }
        }
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_createSubLevel<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    handle: jni::sys::jlong,
    id: jni::sys::jint,
    pose: jni::objects::JDoubleArray<'local>,
) {
    let mut pose_arr: [jni::sys::jdouble; 7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    env.get_double_array_region(pose, 0, &mut pose_arr).unwrap();

    let quat = rapier3d::math::Rotation::from_xyzw(
        pose_arr[3] as marten::Real,
        pose_arr[4] as marten::Real,
        pose_arr[5] as marten::Real,
        pose_arr[6] as marten::Real,
    );

    let global_translation = crate::scene::DVec3::new(
        pose_arr[0] as f64,
        pose_arr[1] as f64,
        pose_arr[2] as f64,
    );

    with_handle(handle, |scene| {
        let translation = global_translation;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();

        let ubody = crate::scene::UniverseBody {
            id: id as crate::scene::LevelColliderID,
            translation,
            rotation: quat,
            linear_velocity: rapier3d::math::Vec3::ZERO,
            angular_velocity: rapier3d::math::Vec3::ZERO,
            dynamics: crate::scene::BodyDynamics {
                additional_mass_properties: None,
                linear_damping: scene.universal_drag,
                angular_damping: scene.universal_drag,
                gravity_scale: 1.0,
                locked_axes: LockedAxes::empty(),
                ccd_enabled: false,
            },
            simulation_tier: crate::scene::SimulationTier::Active,
            bounds: crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(1.0, 1.0, 1.0)),
            last_update_tick: 0,
            next_update_tick: 0,
            schedule_generation: 0,
            resident: None,
            assembly_root: id as crate::scene::LevelColliderID,
            assembly_size: 1,
            command_queue: Vec::new(),
        };
        universe.universe_bodies
            .insert(id as crate::scene::LevelColliderID, ubody);
        universe.spatial_index.update(
            id as crate::scene::LevelColliderID,
            crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(1.0, 1.0, 1.0)),
        );
        universe.schedule_body(id as crate::scene::LevelColliderID, 20);

        let mut sim_data = scene.sim_data.write().unwrap();
        crate::instantiate_rapier_body(
            &mut sim_data,
            &mut sable_data, &mut *universe,
            *scene.world_origin.read().unwrap(),
            id as crate::scene::LevelColliderID,
        );
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeSubLevel<'local>(
    mut _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    handle: jni::sys::jlong,
    id: jni::sys::jint,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let id_usize = id as crate::scene::LevelColliderID;
        evict_rapier_body(&mut sim_data, &mut sable_data, &mut universe, *scene.world_origin.read().unwrap(), id_usize, true, false);
        universe.spatial_index.remove(id_usize);
        universe.universe_bodies.remove(&id_usize);
        sable_data.level_colliders.remove(&id_usize);
    })
}

pub fn insert_block_octree(
    collider_map: &VoxelColliderMap,
    octree: &mut SubLevelOctree,
    state: &BlockState,
    remove: bool,
    x: i32,
    y: i32,
    z: i32,
) {
    let block_collider_id = state.0;
    let block_collider = if block_collider_id > 0 {
        collider_map
            .voxel_colliders
            .get(block_collider_id as usize - 1)
            .and_then(|opt| opt.as_ref())
    } else {
        None
    };
    let voxel_state = state.1;

    let has_collision = if let Some(collider) = block_collider {
        !collider.collision_boxes.is_empty()
    } else {
        true
    };

    let solid = voxel_state != Interior
        && voxel_state != VoxelPhysicsState::Empty
        && (block_collider_id > 0 && has_collision);

    if remove && !solid {
        octree.insert(x, y, z, -1);
    }

    if solid {
        octree.insert(x, y, z, block_collider_id as i32);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addChunk<'local>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    x: jint,
    y: jint,
    z: jint,
    data: JIntArray<'local>,
    global: jboolean,
    object_id: jint,
) {
    let mut ints: [jint; 4096] = [0; 4096];
    env.get_int_array_region(data, 0, &mut ints).unwrap();

    let mut blocks = Vec::with_capacity(ints.len());

    for block in ints {
        // split it in half
        let block_collider_id = (block >> 16) as u16;
        let voxel_state_id = (block & 0xFFFF) as u16;

        blocks.push((
            block_collider_id as u32,
            ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
        ));
    }

    let has_solid_blocks = blocks.iter().any(|block| block.0 != 0);
    let chunk_serial_callback_blocks = {
        let physics_state = get_physics_state();
        blocks
            .iter()
            .filter(|block| {
                physics_state
                    .voxel_collider_map
                    .requires_java_callback(block.0 as usize)
            })
            .count() as u16
    };
    let chunk = ChunkSection::with_serial_step(blocks, chunk_serial_callback_blocks);

    with_handle(handle, |scene| {
        let origin_section = scene.origin_section();
        let (local_x, local_y, local_z) = if global > 0 {
            (
                x - origin_section.x,
                y - origin_section.y,
                z - origin_section.z,
            )
        } else {
            (x, y, z)
        };
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let SableSceneData {
            main_level_chunks,
            level_colliders,
            octree_chunks,
            terrain_serial_callback_sections,
            sublevel_serial_callback_sections,
            ..
        } = &mut *sable_data;

        if global == 0 {
            if object_id != -1 {
                let body = level_colliders
                    .get_mut(&(object_id as LevelColliderID))
                    .unwrap();
                if body.chunk_map.is_none() {
                    body.chunk_map = Some(HashMap::new());
                }
                body.insert_chunk(&chunk, local_x, local_y, local_z, collider_map);
                let key = pack_section_pos(local_x, local_y, local_z);
                if let Some(old) = body.chunk_map.as_mut().unwrap().insert(key, chunk) {
                    if old.serial_callback_blocks > 0 {
                        *sublevel_serial_callback_sections =
                            sublevel_serial_callback_sections.saturating_sub(1);
                    }
                }
                if chunk_serial_callback_blocks > 0 {
                    *sublevel_serial_callback_sections += 1;
                }
                body.mark_section_dirty(local_x, local_y, local_z);
            }
        } else {
            let key = pack_section_pos(local_x, local_y, local_z);
            if has_solid_blocks {
                universe.terrain_sections.insert(pack_section_pos(x, y, z));
            }
            if let Some(old) = main_level_chunks.insert(key, chunk) {
                if old.serial_callback_blocks > 0 {
                    *terrain_serial_callback_sections =
                        terrain_serial_callback_sections.saturating_sub(1);
                }
            }
            if chunk_serial_callback_blocks > 0 {
                *terrain_serial_callback_sections += 1;
            }
            let chunk = main_level_chunks.get(&key).unwrap();
            for bx in 0..16 {
                for by in 0..16 {
                    for bz in 0..16 {
                        let block = chunk.get_block(bx, by, bz);
                        let x = bx + (local_x << CHUNK_SHIFT);
                        let y = by + (local_y << CHUNK_SHIFT);
                        let z = bz + (local_z << CHUNK_SHIFT);

                        // insert into level octree
                        let ox = x >> OCTREE_CHUNK_SHIFT;
                        let oy = y >> OCTREE_CHUNK_SHIFT;
                        let oz = z >> OCTREE_CHUNK_SHIFT;

                        let mut octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));

                        if octree_chunk.is_none() {
                            octree_chunks
                                .insert(pack_section_pos(ox, oy, oz), OctreeChunkSection::new());
                            octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));
                        }

                        let Some(octree_chunk) = octree_chunk else {
                            panic!("No octree chunk!")
                        };

                        if block.0 == 0 {
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.liquid_octree,
                                &block,
                                false,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.octree,
                                &block,
                                false,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                        } else {
                            if collider_map.voxel_colliders[(block.0 - 1) as usize]
                                .as_ref()
                                .unwrap()
                                .is_fluid
                            {
                                insert_block_octree(
                                    collider_map,
                                    &mut octree_chunk.liquid_octree,
                                    &block,
                                    false,
                                    x & (OCTREE_CHUNK_SIZE - 1),
                                    y & (OCTREE_CHUNK_SIZE - 1),
                                    z & (OCTREE_CHUNK_SIZE - 1),
                                );
                            } else {
                                insert_block_octree(
                                    collider_map,
                                    &mut octree_chunk.octree,
                                    &block,
                                    false,
                                    x & (OCTREE_CHUNK_SIZE - 1),
                                    y & (OCTREE_CHUNK_SIZE - 1),
                                    z & (OCTREE_CHUNK_SIZE - 1),
                                );
                            }
                        }
                    }
                }
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeChunk<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    x: jint,
    y: jint,
    z: jint,
    global: jboolean,
    object_id: jint,
) {
    with_handle(handle, |scene| {
        let origin_section = scene.origin_section();
        let (local_x, local_y, local_z) = if global > 0 {
            (
                x - origin_section.x,
                y - origin_section.y,
                z - origin_section.z,
            )
        } else {
            (x, y, z)
        };
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();

        if global > 0 {
            universe.terrain_sections.remove(&pack_section_pos(x, y, z));
            if let Some(old) = sable_data
                .main_level_chunks
                .remove(&pack_section_pos(local_x, local_y, local_z))
            {
                if old.serial_callback_blocks > 0 {
                    sable_data.terrain_serial_callback_sections = sable_data
                        .terrain_serial_callback_sections
                        .saturating_sub(1);
                }
            }
            let octree_chunk = sable_data.octree_chunks.get_mut(&pack_section_pos(
                (local_x << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                (local_y << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                (local_z << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
            ));

            if let Some(octree_chunk) = octree_chunk {
                for bx in 0..16 {
                    for by in 0..16 {
                        for bz in 0..16 {
                            let x = bx + (local_x << CHUNK_SHIFT);
                            let y = by + (local_y << CHUNK_SHIFT);
                            let z = bz + (local_z << CHUNK_SHIFT);

                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.octree,
                                &(0, VoxelPhysicsState::Empty),
                                true,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.liquid_octree,
                                &(0, VoxelPhysicsState::Empty),
                                true,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                        }
                    }
                }

                if octree_chunk.octree.buffer[0] == 0 && octree_chunk.liquid_octree.buffer[0] == 0 {
                    sable_data.octree_chunks.remove(&pack_section_pos(
                        (local_x << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                        (local_y << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                        (local_z << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                    ));
                }
            }
        } else if object_id != -1 {
            let SableSceneData {
                level_colliders,
                sublevel_serial_callback_sections,
                ..
            } = &mut *sable_data;
            let Some(body) = level_colliders.get_mut(&(object_id as LevelColliderID)) else {
                return;
            };
            let removed_chunk = body
                .chunk_map
                .as_mut()
                .and_then(|chunks| chunks.remove(&pack_section_pos(x, y, z)));
            if let Some(old) = removed_chunk {
                if old.serial_callback_blocks > 0 {
                    *sublevel_serial_callback_sections =
                        sublevel_serial_callback_sections.saturating_sub(1);
                }
                let empty = (0, VoxelPhysicsState::Empty);
                for bx in 0..16 {
                    for by in 0..16 {
                        for bz in 0..16 {
                            body.insert_block(
                                bx + (x << CHUNK_SHIFT),
                                by + (y << CHUNK_SHIFT),
                                bz + (z << CHUNK_SHIFT),
                                &empty,
                                true,
                                collider_map,
                            );
                        }
                    }
                }
                body.mark_section_dirty(x, y, z);
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_changeBlock<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    x: jint,
    y: jint,
    z: jint,
    block: jint,
) {
    let block_collider_id = (block >> 16) as u16;
    let voxel_state_id = (block & 0xFFFF) as u16;

    with_handle(handle, |scene| {
        let origin_d = scene.world_origin.read().unwrap();
        let origin = rapier3d::glamx::IVec3::new(origin_d.x as i32, origin_d.y as i32, origin_d.z as i32);
        let x = x - origin.x;
        let y = y - origin.y;
        let z = z - origin.z;
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let SableSceneData {
            main_level_chunks,
            level_colliders,
            octree_chunks,
            ..
        } = &mut *sable_data;

        let chunk = main_level_chunks.get_mut(&pack_section_pos(x >> 4, y >> 4, z >> 4));

        if let Some(chunk) = chunk {
            let block_state = (
                block_collider_id as u32,
                ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
            );

            chunk.set_block(x & 15, y & 15, z & 15, block_state);

            let mut any = false;
            for (_, sable_body) in level_colliders.iter_mut() {
                if sable_body.contains(x, y, z) {
                    sable_body.insert_block(x, y, z, &block_state, true, collider_map);
                    any = true;
                    break;
                }
            }

            if !any {
                // insert into level octree
                let ox = x >> OCTREE_CHUNK_SHIFT;
                let oy = y >> OCTREE_CHUNK_SHIFT;
                let oz = z >> OCTREE_CHUNK_SHIFT;

                let mut octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));

                if octree_chunk.is_none() {
                    octree_chunks.insert(pack_section_pos(ox, oy, oz), OctreeChunkSection::new());
                    octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));
                }

                let Some(octree_chunk) = octree_chunk else {
                    panic!("No octree chunk!")
                };

                if block_collider_id == 0 {
                    insert_block_octree(
                        collider_map,
                        &mut octree_chunk.octree,
                        &block_state,
                        true,
                        x & (OCTREE_CHUNK_SIZE - 1),
                        y & (OCTREE_CHUNK_SIZE - 1),
                        z & (OCTREE_CHUNK_SIZE - 1),
                    );
                    insert_block_octree(
                        collider_map,
                        &mut octree_chunk.liquid_octree,
                        &block_state,
                        true,
                        x & (OCTREE_CHUNK_SIZE - 1),
                        y & (OCTREE_CHUNK_SIZE - 1),
                        z & (OCTREE_CHUNK_SIZE - 1),
                    );
                } else {
                    if collider_map
                        .voxel_colliders
                        .get(block_collider_id as usize - 1)
                        .unwrap()
                        .as_ref()
                        .unwrap()
                        .is_fluid
                    {
                        insert_block_octree(
                            collider_map,
                            &mut octree_chunk.liquid_octree,
                            &block_state,
                            false,
                            x & (OCTREE_CHUNK_SIZE - 1),
                            y & (OCTREE_CHUNK_SIZE - 1),
                            z & (OCTREE_CHUNK_SIZE - 1),
                        );
                    } else {
                        insert_block_octree(
                            collider_map,
                            &mut octree_chunk.octree,
                            &block_state,
                            false,
                            x & (OCTREE_CHUNK_SIZE - 1),
                            y & (OCTREE_CHUNK_SIZE - 1),
                            z & (OCTREE_CHUNK_SIZE - 1),
                        );
                    }
                }
            }
        }
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_changeWorldBlock<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    x: jint,
    y: jint,
    z: jint,
    block: jint,
) {
    let block_collider_id = (block >> 16) as u16;
    let voxel_state_id = (block & 0xFFFF) as u16;

    with_handle(handle, |scene| {
        let origin_d = scene.world_origin.read().unwrap();
        let origin = rapier3d::glamx::IVec3::new(origin_d.x as i32, origin_d.y as i32, origin_d.z as i32);
        let x = x - origin.x;
        let y = y - origin.y;
        let z = z - origin.z;
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let SableSceneData {
            main_level_chunks,
            octree_chunks,
            terrain_serial_callback_sections,
            ..
        } = &mut *sable_data;

        let chunk = main_level_chunks.get_mut(&pack_section_pos(x >> 4, y >> 4, z >> 4));

        if let Some(chunk) = chunk {
            let block_state = (
                block_collider_id as u32,
                ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
            );

            let old_state = chunk.set_block(x & 15, y & 15, z & 15, block_state);
            let old_requires_callback = collider_map.requires_java_callback(old_state.0 as usize);
            let new_requires_callback = collider_map.requires_java_callback(block_state.0 as usize);

            if old_requires_callback != new_requires_callback {
                if new_requires_callback {
                    if chunk.serial_callback_blocks == 0 {
                        *terrain_serial_callback_sections += 1;
                    }
                    chunk.serial_callback_blocks += 1;
                } else {
                    chunk.serial_callback_blocks -= 1;
                    if chunk.serial_callback_blocks == 0 {
                        *terrain_serial_callback_sections =
                            terrain_serial_callback_sections.saturating_sub(1);
                    }
                }
            }

            // insert into level octree
            let ox = x >> OCTREE_CHUNK_SHIFT;
            let oy = y >> OCTREE_CHUNK_SHIFT;
            let oz = z >> OCTREE_CHUNK_SHIFT;

            let mut octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));

            if octree_chunk.is_none() {
                octree_chunks.insert(pack_section_pos(ox, oy, oz), OctreeChunkSection::new());
                octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));
            }

            let Some(octree_chunk) = octree_chunk else {
                panic!("No octree chunk!")
            };

            if block_collider_id == 0 {
                insert_block_octree(
                    collider_map,
                    &mut octree_chunk.octree,
                    &block_state,
                    true,
                    x & (OCTREE_CHUNK_SIZE - 1),
                    y & (OCTREE_CHUNK_SIZE - 1),
                    z & (OCTREE_CHUNK_SIZE - 1),
                );
                insert_block_octree(
                    collider_map,
                    &mut octree_chunk.liquid_octree,
                    &block_state,
                    true,
                    x & (OCTREE_CHUNK_SIZE - 1),
                    y & (OCTREE_CHUNK_SIZE - 1),
                    z & (OCTREE_CHUNK_SIZE - 1),
                );
            } else {
                let liquid = collider_map
                    .voxel_colliders
                    .get(block_collider_id as usize - 1)
                    .unwrap()
                    .as_ref()
                    .unwrap()
                    .is_fluid;

                insert_block_octree(
                    collider_map,
                    if liquid {
                        &mut octree_chunk.liquid_octree
                    } else {
                        &mut octree_chunk.octree
                    },
                    &block_state,
                    false,
                    x & (OCTREE_CHUNK_SIZE - 1),
                    y & (OCTREE_CHUNK_SIZE - 1),
                    z & (OCTREE_CHUNK_SIZE - 1),
                );
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_changeSubLevelBlock<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    x: jint,
    y: jint,
    z: jint,
    block: jint,
    sub_level_id: jint,
) {
    let block_collider_id = (block >> 16) as u16;
    let voxel_state_id = (block & 0xFFFF) as u16;

    with_handle(handle, |scene| {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let SableSceneData {
            level_colliders,
            sublevel_serial_callback_sections,
            ..
        } = &mut *sable_data;

        if let Some(sable_body) = level_colliders.get_mut(&(sub_level_id as LevelColliderID)) {
            let chunk_x = x >> CHUNK_SHIFT;
            let chunk_y = y >> CHUNK_SHIFT;
            let chunk_z = z >> CHUNK_SHIFT;
            let block_state = (
                block_collider_id as u32,
                ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
            );

            // Live assembly starts with an empty plot chunk. Empty sections are
            // not uploaded through addChunk, so the first per-block change must
            // create native section storage instead of being silently dropped.
            let chunk_key = pack_section_pos(chunk_x, chunk_y, chunk_z);
            let chunk = sable_body
                .chunk_map
                .get_or_insert_with(HashMap::new)
                .entry(chunk_key)
                .or_insert_with(|| {
                    ChunkSection::new(vec![
                        (0, VoxelPhysicsState::Empty);
                        (1_usize << CHUNK_SHIFT).pow(3)
                    ])
                });
            {
                let old_state = chunk.set_block(x & 15, y & 15, z & 15, block_state);
                let old_requires_callback =
                    collider_map.requires_java_callback(old_state.0 as usize);
                let new_requires_callback =
                    collider_map.requires_java_callback(block_state.0 as usize);

                if old_requires_callback != new_requires_callback {
                    if new_requires_callback {
                        if chunk.serial_callback_blocks == 0 {
                            *sublevel_serial_callback_sections += 1;
                        }
                        chunk.serial_callback_blocks += 1;
                    } else {
                        chunk.serial_callback_blocks -= 1;
                        if chunk.serial_callback_blocks == 0 {
                            *sublevel_serial_callback_sections =
                                sublevel_serial_callback_sections.saturating_sub(1);
                        }
                    }
                }
            }
            if sable_body.contains(x, y, z) {
                sable_body.insert_block(x, y, z, &block_state, true, collider_map);
            }
            sable_body.mark_section_dirty(chunk_x, chunk_y, chunk_z);
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setMassProperties<
    'local,
>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    mass: jdouble,
    center_of_mass: JDoubleArray<'local>,
    inertia: JDoubleArray<'local>,
) {
    let mut com: [jdouble; 3] = [0.0, 0.0, 0.0];
    env.get_double_array_region(center_of_mass, 0, &mut com)
        .unwrap();

    let mut inertia_arr: [jdouble; 9] = [0.0; 9];
    env.get_double_array_region(inertia, 0, &mut inertia_arr)
        .unwrap();

    let inertia_tensor = Mat3::from_cols(
        Vec3::new(
            inertia_arr[0] as Real,
            inertia_arr[1] as Real,
            inertia_arr[2] as Real,
        ),
        Vec3::new(
            inertia_arr[3] as Real,
            inertia_arr[4] as Real,
            inertia_arr[5] as Real,
        ),
        Vec3::new(
            inertia_arr[6] as Real,
            inertia_arr[7] as Real,
            inertia_arr[8] as Real,
        ),
    );

    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let properties =
            MassProperties::with_inertia_matrix(Vec3::ZERO, mass as Real, inertia_tensor.into());
        let ubody = universe.universe_bodies
            .get_mut(&(id as LevelColliderID))
            .expect("No universe body for id");
        ubody.dynamics.additional_mass_properties = Some(properties.clone());

        if let Some(resident) = &ubody.resident {
            sim_data.rigid_body_set[resident.rigid_body]
                .set_additional_mass_properties(properties, true);
        }
    })
}

/// Teleports the object to the given position.
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_teleportObject<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    x: jdouble,
    y: jdouble,
    z: jdouble,
    i: jdouble,
    j: jdouble,
    k: jdouble,
    r: jdouble,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let ubody = universe.universe_bodies
            .get_mut(&(id as LevelColliderID))
            .expect("No universe body for id");
        ubody.translation = crate::scene::DVec3::new(x as f64, y as f64, z as f64);
        ubody.rotation = Quat::from_xyzw(i as Real, j as Real, k as Real, r as Real);
        ubody.bounds = recentered_bounds(ubody.bounds, ubody.translation);
        let bounds = ubody.bounds;

        if let Some(resident) = &ubody.resident {
            let rb = &mut sim_data.rigid_body_set[resident.rigid_body];
            let mut pose = *rb.position();
            pose.translation = Vec3::new(ubody.translation.x as f32, ubody.translation.y as f32, ubody.translation.z as f32);
            pose.rotation = ubody.rotation;
            rb.set_position(pose, true);
        }
        universe.spatial_index
            .update(id as LevelColliderID, bounds);
        let tick = universe.current_tick + 1;
        universe.schedule_body(id as LevelColliderID, tick);
    })
}

/// Wakes up an object.
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_wakeUpObject<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
) {
    with_universe(handle, |universe| {
        if let Some(body) = universe.universe_bodies.get_mut(&(id as crate::scene::LevelColliderID)) {
            body.command_queue.push(crate::scene::UniverseCommand::WakeUp);
        }
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_wakeUpRegionObject<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
) {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let id = id as crate::scene::LevelColliderID;
        if let Some(handle) = sable_data.rigid_bodies.get(&id).copied() {
            sim_data.rigid_body_set[handle].wake_up(true);
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addLinearAngularVelocities<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    linear_x: jdouble,
    linear_y: jdouble,
    linear_z: jdouble,
    angular_x: jdouble,
    angular_y: jdouble,
    angular_z: jdouble,
    wake_up: jboolean,
) {
    with_universe(handle, |universe| {
        if let Some(body) = universe.universe_bodies.get_mut(&(id as crate::scene::LevelColliderID)) {
            body.command_queue.push(crate::scene::UniverseCommand::AddLinearAngularVelocities {
                linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, wake_up: wake_up != 0,
            });
        }
    });
}

/// Clears & queries all collisions
///
/// TODO: Do not pass body IDs as doubles, stupid as hell lmao
///
/// A collision is formatted as follows:
/// [body_a, body_b, force_amount, local_normal_a, local_normal_b, local_point_a, local_point_b]
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_clearCollisions<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
) -> JDoubleArray<'local> {
    let arr: Vec<jdouble> = with_handle(handle, |scene| {
        let mut reported = scene.reported_collisions.lock();

        let max_collisions = 100;

        reported.truncate(max_collisions);
        let mut arr: Vec<jdouble> = Vec::with_capacity(reported.len() * 15);

        for collision in reported.iter() {
            let body_a = if let Some(id) = collision.body_a {
                id as jdouble
            } else {
                -1.0
            };

            let body_b = if let Some(id) = collision.body_b {
                id as jdouble
            } else {
                -1.0
            };

            arr.push(body_a);
            arr.push(body_b);
            arr.push(collision.force_amount as jdouble);
            arr.push(collision.local_normal_a.x as jdouble);
            arr.push(collision.local_normal_a.y as jdouble);
            arr.push(collision.local_normal_a.z as jdouble);
            arr.push(collision.local_normal_b.x as jdouble);
            arr.push(collision.local_normal_b.y as jdouble);
            arr.push(collision.local_normal_b.z as jdouble);
            arr.push(collision.local_point_a.x as jdouble);
            arr.push(collision.local_point_a.y as jdouble);
            arr.push(collision.local_point_a.z as jdouble);
            arr.push(collision.local_point_b.x as jdouble);
            arr.push(collision.local_point_b.y as jdouble);
            arr.push(collision.local_point_b.z as jdouble);
        }

        reported.clear();

        arr
    });

    let double_array = _env.new_double_array(arr.len() as jint).unwrap();
    _env.set_double_array_region(&double_array, 0, &arr)
        .unwrap();

    double_array
}

/// Applies a force to a body
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_applyForce<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    x: jdouble,
    y: jdouble,
    z: jdouble,
    fx: jdouble,
    fy: jdouble,
    fz: jdouble,
    wake_up: jboolean,
) {
    with_universe(handle, |universe| {
        if let Some(body) = universe.universe_bodies.get_mut(&(id as crate::scene::LevelColliderID)) {
            body.command_queue.push(crate::scene::UniverseCommand::ApplyForce {
                x, y, z, fx, fy, fz, wake_up: wake_up != 0,
            });
        }
    });
}

/// Applies a force and torque
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_applyForceAndTorque<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    fx: jdouble,
    fy: jdouble,
    fz: jdouble,
    tx: jdouble,
    ty: jdouble,
    tz: jdouble,
    wake_up: jboolean,
) {
    with_universe(handle, |universe| {
        if let Some(body) = universe.universe_bodies.get_mut(&(id as crate::scene::LevelColliderID)) {
            body.command_queue.push(crate::scene::UniverseCommand::ApplyForceAndTorque {
                fx, fy, fz, tx, ty, tz, wake_up: wake_up != 0,
            });
        }
    });
}

/// Gets the linear velocity of a body
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getLinearVelocity<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    store: JDoubleArray<'local>,
) {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let body = universe.universe_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let vel = body
            .resident
            .as_ref()
            .map_or(body.linear_velocity, |resident| {
                sim_data.rigid_body_set[resident.rigid_body].linvel()
            });

        _env.set_double_array_region(
            &store,
            0,
            &[vel.x as jdouble, vel.y as jdouble, vel.z as jdouble],
        )
        .unwrap();
    })
}

/// Gets the angular velocity of a body
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getAngularVelocity<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jint,
    store: JDoubleArray<'local>,
) {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let body = universe.universe_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let vel = body
            .resident
            .as_ref()
            .map_or(body.angular_velocity, |resident| {
                sim_data.rigid_body_set[resident.rigid_body].angvel()
            });

        _env.set_double_array_region(
            &store,
            0,
            &[vel.x as jdouble, vel.y as jdouble, vel.z as jdouble],
        )
        .unwrap();
    })
}
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_writeActivePoses<
    'local,
>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    buffer: jni::objects::JObject<'local>,
    max_bodies: jint,
) -> jint {
    let capacity = env
        .get_direct_buffer_capacity((&buffer).into())
        .unwrap_or(0);
    let data_ptr = env
        .get_direct_buffer_address((&buffer).into())
        .unwrap_or(std::ptr::null_mut());

    if data_ptr.is_null() || capacity == 0 {
        return 0;
    }

    let max_allowed = std::cmp::min(max_bodies.max(0) as usize, capacity / 60);

    with_handle(handle, |scene| {
        let poses = match collect_active_poses(scene, max_allowed) {
            Ok(poses) => poses,
            Err(required) => return -(required.min(jint::MAX as usize) as jint),
        };
        let output = unsafe { std::slice::from_raw_parts_mut(data_ptr, poses.len() * 60) };
        encode_active_poses(&poses, output);
        poses.len() as jint
    }) as jint
}

pub struct ExportPose {
    pub id: i32,
    pub position: crate::scene::DVec3,
    pub rotation: rapier3d::glamx::Quat,
}

fn collect_active_poses(
    scene: &PhysicsScene,
    max_allowed: usize,
) -> Result<Vec<ExportPose>, usize> {
    let sim_data = scene.sim_data.read().unwrap();
    let active_handles: Vec<_> = sim_data.island_manager.active_bodies().collect();
    let required = active_handles.len();
    if required > max_allowed {
        return Err(required);
    }

    let mut poses = Vec::with_capacity(required);
    for handle in active_handles {
        let rb = &sim_data.rigid_body_set[handle];
        poses.push(ExportPose {
            id: rb.user_data as i32,
            position: scene.local_to_global(rb.translation().clone()),
            rotation: *rb.rotation(),
        });
    }
    Ok(poses)
}

fn encode_active_poses(poses: &[ExportPose], output: &mut [u8]) {
    for (index, pose) in poses.iter().enumerate() {
        let record = &mut output[index * 60..(index + 1) * 60];
        record[0..4].copy_from_slice(&pose.id.to_ne_bytes());
        let doubles: [f64; 7] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.rotation.x as f64,
            pose.rotation.y as f64,
            pose.rotation.z as f64,
            pose.rotation.w as f64,
        ];
        for (slot, value) in doubles.into_iter().enumerate() {
            let offset = 4 + slot * 8;
            record[offset..offset + 8].copy_from_slice(&value.to_ne_bytes());
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_processCommands<
    'local,
>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    handle: jni::sys::jlong,
    buffer: jni::objects::JByteBuffer<'local>,
    length: jni::sys::jint,
) {
    if length < COMMAND_HEADER_SIZE as jint {
        log::error!("Rejected physics command buffer shorter than its header");
        return;
    }

    let capacity = env.get_direct_buffer_capacity(&buffer).unwrap_or(0);
    if length as usize > capacity {
        log::error!(
            "Rejected physics command buffer length {} larger than capacity {}",
            length,
            capacity
        );
        return;
    }

    let buffer_ptr = env.get_direct_buffer_address(&buffer).unwrap();
    let data = unsafe { std::slice::from_raw_parts(buffer_ptr as *const u8, length as usize) };

    let command_count = match validate_command_header(data) {
        Ok(command_count) => command_count,
        Err(reason) => {
            log::error!("Rejected physics command header: {}", reason);
            return;
        }
    };

    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let mut offset = COMMAND_HEADER_SIZE;
        let mut commands_read = 0;
        while commands_read < command_count {
            if offset >= data.len() {
                log::error!("Rejected physics command buffer ending before all declared commands");
                break;
            }
            let cmd = data[offset];
            offset += 1;

            let payload_length = match cmd {
                1..=3 => 53,
                4 => 4,
                _ => {
                    log::error!("Rejected unknown physics command opcode {}", cmd);
                    break;
                }
            };
            if data.len().saturating_sub(offset) < payload_length {
                log::error!(
                    "Rejected truncated physics command {}: expected {} payload bytes, found {}",
                    cmd,
                    payload_length,
                    data.len().saturating_sub(offset)
                );
                break;
            }

            match cmd {
                1 => {
                    // applyImpulse
                    let id = i32::from_ne_bytes(data[offset..offset + 4].try_into().unwrap());
                    offset += 4;
                    let px = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let py = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let pz = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let fx = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let fy = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let fz = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let wake_up = data[offset] != 0;
                    offset += 1;

                    if wake_up {
                        instantiate_rapier_body(
                            &mut sim_data,
                            &mut sable_data, &mut *universe,
                            *scene.world_origin.read().unwrap(),
                            id as crate::scene::LevelColliderID,
                        );
                        let body = universe.universe_bodies
                            .get_mut(&(id as LevelColliderID))
                            .unwrap();
                        body.simulation_tier = crate::scene::SimulationTier::Critical;
                        body.dynamics.ccd_enabled = true;
                        let tick = universe.current_tick + CRITICAL_RECHECK_INTERVAL;
                        universe.schedule_body(id as LevelColliderID, tick);
                    }
                    if let Some(resident) = &universe.universe_bodies
                        .get(&(id as crate::scene::LevelColliderID))
                        .unwrap()
                        .resident
                    {
                        let rb = &mut sim_data.rigid_body_set[resident.rigid_body];
                        if wake_up {
                            rb.enable_ccd(true);
                        }
                        if wake_up || !rb.is_sleeping() {
                            rb.apply_impulse_at_point(
                                rapier3d::math::Vec3::new(
                                    fx as marten::Real,
                                    fy as marten::Real,
                                    fz as marten::Real,
                                ),
                                rapier3d::math::Vec3::new(
                                    px as marten::Real,
                                    py as marten::Real,
                                    pz as marten::Real,
                                ),
                                wake_up,
                            );
                        }
                    }
                }
                2 => {
                    // applyLinearAndAngularImpulse
                    let id = i32::from_ne_bytes(data[offset..offset + 4].try_into().unwrap());
                    offset += 4;
                    let fx = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let fy = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let fz = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let tx = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let ty = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let tz = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let wake_up = data[offset] != 0;
                    offset += 1;

                    if wake_up {
                        instantiate_rapier_body(
                            &mut sim_data,
                            &mut sable_data, &mut *universe,
                            *scene.world_origin.read().unwrap(),
                            id as crate::scene::LevelColliderID,
                        );
                        let body = universe.universe_bodies
                            .get_mut(&(id as LevelColliderID))
                            .unwrap();
                        body.simulation_tier = crate::scene::SimulationTier::Critical;
                        body.dynamics.ccd_enabled = true;
                        let tick = universe.current_tick + CRITICAL_RECHECK_INTERVAL;
                        universe.schedule_body(id as LevelColliderID, tick);
                    }
                    if let Some(resident) = &universe.universe_bodies
                        .get(&(id as crate::scene::LevelColliderID))
                        .unwrap()
                        .resident
                    {
                        let rb = &mut sim_data.rigid_body_set[resident.rigid_body];
                        if wake_up {
                            rb.enable_ccd(true);
                        }
                        if wake_up || !rb.is_sleeping() {
                            rb.apply_impulse(
                                rapier3d::math::Vec3::new(
                                    fx as marten::Real,
                                    fy as marten::Real,
                                    fz as marten::Real,
                                ),
                                wake_up,
                            );
                            rb.apply_torque_impulse(
                                rapier3d::math::Vec3::new(
                                    tx as marten::Real,
                                    ty as marten::Real,
                                    tz as marten::Real,
                                ),
                                wake_up,
                            );
                        }
                    }
                }
                3 => {
                    // addLinearAndAngularVelocity
                    let id = i32::from_ne_bytes(data[offset..offset + 4].try_into().unwrap());
                    offset += 4;
                    let vx = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let vy = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let vz = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let ax = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let ay = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let az = f64::from_ne_bytes(data[offset..offset + 8].try_into().unwrap());
                    offset += 8;
                    let wake_up = data[offset] != 0;
                    offset += 1;

                    if wake_up {
                        instantiate_rapier_body(
                            &mut sim_data,
                            &mut sable_data, &mut *universe,
                            *scene.world_origin.read().unwrap(),
                            id as crate::scene::LevelColliderID,
                        );
                        let body = universe.universe_bodies
                            .get_mut(&(id as LevelColliderID))
                            .unwrap();
                        body.simulation_tier = crate::scene::SimulationTier::Critical;
                        body.dynamics.ccd_enabled = true;
                        let tick = universe.current_tick + CRITICAL_RECHECK_INTERVAL;
                        universe.schedule_body(id as LevelColliderID, tick);
                    }
                    if let Some(resident) = &universe.universe_bodies
                        .get(&(id as crate::scene::LevelColliderID))
                        .unwrap()
                        .resident
                    {
                        let rb = &mut sim_data.rigid_body_set[resident.rigid_body];
                        if wake_up {
                            rb.enable_ccd(true);
                        }
                        if wake_up || !rb.is_sleeping() {
                            rb.set_linvel(
                                rb.linvel()
                                    + rapier3d::math::Vec3::new(
                                        vx as marten::Real,
                                        vy as marten::Real,
                                        vz as marten::Real,
                                    ),
                                wake_up,
                            );
                            rb.set_angvel(
                                rb.angvel()
                                    + rapier3d::math::Vec3::new(
                                        ax as marten::Real,
                                        ay as marten::Real,
                                        az as marten::Real,
                                    ),
                                wake_up,
                            );
                        }
                    }
                }
                4 => {
                    // wakeUp
                    let id = i32::from_ne_bytes(data[offset..offset + 4].try_into().unwrap());
                    offset += 4;

                    instantiate_rapier_body(
                        &mut sim_data,
                        &mut sable_data, &mut *universe,
                        *scene.world_origin.read().unwrap(),
                        id as crate::scene::LevelColliderID,
                    );
                    universe.universe_bodies
                        .get_mut(&(id as LevelColliderID))
                        .unwrap()
                        .simulation_tier = crate::scene::SimulationTier::Active;
                    let tick = universe.current_tick + ACTIVE_RECHECK_INTERVAL;
                    universe.schedule_body(id as LevelColliderID, tick);
                    if let Some(resident) = &universe.universe_bodies
                        .get(&(id as crate::scene::LevelColliderID))
                        .unwrap()
                        .resident
                    {
                        let rb = &mut sim_data.rigid_body_set[resident.rigid_body];
                        rb.wake_up(true);
                    }
                }
                _ => unreachable!("opcode was validated before decoding"),
            }
            commands_read += 1;
        }

        if commands_read != command_count || offset != data.len() {
            log::error!(
                "Physics command buffer was not consumed exactly: commands {}/{}, bytes {}/{}",
                commands_read,
                command_count,
                offset,
                data.len()
            );
        }
    });
}

const ACTIVE_RECHECK_INTERVAL: u64 = 20;
const CRITICAL_RECHECK_INTERVAL: u64 = 5;
const BALLISTIC_MAX_INTERVAL: u64 = 20;
const DORMANT_LINEAR_SPEED_SQ: Real = 0.0025;
const DORMANT_ANGULAR_SPEED_SQ: Real = 0.0025;

fn recentered_bounds(bounds: crate::scene::UniverseAabb, center: crate::scene::DVec3) -> crate::scene::UniverseAabb {
    let half_extents = (bounds.max - bounds.min) * 0.5;
    let half_extents_f = crate::scene::DVec3::new(half_extents.x.max(0.5), half_extents.y.max(0.5), half_extents.z.max(0.5));
    crate::scene::UniverseAabb::around(center, half_extents_f)
}

fn body_has_constraints(
    sim: &SimulationSceneData,
    sable: &SableSceneData,
    id: LevelColliderID,
) -> bool {
    let Some(handle) = sable.rigid_bodies.get(&id).copied() else {
        return sable
            .level_colliders
            .get(&id)
            .and_then(|info| info.static_mount)
            .is_some();
    };
    sim.impulse_joint_set
        .attached_joints(handle)
        .next()
        .is_some()
        || sim.multibody_joint_set.rigid_body_link(handle).is_some()
        || sable
            .level_colliders
            .get(&id)
            .and_then(|info| info.static_mount)
            .is_some()
}

fn terrain_overlaps_bounds(sable: &SableSceneData, bounds: crate::scene::UniverseAabb) -> bool {
    let min = rapier3d::glamx::IVec3::new(bounds.min.x.floor() as i32, bounds.min.y.floor() as i32, bounds.min.z.floor() as i32);
    let max = rapier3d::glamx::IVec3::new((bounds.max.x - std::f64::EPSILON).floor() as i32, (bounds.max.y - std::f64::EPSILON).floor() as i32, (bounds.max.z - std::f64::EPSILON).floor() as i32);
    let block_count = (max.x as i64 - min.x as i64 + 1)
        .max(0)
        .saturating_mul((max.y as i64 - min.y as i64 + 1).max(0))
        .saturating_mul((max.z as i64 - min.z as i64 + 1).max(0));
    if block_count > 4096 {
        return true;
    }
    for x in min.x..=max.x {
        for y in min.y..=max.y {
            for z in min.z..=max.z {
                if sable
                    .get_chunk(x >> CHUNK_SHIFT, y >> CHUNK_SHIFT, z >> CHUNK_SHIFT)
                    .is_some_and(|chunk| chunk.get_block(x & 15, y & 15, z & 15).0 != 0)
                {
                    return true;
                }
            }
        }
    }
    false
}

pub fn sync_active_scene_bodies(
    sim: &mut SimulationSceneData,
    _sable: &mut SableSceneData,
    universe: &mut crate::scene::DimensionUniverse,
    world_origin: crate::scene::DVec3,
) {
    let active_states: Vec<_> = sim
        .island_manager
        .active_bodies()
        .map(|handle| {
            let rb = &sim.rigid_body_set[handle];
            (
                rb.user_data as LevelColliderID,
                rb.translation(),
                *rb.rotation(),
                rb.linvel(),
                rb.angvel(),
            )
        })
        .collect();
    for (id, translation, rotation, linear_velocity, angular_velocity) in active_states {
        let Some(body) = universe.universe_bodies.get_mut(&id) else {
            continue;
        };
        let d_translation = crate::scene::DVec3::new(translation.x as f64, translation.y as f64, translation.z as f64) + world_origin;
        body.translation = d_translation;
        body.rotation = rotation;
        body.linear_velocity = linear_velocity;
        body.angular_velocity = angular_velocity;
        if let Some(geom) = universe.body_geometries.get(&id) {
            body.update_bounds(geom);
        } else {
            body.bounds = recentered_bounds(body.bounds, d_translation);
        }
        let bounds = body.bounds;
        universe.spatial_index.update(id, bounds);
    }
}

pub fn check_scene_evictions(
    sim: &mut SimulationSceneData,
    sable: &mut SableSceneData,
    universe: &mut crate::scene::DimensionUniverse,
    world_origin: crate::scene::DVec3,
    gravity: Vec3,
) {
    let current_tick = universe.current_tick;
    let resident_ids: Vec<LevelColliderID> = sable.rigid_bodies.keys().copied().collect();
    for id in resident_ids {
        let Some(&handle) = sable.rigid_bodies.get(&id) else { continue };
        let rb = &sim.rigid_body_set[handle];
        let velocity = rb.linvel();
        let angular_velocity = rb.angvel();
        let constrained = body_has_constraints(sim, sable, id);
        if constrained {
            continue;
        }

        let ubody = match universe.universe_bodies.get(&id) {
            Some(b) => b,
            None => continue,
        };
        let effective_gravity = gravity * ubody.dynamics.gravity_scale;
        let is_gravity_free = effective_gravity.length_squared() < 1e-4;
        let is_slow = velocity.length_squared() <= DORMANT_LINEAR_SPEED_SQ
            && angular_velocity.length_squared() <= DORMANT_ANGULAR_SPEED_SQ;

        let bounds = Some(ubody.bounds);
        if is_slow && is_gravity_free {
            let has_collision = bounds.is_some_and(|b| !universe.spatial_index.query(b, id).is_empty() || terrain_overlaps_bounds(sable, b));
            if !has_collision {
                if evict_rapier_body(sim, sable, universe, world_origin, id, false, true) {
                    if let Some(body) = universe.universe_bodies.get_mut(&id) {
                        body.last_update_tick = current_tick;
                        body.simulation_tier = crate::scene::SimulationTier::Dormant;
                    }
                }
            }
        } else if let Some(bounds) = bounds {
            let elapsed = 0.05 * BALLISTIC_MAX_INTERVAL as Real;
            let lookahead_disp = velocity * elapsed + effective_gravity * (0.5 * elapsed * elapsed);
            let swept = bounds.swept(crate::scene::DVec3::new(lookahead_disp.x as f64, lookahead_disp.y as f64, lookahead_disp.z as f64));
            let has_collision = !universe.spatial_index.query(swept, id).is_empty()
                || terrain_overlaps_bounds(sable, swept)
                || universe.swept_intersects_terrain(swept);
            if !has_collision {
                if evict_rapier_body(sim, sable, universe, world_origin, id, false, true) {
                    if let Some(body) = universe.universe_bodies.get_mut(&id) {
                        body.last_update_tick = current_tick;
                        body.simulation_tier = crate::scene::SimulationTier::Ballistic;
                    }
                    universe.schedule_body(id, current_tick + BALLISTIC_MAX_INTERVAL);
                }
            }
        }
    }
}

pub fn tick_universe(
    universe: &mut crate::scene::DimensionUniverse,
    gravity: Vec3,
    time_step: Real,
    absolute_tick: u64,
) {
    universe.current_tick = absolute_tick;
    let current_tick = universe.current_tick;

    while let Some(id) = universe.pop_due_body() {
        let Some(body) = universe.universe_bodies.get(&id) else {
            continue;
        };
        let tier = body.simulation_tier;
        let bounds = body.bounds;
        let velocity = body.linear_velocity;

        match tier {
            crate::scene::SimulationTier::Dormant => {}
            crate::scene::SimulationTier::Ballistic => {
                let elapsed_ticks = current_tick.saturating_sub(body.last_update_tick).max(1);
                let elapsed = time_step * elapsed_ticks as Real;
                let effective_gravity = gravity * body.dynamics.gravity_scale;
                let displacement =
                    velocity * elapsed + effective_gravity * (0.5 * elapsed * elapsed);
                let lookahead = time_step * elapsed_ticks.max(BALLISTIC_MAX_INTERVAL) as Real;
                let lookahead_displacement =
                    velocity * lookahead + effective_gravity * (0.5 * lookahead * lookahead);
                let swept = bounds.swept(crate::scene::DVec3::new(lookahead_displacement.x as f64, lookahead_displacement.y as f64, lookahead_displacement.z as f64));
                let neighbors = universe.spatial_index.query(swept, id);
                let terrain_risk = universe.swept_intersects_terrain(swept);
                if !neighbors.is_empty() || terrain_risk {
                    universe.request_materialization(id);
                    if let Some(body) = universe.universe_bodies.get_mut(&id) {
                        body.simulation_tier = crate::scene::SimulationTier::Active;
                    }
                    for neighbor in neighbors {
                        if universe.universe_bodies.contains_key(&neighbor) {
                            universe.request_materialization(neighbor);
                            if let Some(body) = universe.universe_bodies.get_mut(&neighbor) {
                                body.simulation_tier = crate::scene::SimulationTier::Active;
                            }
                            universe.schedule_body(neighbor, current_tick + 1);
                        }
                    }
                    universe.schedule_body(id, current_tick + 1);
                } else {
                    let body = universe.universe_bodies.get_mut(&id).unwrap();
                    body.translation += crate::scene::DVec3::new(displacement.x as f64, displacement.y as f64, displacement.z as f64);
                    body.linear_velocity += effective_gravity * elapsed;
                    let linear_decay = (-body.dynamics.linear_damping * elapsed).exp();
                    let angular_decay = (-body.dynamics.angular_damping * elapsed).exp();
                    body.linear_velocity *= linear_decay;
                    body.angular_velocity *= angular_decay;
                    if body.angular_velocity.length_squared() > Real::EPSILON {
                        body.rotation = (Quat::from_scaled_axis(body.angular_velocity * elapsed)
                            * body.rotation)
                            .normalize();
                    }
                    if let Some(geom) = universe.body_geometries.get(&id) {
                        body.update_bounds(geom);
                    } else {
                        body.bounds = recentered_bounds(body.bounds, body.translation);
                    }
                    body.last_update_tick = current_tick;
                    let new_bounds = body.bounds;
                    universe.pose_dirty_bodies.insert(id);
                    universe.spatial_index.update(id, new_bounds);

                    let is_slow = body.linear_velocity.length_squared() <= DORMANT_LINEAR_SPEED_SQ
                        && body.angular_velocity.length_squared() <= DORMANT_ANGULAR_SPEED_SQ;
                    let is_gravity_free = effective_gravity.length_squared() < 1e-4;
                    if is_slow && is_gravity_free {
                        body.simulation_tier = crate::scene::SimulationTier::Dormant;
                    } else {
                        universe.schedule_body(id, current_tick + BALLISTIC_MAX_INTERVAL);
                    }
                }
            }
            crate::scene::SimulationTier::Active | crate::scene::SimulationTier::Critical => {
                if body.resident.is_none() {
                    universe.request_materialization(id);
                }
                universe.schedule_body(id, current_tick + ACTIVE_RECHECK_INTERVAL);
            }
        }
    }
}

pub fn instantiate_rapier_body(
    sim: &mut SimulationSceneData,
    sable_data: &mut SableSceneData,
    universe: &mut crate::scene::DimensionUniverse,
    world_origin: crate::scene::DVec3,
    id: crate::scene::LevelColliderID,
) {
    let ubody = universe.universe_bodies.get_mut(&id).unwrap();
    if ubody.resident.is_some() {
        return;
    }

    let rigid_body = build_resident_rigid_body(id, ubody, world_origin);

    let handle = sim.rigid_body_set.insert(rigid_body);

    let collider = rapier3d::prelude::ColliderBuilder::new(rapier3d::prelude::SharedShape::new(
        crate::collider::LevelCollider::new(Some(id as crate::scene::LevelColliderID), false),
    ))
    .friction(0.525)
    .active_events(rapier3d::prelude::ActiveEvents::CONTACT_FORCE_EVENTS)
    .active_hooks(rapier3d::prelude::ActiveHooks::MODIFY_SOLVER_CONTACTS)
    .density(0.0)
    .collision_groups(LEVEL_GROUP)
    .build();

    let collider_handle =
        sim.collider_set
            .insert_with_parent(collider, handle, &mut sim.rigid_body_set);

    let geom = universe.body_geometries.get_mut(&id);
    let mut info = ActiveLevelColliderInfo::new(Some(collider_handle));
    if let Some(g) = geom {
        info.local_bounds_min = g.local_bounds_min.or(Some(IVec3::splat(-1)));
        info.local_bounds_max = g.local_bounds_max.or(Some(IVec3::splat(1)));
        info.center_of_mass = g.center_of_mass.or(Some(DVec3::ZERO));
        info.octree = g.octree.clone();
        info.section_octrees = g.section_octrees.clone();
        info.octree_origin = g.octree_origin;
        info.geometry_version = g.geometry_version;
        info.dirty_sections = g.dirty_sections.clone();
        info.chunk_map = g.chunk_map.clone().or_else(|| Some(HashMap::new()));

        if info.octree.is_none() || !info.dirty_sections.is_empty() {
            let physics_state = get_physics_state();
            let collider_map = &physics_state.voxel_collider_map;
            let min = info.local_bounds_min.unwrap();
            let max = info.local_bounds_max.unwrap();
            info.octree = None;
            info.section_octrees.clear();
            info.dirty_sections.clear();
            info.set_local_bounds(min, max, &sable_data.main_level_chunks, collider_map);

            g.octree = info.octree.clone();
            g.section_octrees = info.section_octrees.clone();
            g.octree_origin = info.octree_origin;
            g.dirty_sections.clear();
        }
    } else {
        info.local_bounds_min = Some(IVec3::splat(-1));
        info.local_bounds_max = Some(IVec3::splat(1));
        info.center_of_mass = Some(DVec3::ZERO);
        info.chunk_map = Some(HashMap::new());
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        info.set_local_bounds(IVec3::splat(-1), IVec3::splat(1), &sable_data.main_level_chunks, collider_map);
    }
    update_collider_aabb(sim, &mut info);
    let callback_sections = info
        .chunk_map
        .as_ref()
        .map(|cm| cm.values().filter(|chunk| chunk.serial_callback_blocks > 0).count())
        .unwrap_or(0) as u32;
    sable_data.sublevel_serial_callback_sections += callback_sections;

    sable_data.level_colliders.insert(id, info);

    sable_data.rigid_bodies.insert(id, handle);

    ubody.resident = Some(crate::scene::ResidentPhysicsBody { rigid_body: handle, scene_handle: sable_data.scene_handle });
    universe.eviction_events.retain(|&e_id| e_id != id);
    universe.pending_evictions.remove(&id);
    sim.island_manager
        .wake_up(&mut sim.rigid_body_set, handle, true);
}

fn build_resident_rigid_body(
    id: crate::scene::LevelColliderID,
    ubody: &crate::scene::UniverseBody,
    world_origin: crate::scene::DVec3,
) -> RigidBody {
    let local = ubody.translation - world_origin;
    let local_translation = Vec3::new(local.x as f32, local.y as f32, local.z as f32);
    let mut rigid_body = rapier3d::prelude::RigidBodyBuilder::dynamic()
        .ccd_enabled(ubody.dynamics.ccd_enabled)
        .user_data(id as u128)
        .translation(local_translation)
        .build();
    rigid_body.set_rotation(ubody.rotation, false);
    rigid_body.set_linvel(ubody.linear_velocity, false);
    rigid_body.set_angvel(ubody.angular_velocity, false);
    if let Some(properties) = &ubody.dynamics.additional_mass_properties {
        rigid_body.set_additional_mass_properties(properties.clone(), false);
    }
    rigid_body.set_gravity_scale(ubody.dynamics.gravity_scale, false);
    rigid_body.set_locked_axes(ubody.dynamics.locked_axes, false);

    let activation_params = rigid_body.activation_mut();
    activation_params.angular_threshold = 0.15;
    activation_params.normalized_linear_threshold = 0.15;

    rigid_body.set_linear_damping(ubody.dynamics.linear_damping);
    rigid_body.set_angular_damping(ubody.dynamics.angular_damping);
    rigid_body.enable_gyroscopic_forces(true);
    rigid_body
}

fn snapshot_resident_state(ubody: &mut crate::scene::UniverseBody, rigid_body: &RigidBody, world_origin: crate::scene::DVec3) {
    let local = rigid_body.translation();
    ubody.translation = crate::scene::DVec3::new(local.x as f64, local.y as f64, local.z as f64) + world_origin;
    ubody.rotation = *rigid_body.rotation();
    ubody.linear_velocity = rigid_body.linvel();
    ubody.angular_velocity = rigid_body.angvel();
    ubody.dynamics.linear_damping = rigid_body.linear_damping();
    ubody.dynamics.angular_damping = rigid_body.angular_damping();
    ubody.dynamics.gravity_scale = rigid_body.gravity_scale();
    ubody.dynamics.locked_axes = rigid_body.locked_axes();
    ubody.dynamics.ccd_enabled = rigid_body.is_ccd_enabled();
}

pub fn evict_rapier_body(
    sim: &mut SimulationSceneData,
    sable_data: &mut SableSceneData,
    universe: &mut crate::scene::DimensionUniverse,
    world_origin: crate::scene::DVec3,
    id: crate::scene::LevelColliderID,
    force: bool,
    emit_event: bool,
) -> bool {
    let ubody = universe.universe_bodies.get_mut(&id).unwrap();
    if let Some(resident_handle) = ubody.resident.as_ref().map(|resident| resident.rigid_body) {
        if !force {
            let handle = resident_handle;
            let has_impulse_joint = sim
                .impulse_joint_set
                .attached_joints(handle)
                .next()
                .is_some();
            let has_multibody_joint = sim.multibody_joint_set.rigid_body_link(handle).is_some();
            let has_static_mount = sable_data
                .level_colliders
                .get(&id)
                .and_then(|info| info.static_mount)
                .is_some();
            if has_impulse_joint || has_multibody_joint || has_static_mount {
                return false;
            }
        }

        let rb = &sim.rigid_body_set[resident_handle];
        snapshot_resident_state(ubody, rb, world_origin);

        if let Some(mut info) = sable_data.level_colliders.remove(&id) {
            if let Some(collider_handle) = info.collider {
                sim.collider_set.remove(
                    collider_handle,
                    &mut sim.island_manager,
                    &mut sim.rigid_body_set,
                    true,
                );
            }
            let callback_sections = info
                .chunk_map
                .as_ref()
                .map(|cm| cm.values().filter(|chunk| chunk.serial_callback_blocks > 0).count())
                .unwrap_or(0) as u32;
            sable_data.sublevel_serial_callback_sections =
                sable_data.sublevel_serial_callback_sections.saturating_sub(callback_sections);

            info.collider = None;
            let geom = universe.body_geometries.entry(id).or_default();
            geom.local_bounds_min = info.local_bounds_min;
            geom.local_bounds_max = info.local_bounds_max;
            geom.center_of_mass = info.center_of_mass;
            geom.octree = info.octree.clone();
            geom.section_octrees = info.section_octrees.clone();
            geom.octree_origin = info.octree_origin;
            geom.geometry_version = info.geometry_version;
            geom.dirty_sections = info.dirty_sections.clone();
            if let Some(cm) = info.chunk_map {
                geom.chunk_map = Some(cm);
            }
        }

        sim.rigid_body_set.remove(
            resident_handle,
            &mut sim.island_manager,
            &mut sim.collider_set,
            &mut sim.impulse_joint_set,
            &mut sim.multibody_joint_set,
            true,
        );

        sable_data.rigid_bodies.remove(&id);
        ubody.resident = None;
        universe.pose_dirty_bodies.insert(id);
        if emit_event {
            universe.record_eviction(id);
        }
    }
    true
}

/// Benchmark-only facade over the same registry, scheduler, spatial index,
/// Rapier scene, and pose-export data used by production JNI calls.
#[doc(hidden)]
pub struct RegistryScalingHarness {
    scene: PhysicsScene,
    pose_buffer: Vec<u8>,
}

#[doc(hidden)]
impl RegistryScalingHarness {
    pub fn new(persistent_count: usize, resident_count: usize, awake_count: usize) -> Self {
        assert!(awake_count <= resident_count && resident_count <= persistent_count);
        let sable_data = Arc::new(RwLock::new(SableSceneData {
        scene_handle: 0,
        main_level_chunks: HashMap::new(),
            octree_chunks: HashMap::new(),
            joint_set: SableJointSet::new(),
            rope_map: RopeMap::default(),
            level_colliders: HashMap::new(),
            rigid_bodies: HashMap::new(),
                        terrain_serial_callback_sections: 0,
            sublevel_serial_callback_sections: 0,
        }));
        let manifold_info_map = Arc::new(SableManifoldInfoMap::default());
        let reported_collisions = Arc::new(ReportedCollisionBuffer::new());
        let dispatcher = SableDispatcher {
            sable_data: Arc::clone(&sable_data),
            manifold_info_map: Arc::clone(&manifold_info_map),
        };
        let sim_data = SimulationSceneData {
            integration_parameters: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::with_query_dispatcher(
                dispatcher.chain(DefaultQueryDispatcher),
            ),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: SablePhysicsHooks {
                sable_data: Arc::clone(&sable_data),
                manifold_info_map: Arc::clone(&manifold_info_map),
                current_step_vm: None,
            },
            event_handler: SableEventHandler {
                reported_collisions: Arc::clone(&reported_collisions),
            },
        };
        let scene = PhysicsScene {
            universe: std::sync::Arc::new(std::sync::RwLock::new(crate::scene::DimensionUniverse::default())),
            sim_data: RwLock::new(sim_data),
            sable_data,
            reported_collisions,
            manifold_info_map,
            current_step_vm: None,
            ground_handle: None,
            gravity: Vec3::ZERO,
            world_origin: RwLock::new(crate::scene::DVec3::zeros()),
            universal_drag: 0.0,
        };

        {
            let mut sable = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
            for id in 0..persistent_count {
                let translation = crate::scene::DVec3::new(
                    (id % 10_000) as f64 * (crate::scene::MACRO_CELL_SIZE as f64 * 2.0),
                    0.0,
                    (id / 10_000) as f64 * (crate::scene::MACRO_CELL_SIZE as f64 * 2.0),
                );
                let bounds = crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
                universe.universe_bodies.insert(
                    id,
                    crate::scene::UniverseBody {
                        id,
                        translation,
                        rotation: Quat::IDENTITY,
                        linear_velocity: if id < awake_count {
                            Vec3::X
                        } else {
                            Vec3::ZERO
                        },
                        angular_velocity: Vec3::ZERO,
                        dynamics: crate::scene::BodyDynamics {
                            additional_mass_properties: None,
                            linear_damping: 0.0,
                            angular_damping: 0.0,
                            gravity_scale: 0.0,
                            locked_axes: LockedAxes::empty(),
                            ccd_enabled: false,
                        },
                        simulation_tier: if id < resident_count {
                            crate::scene::SimulationTier::Active
                        } else {
                            crate::scene::SimulationTier::Dormant
                        },
                        bounds,
                        last_update_tick: 0,
                        next_update_tick: 0,
                        schedule_generation: 0,
                        resident: None,
                        assembly_root: id,
                        assembly_size: 1,
            command_queue: Vec::new(),
                    },
                );
                universe.spatial_index.update(id, bounds);
            }
            let mut sim = scene.sim_data.write().unwrap();
            for id in 0..resident_count {
                let mut info = ActiveLevelColliderInfo::new(None);
                info.local_bounds_min = Some(IVec3::splat(-1));
                info.local_bounds_max = Some(IVec3::splat(1));
                info.center_of_mass = Some(DVec3::ZERO);
                sable.level_colliders.insert(id, info);
                instantiate_rapier_body(&mut sim, &mut sable, &mut *universe, crate::scene::DVec3::zeros(), id);
                if id >= awake_count {
                    let handle = sable.rigid_bodies[&id];
                    sim.rigid_body_set[handle].sleep();
                }
            }
        }
        Self {
            scene,
            pose_buffer: vec![0; (resident_count + 1) * 60],
        }
    }

    pub fn tick_and_export_poses(&mut self) -> usize {
        {
            let mut sim = self.scene.sim_data.write().unwrap();
            let sim: &mut SimulationSceneData = &mut sim;
            let params = sim.integration_parameters.clone();
            sim.pipeline.step(
                Vec3::ZERO,
                &params,
                &mut sim.island_manager,
                &mut sim.broad_phase,
                &mut sim.narrow_phase,
                &mut sim.rigid_body_set,
                &mut sim.collider_set,
                &mut sim.impulse_joint_set,
                &mut sim.multibody_joint_set,
                &mut sim.ccd_solver,
                &sim.physics_hooks,
                &sim.event_handler,
            );
        }
        {
            let mut sable = self.scene.sable_data.write().unwrap();
            let mut universe = self.scene.universe.write().unwrap();
            let mut sim = self.scene.sim_data.write().unwrap();
            let next_tick = universe.current_tick + 1;
            tick_universe(&mut universe, Vec3::ZERO, 0.05, next_tick);
            for req in universe.materialization_requests.clone() {
                instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), req.id);
            }
            universe.materialization_requests.clear();
            sync_active_scene_bodies(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros());
            check_scene_evictions(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), Vec3::ZERO);
        }
        let poses = match collect_active_poses(&self.scene, self.pose_buffer.len() / 60) {
            Ok(poses) => poses,
            Err(required) => {
                self.pose_buffer.resize((required + 1) * 60, 0);
                collect_active_poses(&self.scene, required)
                    .expect("resized benchmark pose buffer must fit the active export set")
            }
        };
        encode_active_poses(&poses, &mut self.pose_buffer[..poses.len() * 60]);
        poses.len()
    }

    pub fn persistent_count(&self) -> usize {
        self.scene.universe.read().unwrap().universe_bodies.len()
    }

    pub fn schedule_bodies(&mut self, count: usize, due_tick: u64) {
        self.schedule_bodies_range(0, count, due_tick);
    }

    pub fn schedule_bodies_range(&mut self, start_id: usize, count: usize, due_tick: u64) {
        let mut sable = self.scene.sable_data.write().unwrap();
        let mut universe = self.scene.universe.write().unwrap();
        for id in start_id..start_id.saturating_add(count) {
            if universe.universe_bodies.contains_key(&id) {
                universe.schedule_body(id, due_tick);
            }
        }
    }

    pub fn set_ballistic(&mut self, start_id: usize, count: usize) {
        let mut sable = self.scene.sable_data.write().unwrap();
        let mut universe = self.scene.universe.write().unwrap();
        for id in start_id..start_id + count {
            if let Some(body) = universe.universe_bodies.get_mut(&id) {
                body.simulation_tier = crate::scene::SimulationTier::Ballistic;
            }
        }
    }

    pub fn reset_scheduler_state(&mut self, resident_count: usize) {
        let mut sable = self.scene.sable_data.write().unwrap();
        let mut universe = self.scene.universe.write().unwrap();
        universe.current_tick = 0;
        universe.scheduled_bodies.clear();
        universe.pose_dirty_bodies.clear();
        for (id, body) in universe.universe_bodies.iter_mut() {
            body.simulation_tier = if *id < resident_count {
                crate::scene::SimulationTier::Active
            } else {
                crate::scene::SimulationTier::Dormant
            };
            body.next_update_tick = 0;
            body.last_update_tick = 0;
            body.schedule_generation += 1;
        }
    }
}

#[cfg(test)]
mod residency_tests {
    use super::*;

    fn body(id: LevelColliderID) -> crate::scene::UniverseBody {
        let translation = crate::scene::DVec3::new(12.5, -4.0, 88.25);
        crate::scene::UniverseBody {
            id,
            translation,
            rotation: Quat::from_rotation_y(0.7),
            linear_velocity: Vec3::new(3.0, 4.0, 5.0),
            angular_velocity: Vec3::new(0.2, 0.3, 0.4),
            dynamics: crate::scene::BodyDynamics {
                additional_mass_properties: Some(MassProperties::with_inertia_matrix(
                    Vec3::ZERO,
                    50_000.0,
                    Mat3::from_diagonal(Vec3::new(100.0, 200.0, 300.0)).into(),
                )),
                linear_damping: 0.12,
                angular_damping: 0.34,
                gravity_scale: 0.75,
                locked_axes: LockedAxes::TRANSLATION_LOCKED_Y,
                ccd_enabled: true,
            },
            simulation_tier: crate::scene::SimulationTier::Active,
            bounds: crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(1.0, 1.0, 1.0)),
            last_update_tick: 0,
            next_update_tick: 0,
            schedule_generation: 0,
            resident: None,
            assembly_root: id,
            assembly_size: 1,
            command_queue: Vec::new(),
        }
    }

    fn scene_data() -> (SimulationSceneData, Arc<RwLock<SableSceneData>>, crate::scene::DimensionUniverse) {
        let sable_data = Arc::new(RwLock::new(SableSceneData {
            scene_handle: 0,
            main_level_chunks: HashMap::new(),
            octree_chunks: HashMap::new(),
            joint_set: SableJointSet::new(),
            rope_map: RopeMap::default(),
            level_colliders: HashMap::new(),
            rigid_bodies: HashMap::new(),
            terrain_serial_callback_sections: 0,
            sublevel_serial_callback_sections: 0,
        }));
        let manifold_info_map = Arc::new(SableManifoldInfoMap::default());
        let reported_collisions = Arc::new(ReportedCollisionBuffer::new());
        let dispatcher = SableDispatcher {
            sable_data: Arc::clone(&sable_data),
            manifold_info_map: Arc::clone(&manifold_info_map),
        };
        let sim = SimulationSceneData {
            integration_parameters: IntegrationParameters::default(),
            pipeline: PhysicsPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::with_query_dispatcher(
                dispatcher.chain(DefaultQueryDispatcher),
            ),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: SablePhysicsHooks {
                sable_data: Arc::clone(&sable_data),
                manifold_info_map,
                current_step_vm: None,
            },
            event_handler: SableEventHandler {
                reported_collisions,
            },
        };
        (sim, sable_data, crate::scene::DimensionUniverse::default())
    }

    #[test]
    fn eviction_reinstantiation_preserves_dynamics_and_geometry() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 42;
        universe.universe_bodies.insert(id, body(id));
        let mut info = ActiveLevelColliderInfo::new(None);
        info.local_bounds_min = Some(IVec3::new(-8, -4, -2));
        info.local_bounds_max = Some(IVec3::new(9, 6, 3));
        info.center_of_mass = Some(DVec3::new(1.5, 0.5, -0.5));
        sable.level_colliders.insert(id, info);

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        let first_handle = universe.universe_bodies[&id]
            .resident
            .as_ref()
            .unwrap()
            .rigid_body;
        let first = &sim.rigid_body_set[first_handle];
        let first_mass_properties = first.mass_properties().additional_local_mprops.clone();
        assert!(first_mass_properties.is_some());
        assert_eq!(first.translation(), Vec3::new(12.5, -4.0, 88.25));
        assert_eq!(first.linvel(), Vec3::new(3.0, 4.0, 5.0));
        assert_eq!(first.locked_axes(), LockedAxes::TRANSLATION_LOCKED_Y);
        assert!(first.is_ccd_enabled());
        let first_collider = sable.level_colliders[&id].collider.unwrap();
        assert!(
            sim.collider_set[first_collider]
                .shape()
                .as_shape::<LevelCollider>()
                .unwrap()
                .cached_aabb
                .is_some()
        );

        assert!(evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, false, false));
        assert!(universe.universe_bodies[&id].resident.is_none());
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        let second_handle = universe.universe_bodies[&id]
            .resident
            .as_ref()
            .unwrap()
            .rigid_body;
        let second = &sim.rigid_body_set[second_handle];
        assert_eq!(
            second.mass_properties().additional_local_mprops,
            first_mass_properties
        );
        assert_eq!(second.translation(), Vec3::new(12.5, -4.0, 88.25));
        assert_eq!(second.linvel(), Vec3::new(3.0, 4.0, 5.0));
        assert_eq!(second.locked_axes(), LockedAxes::TRANSLATION_LOCKED_Y);
        assert!(second.is_ccd_enabled());
        let second_collider = sable.level_colliders[&id].collider.unwrap();
        assert!(
            sim.collider_set[second_collider]
                .shape()
                .as_shape::<LevelCollider>()
                .unwrap()
                .cached_aabb
                .is_some()
        );
    }

    #[test]
    fn constrained_body_is_not_evictable_without_force() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 7;
        universe.universe_bodies.insert(id, body(id));
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        let body_handle = universe.universe_bodies[&id]
            .resident
            .as_ref()
            .unwrap()
            .rigid_body;
        let other = sim.rigid_body_set.insert(RigidBodyBuilder::fixed().build());
        let joint = sim.impulse_joint_set.insert(
            body_handle,
            other,
            FixedJointBuilder::new().build(),
            true,
        );

        assert!(!evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, false, false));
        assert!(universe.universe_bodies[&id].resident.is_some());
        sim.impulse_joint_set.remove(joint, true);
        assert!(evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, false, false));
    }

    #[test]
    fn nonresident_registry_size_does_not_change_rapier_body_count() {
        let (sim, _sable_data, mut universe) = scene_data();
        for id in 0..100_000 {
            universe.universe_bodies.insert(id, body(id));
        }
        assert_eq!(universe.universe_bodies.len(), 100_000);
        assert_eq!(sim.rigid_body_set.len(), 0);
    }

    #[test]
    fn scheduler_discards_stale_entries() {
        let (_sim, _sable_data, mut universe) = scene_data();
        universe.universe_bodies.insert(1, body(1));
        universe.schedule_body(1, 10);
        universe.schedule_body(1, 5);
        assert_eq!(universe.ticks_until_next_scheduled_body(), Some(5));
        universe.current_tick = 5;
        assert_eq!(universe.pop_due_body(), Some(1));
        universe.current_tick = 10;
        assert_eq!(universe.pop_due_body(), None);
    }

    #[test]
    fn ballistic_body_advances_without_becoming_resident() {
        let (sim, _sable_data, mut universe) = scene_data();
        let mut isolated = body(1);
        isolated.translation = crate::scene::DVec3::zeros();
        isolated.linear_velocity = Vec3::new(10.0, 0.0, 0.0);
        isolated.angular_velocity = Vec3::ZERO;
        isolated.dynamics.gravity_scale = 0.0;
        isolated.dynamics.linear_damping = 0.0;
        isolated.dynamics.angular_damping = 0.0;
        isolated.bounds = crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0));
        isolated.simulation_tier = crate::scene::SimulationTier::Ballistic;
        universe.universe_bodies.insert(1, isolated);
        universe.spatial_index
            .update(1, crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.schedule_body(1, 1);

        tick_universe(&mut universe, Vec3::new(0.0, -10.0, 0.0), 0.05, 1);

        let body = &universe.universe_bodies[&1];
        assert_eq!(
            body.simulation_tier,
            crate::scene::SimulationTier::Ballistic
        );
        assert_eq!(body.translation, crate::scene::DVec3::new(0.5, 0.0, 0.0));
        assert!(body.resident.is_none());
        assert_eq!(sim.rigid_body_set.len(), 0);
        assert!(universe.pose_dirty_bodies.contains(&1));
    }

    #[test]
    fn elapsed_scene_ticks_preserve_ballistic_time() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let mut isolated = body(1);
        isolated.translation = crate::scene::DVec3::zeros();
        isolated.linear_velocity = Vec3::new(10.0, 0.0, 0.0);
        isolated.angular_velocity = Vec3::ZERO;
        isolated.dynamics.gravity_scale = 0.0;
        isolated.dynamics.linear_damping = 0.0;
        isolated.dynamics.angular_damping = 0.0;
        isolated.bounds = crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0));
        isolated.simulation_tier = crate::scene::SimulationTier::Ballistic;
        universe.universe_bodies.insert(1, isolated);
        universe.spatial_index
            .update(1, crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.schedule_body(1, 20);

        tick_universe(&mut universe, Vec3::ZERO, 0.05, 20);

        assert_eq!(universe.current_tick, 20);
        assert_eq!(
            universe.universe_bodies[&1].translation,
            crate::scene::DVec3::new(10.0, 0.0, 0.0)
        );
    }

    #[test]
    fn ballistic_sweep_wakes_nearby_dormant_body() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let mut moving = body(1);
        moving.translation = crate::scene::DVec3::zeros();
        moving.linear_velocity = Vec3::new(10.0, 0.0, 0.0);
        moving.angular_velocity = Vec3::ZERO;
        moving.bounds = crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0));
        moving.simulation_tier = crate::scene::SimulationTier::Ballistic;
        let mut dormant = body(2);
        dormant.translation = crate::scene::DVec3::new(8.0, 0.0, 0.0);
        dormant.linear_velocity = Vec3::ZERO;
        dormant.angular_velocity = Vec3::ZERO;
        dormant.bounds = crate::scene::UniverseAabb::around(dormant.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        dormant.simulation_tier = crate::scene::SimulationTier::Dormant;
        universe.universe_bodies.insert(1, moving);
        universe.universe_bodies.insert(2, dormant);
        universe.spatial_index
            .update(1, crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.spatial_index.update(
            2,
            crate::scene::UniverseAabb::around(crate::scene::DVec3::new(8.0, 0.0, 0.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)),
        );
        universe.schedule_body(1, 1);

        tick_universe(&mut universe, Vec3::ZERO, 0.05, 1);

        assert_eq!(
            universe.universe_bodies[&1].simulation_tier,
            crate::scene::SimulationTier::Active
        );
        assert_eq!(
            universe.universe_bodies[&2].simulation_tier,
            crate::scene::SimulationTier::Active
        );
        assert!(universe.materialization_requests.iter().any(|r| r.id == 1));
        assert!(universe.materialization_requests.iter().any(|r| r.id == 2));
        for req in universe.materialization_requests.clone() {
            instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), req.id);
        }
        let handle1 = universe.universe_bodies[&1]
            .resident
            .as_ref()
            .unwrap()
            .rigid_body;
        let handle2 = universe.universe_bodies[&2]
            .resident
            .as_ref()
            .unwrap()
            .rigid_body;
        assert!(!sim.rigid_body_set[handle1].is_sleeping());
        assert!(!sim.rigid_body_set[handle2].is_sleeping());
        assert_eq!(universe.next_due_tick(), Some(2));

        let params = sim.integration_parameters.clone();
        sim.pipeline.step(
            Vec3::ZERO,
            &params,
            &mut sim.island_manager,
            &mut sim.broad_phase,
            &mut sim.narrow_phase,
            &mut sim.rigid_body_set,
            &mut sim.collider_set,
            &mut sim.impulse_joint_set,
            &mut sim.multibody_joint_set,
            &mut sim.ccd_solver,
            &sim.physics_hooks,
            &sim.event_handler,
        );
        assert_eq!(sim.island_manager.active_bodies().count(), 2);
    }

    #[test]
    fn serial_step_refcounting_tracks_section_callbacks() {
        let (_, sable_data, _) = scene_data();
        let mut sable = sable_data.write().unwrap();
        assert!(sable.can_parallel_step());

        sable.terrain_serial_callback_sections += 1;
        assert!(!sable.can_parallel_step());

        sable.terrain_serial_callback_sections =
            sable.terrain_serial_callback_sections.saturating_sub(1);
        assert!(sable.can_parallel_step());
    }

    #[test]
    fn scale_dormant_bodies_zero_rapier_cost() {
        let (_sim, _sable_data, mut universe) = scene_data();
        for id in 0..10_000 {
            let mut b = body(id);
            b.translation = crate::scene::DVec3::new(id as f64 * 5.0, 0.0, 0.0);
            b.bounds = crate::scene::UniverseAabb::around(b.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
            universe.spatial_index.update(id, b.bounds);
            universe.universe_bodies.insert(id, b);
        }
        assert_eq!(universe.universe_bodies.len(), 10_000);
        tick_universe(&mut universe, Vec3::new(0.0, -9.81, 0.0), 0.05, 1);
        assert_eq!(universe.materialization_requests.len(), 0);
    }

    #[test]
    fn rebase_isolation_never_mutates_universe_body_or_spatial_index() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 42;
        let global_pos = crate::scene::DVec3::new(100.0, 50.0, 200.0);
        let mut b = body(id);
        b.translation = global_pos;
        b.bounds = crate::scene::UniverseAabb::around(global_pos, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        universe.universe_bodies.insert(id, b);
        universe.spatial_index.update(id, crate::scene::UniverseAabb::around(global_pos, crate::scene::DVec3::new(1.0, 1.0, 1.0)));

        let old_origin = crate::scene::DVec3::new(0.0, 0.0, 0.0);
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, old_origin, id);

        let handle = sable.rigid_bodies[&id];
        assert_eq!(sim.rigid_body_set[handle].translation(), Vec3::new(100.0, 50.0, 200.0));

        let new_origin = crate::scene::DVec3::new(50.0, 0.0, 100.0);
        let delta = new_origin - old_origin;
        let delta_f32 = Vec3::new(delta.x as f32, delta.y as f32, delta.z as f32);
        for (_id, handle) in &sable.rigid_bodies {
            if let Some(rb) = sim.rigid_body_set.get_mut(*handle) {
                let mut pos = rb.translation().clone();
                pos -= delta_f32;
                rb.set_translation(pos, true);
            }
        }

        assert_eq!(sim.rigid_body_set[handle].translation(), Vec3::new(50.0, 50.0, 100.0));
        assert_eq!(universe.universe_bodies[&id].translation, global_pos);
        let queried = universe.spatial_index.query(crate::scene::UniverseAabb::around(global_pos, crate::scene::DVec3::new(1.0, 1.0, 1.0)), 999);
        assert!(queried.contains(&id));
    }

    #[test]
    fn active_to_ballistic_transition_when_moving_safely() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 10;
        let mut moving = body(id);
        moving.translation = crate::scene::DVec3::new(500.0, 100.0, 500.0);
        moving.linear_velocity = Vec3::new(5.0, 0.0, 0.0);
        moving.bounds = crate::scene::UniverseAabb::around(moving.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        moving.simulation_tier = crate::scene::SimulationTier::Active;
        universe.universe_bodies.insert(id, moving);
        universe.spatial_index.update(id, crate::scene::UniverseAabb::around(crate::scene::DVec3::new(500.0, 100.0, 500.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)));

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        assert!(universe.universe_bodies[&id].resident.is_some());

        check_scene_evictions(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), Vec3::ZERO);

        assert!(universe.universe_bodies[&id].resident.is_none());
        assert_eq!(universe.universe_bodies[&id].simulation_tier, crate::scene::SimulationTier::Ballistic);
        assert!(universe.eviction_events.contains(&id));
    }

    #[test]
    fn terrain_risk_wakes_ballistic_body() {
        let (_sim, _sable_data, mut universe) = scene_data();
        universe.terrain_sections.insert(pack_section_pos(5, 0, 0)); // terrain at x in [80..96]

        let mut ballistic = body(100);
        ballistic.translation = crate::scene::DVec3::new(70.0, 8.0, 8.0);
        ballistic.linear_velocity = Vec3::new(10.0, 0.0, 0.0); // will sweep into x: 70..80+
        ballistic.bounds = crate::scene::UniverseAabb::around(ballistic.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        ballistic.simulation_tier = crate::scene::SimulationTier::Ballistic;
        universe.universe_bodies.insert(100, ballistic);
        universe.spatial_index.update(100, crate::scene::UniverseAabb::around(crate::scene::DVec3::new(70.0, 8.0, 8.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.schedule_body(100, 1);

        tick_universe(&mut universe, Vec3::ZERO, 0.05, 1);

        assert_eq!(universe.universe_bodies[&100].simulation_tier, crate::scene::SimulationTier::Active);
        assert!(universe.materialization_requests.iter().any(|r| r.id == 100));
    }

    #[test]
    fn command_header_rejects_version_and_length_mismatches() {
        let mut header = Vec::new();
        header.extend_from_slice(&COMMAND_MAGIC.to_ne_bytes());
        header.extend_from_slice(&COMMAND_PROTOCOL_VERSION.to_ne_bytes());
        header.extend_from_slice(&0_i32.to_ne_bytes());
        header.extend_from_slice(&(COMMAND_HEADER_SIZE as i32).to_ne_bytes());
        assert_eq!(validate_command_header(&header), Ok(0));

        let mut wrong_version = header.clone();
        wrong_version[4..6].copy_from_slice(&2_i16.to_ne_bytes());
        assert!(validate_command_header(&wrong_version).is_err());

        let mut wrong_length = header;
        wrong_length[10..14].copy_from_slice(&999_i32.to_ne_bytes());
        assert!(validate_command_header(&wrong_length).is_err());
        assert!(validate_command_header(&wrong_length[..8]).is_err());
    }

    #[test]
    fn dirty_section_tracking_is_incremental_and_persistent() {
        let mut info = ActiveLevelColliderInfo::new(None);
        info.chunk_map = Some(HashMap::new());
        info.mark_section_dirty(4, -2, 9);
        info.mark_section_dirty(4, -2, 9);
        info.mark_section_dirty(5, -2, 9);

        assert_eq!(info.geometry_version, 3);
        assert_eq!(info.dirty_sections.len(), 2);
        assert!(info.dirty_sections.contains(&pack_section_pos(4, -2, 9)));
        assert!(info.has_own_chunks());
    }

    #[test]
    fn unresident_deletion_cleans_all_universe_records() {
        let (_sim, _sable, mut universe) = scene_data();
        let id = 42;
        let mut b = body(id);
        b.translation = crate::scene::DVec3::new(100.0, 50.0, 100.0);
        universe.universe_bodies.insert(id, b);
        universe.spatial_index.update(id, crate::scene::UniverseAabb::around(crate::scene::DVec3::new(100.0, 50.0, 100.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.body_geometries.insert(id, crate::scene::PersistentBodyGeometry::default());
        universe.pose_dirty_bodies.insert(id);
        universe.pending_command_bodies.insert(id);
        universe.pending_materializations.insert(id);
        universe.pending_evictions.insert(id);

        universe.spatial_index.remove(id);
        universe.universe_bodies.remove(&id);
        universe.body_geometries.remove(&id);
        universe.pose_dirty_bodies.remove(&id);
        universe.pending_command_bodies.remove(&id);
        universe.pending_materializations.remove(&id);
        universe.pending_evictions.remove(&id);

        assert!(!universe.universe_bodies.contains_key(&id));
        assert!(!universe.body_geometries.contains_key(&id));
        assert!(!universe.pose_dirty_bodies.contains(&id));
        assert!(!universe.pending_command_bodies.contains(&id));
        assert!(!universe.pending_materializations.contains(&id));
        assert!(!universe.pending_evictions.contains(&id));
        assert!(universe.spatial_index.query(crate::scene::UniverseAabb::around(crate::scene::DVec3::new(100.0, 50.0, 100.0), crate::scene::DVec3::new(2.0, 2.0, 2.0)), 999).is_empty());
    }

    #[test]
    fn dormant_body_persistent_geometry_retained_across_materialization() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 55;
        let mut b = body(id);
        b.translation = crate::scene::DVec3::new(200.0, 64.0, 200.0);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(-2, 0, -2));
        geom.local_bounds_max = Some(IVec3::new(3, 5, 3));
        geom.center_of_mass = Some(rapier3d::glamx::DVec3::new(0.5, 2.5, 0.5));
        geom.geometry_version = 7;
        universe.body_geometries.insert(id, geom);

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        let info = &sable.level_colliders[&id];
        assert_eq!(info.local_bounds_min, Some(IVec3::new(-2, 0, -2)));
        assert_eq!(info.local_bounds_max, Some(IVec3::new(3, 5, 3)));
        assert_eq!(info.center_of_mass, Some(rapier3d::glamx::DVec3::new(0.5, 2.5, 0.5)));
        assert_eq!(info.geometry_version, 7);

        evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);
        let retained = &universe.body_geometries[&id];
        assert_eq!(retained.local_bounds_min, Some(IVec3::new(-2, 0, -2)));
        assert_eq!(retained.local_bounds_max, Some(IVec3::new(3, 5, 3)));
        assert_eq!(retained.center_of_mass, Some(rapier3d::glamx::DVec3::new(0.5, 2.5, 0.5)));
    }

    #[test]
    fn gravity_prevents_unsupported_dormancy() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 77;
        let mut floating = body(id);
        floating.translation = crate::scene::DVec3::new(100.0, 200.0, 100.0);
        floating.linear_velocity = Vec3::ZERO;
        floating.angular_velocity = Vec3::ZERO;
        floating.dynamics.gravity_scale = 1.0;
        floating.bounds = crate::scene::UniverseAabb::around(floating.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        floating.simulation_tier = crate::scene::SimulationTier::Active;
        universe.universe_bodies.insert(id, floating);
        universe.spatial_index.update(id, crate::scene::UniverseAabb::around(crate::scene::DVec3::new(100.0, 200.0, 100.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)));

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        check_scene_evictions(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), Vec3::new(0.0, -9.81, 0.0));

        assert!(universe.universe_bodies[&id].resident.is_none());
        assert_eq!(universe.universe_bodies[&id].simulation_tier, crate::scene::SimulationTier::Ballistic);
    }

    #[test]
    fn terrain_domain_guard_at_huge_coordinates() {
        let mut universe = crate::scene::DimensionUniverse::default();
        universe.terrain_sections.insert(pack_section_pos(0, 0, 0));

        let bounds_huge = crate::scene::UniverseAabb::around(
            crate::scene::DVec3::new(10_000_000_000.0, 100.0, 10_000_000_000.0),
            crate::scene::DVec3::new(10.0, 10.0, 10.0),
        );
        assert!(!universe.swept_intersects_terrain(bounds_huge));
    }

    #[test]
    fn eviction_dirties_pose_for_universe_channel() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 88;
        let mut b = body(id);
        b.translation = crate::scene::DVec3::new(10.0, 0.0, 0.0);
        universe.universe_bodies.insert(id, b);

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        // Move Rapier body to X=11.0
        let resident_handle = sable.rigid_bodies[&id];
        let rb = &mut sim.rigid_body_set[resident_handle];
        rb.set_translation(rapier3d::math::Vec3::new(11.0, 0.0, 0.0), true);

        // Evict
        evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);

        assert!(universe.pose_dirty_bodies.contains(&id));
        let evicted = &universe.universe_bodies[&id];
        assert_eq!(evicted.translation.x, 11.0);
    }

    #[test]
    fn rotated_body_aabb_is_conservative() {
        // Body 100 x 10 x 10, bounds 0..=99, 0..=9, 0..=9
        let min_ivec = IVec3::new(0, 0, 0);
        let max_ivec = IVec3::new(99, 9, 9);
        let com = rapier3d::glamx::DVec3::new(50.0, 5.0, 5.0);

        // Rotated 90 degrees around Y axis
        let rot_90_y = rapier3d::glamx::Quat::from_axis_angle(
            rapier3d::math::Vec3::new(0.0, 1.0, 0.0),
            std::f32::consts::FRAC_PI_2,
        );

        let translation = crate::scene::DVec3::new(0.0, 0.0, 0.0);
        let aabb = crate::scene::compute_universe_body_aabb(translation, rot_90_y, min_ivec, max_ivec, com);

        // Unrotated: X extent is 100 (half extent 50), Z extent is 10 (half extent 5)
        // Rotated 90 deg around Y: X extent should be 10 (half extent 5), Z extent should be 100 (half extent 50)
        let half_extents = (aabb.max - aabb.min) * 0.5;
        assert!((half_extents.x - 5.0).abs() < 1e-3, "X half extent should be ~5.0, got {}", half_extents.x);
        assert!((half_extents.y - 5.0).abs() < 1e-3, "Y half extent should be ~5.0, got {}", half_extents.y);
        assert!((half_extents.z - 50.0).abs() < 1e-3, "Z half extent should be ~50.0, got {}", half_extents.z);
    }

    #[test]
    fn universe_sublevel_block_change_updates_persistent_geometry() {
        let mut universe = crate::scene::DimensionUniverse::default();
        let id = 99;
        let mut b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        let mut chunk_map = std::collections::HashMap::new();
        let chunk = marten::level::ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        chunk_map.insert(pack_section_pos(0, 0, 0), chunk);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        // Mutate block at (2, 3, 4) in universe
        let sec_pos = pack_section_pos(0, 0, 0);
        if let Some(geom) = universe.body_geometries.get_mut(&id) {
            if let Some(cm) = &mut geom.chunk_map {
                let section = cm.get_mut(&sec_pos).unwrap();
                section.set_block(2, 3, 4, (42, VoxelPhysicsState::Interior));
            }
            geom.mark_section_dirty(0, 0, 0);
        }

        let updated_geom = &universe.body_geometries[&id];
        assert!(updated_geom.dirty_sections.contains(&sec_pos));
        let block = updated_geom.chunk_map.as_ref().unwrap()[&sec_pos].get_block(2, 3, 4);
        assert_eq!(block.0, 42);
    }

    #[test]
    fn zero_gravity_static_body_transitions_to_dormant() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let mut stationary = body(1);
        stationary.translation = crate::scene::DVec3::zeros();
        stationary.linear_velocity = Vec3::ZERO;
        stationary.angular_velocity = Vec3::ZERO;
        stationary.dynamics.gravity_scale = 0.0;
        stationary.simulation_tier = crate::scene::SimulationTier::Ballistic;
        universe.universe_bodies.insert(1, stationary);
        universe.spatial_index.update(1, crate::scene::UniverseAabb::around(crate::scene::DVec3::zeros(), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.schedule_body(1, 1);

        tick_universe(&mut universe, Vec3::ZERO, 0.05, 1);

        assert_eq!(universe.universe_bodies[&1].simulation_tier, crate::scene::SimulationTier::Dormant);
        assert_eq!(universe.pop_due_body(), None);
    }

    #[test]
    fn dormant_block_mutation_builds_octree_and_participates_in_collision() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 101;
        let mut b = body(id);
        b.translation = crate::scene::DVec3::zeros();
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(0, 0, 0));
        geom.local_bounds_max = Some(IVec3::new(15, 15, 15));
        geom.center_of_mass = Some(rapier3d::glamx::DVec3::new(8.0, 8.0, 8.0));

        let mut chunk_map = std::collections::HashMap::new();
        let chunk = marten::level::ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        chunk_map.insert(pack_section_pos(0, 0, 0), chunk);
        geom.chunk_map = Some(chunk_map);
        geom.octree = None; // Unbuilt octree
        universe.body_geometries.insert(id, geom);

        // Place a block at (4, 4, 4) in the dormant body
        let sec_key = pack_section_pos(0, 0, 0);
        let chunk_map = universe.body_geometries.get_mut(&id).unwrap().chunk_map.as_mut().unwrap();
        chunk_map.get_mut(&sec_key).unwrap().set_block(4, 4, 4, (1, VoxelPhysicsState::Face));
        universe.body_geometries.get_mut(&id).unwrap().mark_section_dirty(0, 0, 0);

        // Materialize the body
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        let collider_info = &sable.level_colliders[&id];
        assert!(collider_info.octree.is_some(), "Octree must be rebuilt on materialization");
        assert!(collider_info.section_octrees.contains_key(&sec_key), "Section octree must exist");
        assert!(!collider_info.section_octrees[&sec_key].is_empty(), "Section octree must contain mutated block");
    }

    #[test]
    fn gravity_body_falls_without_manual_wake() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let mut b = body(1);
        b.translation = crate::scene::DVec3::new(0.0, 100.0, 0.0);
        b.linear_velocity = Vec3::ZERO;
        b.dynamics.gravity_scale = 1.0;
        b.simulation_tier = crate::scene::SimulationTier::Ballistic;
        universe.universe_bodies.insert(1, b);
        universe.spatial_index.update(1, crate::scene::UniverseAabb::around(crate::scene::DVec3::new(0.0, 100.0, 0.0), crate::scene::DVec3::new(1.0, 1.0, 1.0)));
        universe.schedule_body(1, 1);

        tick_universe(&mut universe, Vec3::new(0.0, -11.0, 0.0), 0.05, 1);

        let updated = &universe.universe_bodies[&1];
        assert_eq!(updated.simulation_tier, crate::scene::SimulationTier::Ballistic);
        assert!(updated.translation.y < 100.0, "Body under gravity must fall during ballistic integration");
    }

    #[test]
    fn dormant_callback_block_tracks_serial_sections_across_materialization_and_eviction() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 102;
        let mut b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        let mut chunk_map = std::collections::HashMap::new();
        let mut chunk = marten::level::ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        chunk.serial_callback_blocks = 3; // Chunk has callback blocks
        let sec_key = pack_section_pos(0, 0, 0);
        chunk_map.insert(sec_key, chunk);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        assert_eq!(sable.sublevel_serial_callback_sections, 0);

        // Materialize into scene
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        assert_eq!(sable.sublevel_serial_callback_sections, 1, "Materializing body with callback blocks must increment serial callback sections");

        // Evict from scene
        evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);
        assert_eq!(sable.sublevel_serial_callback_sections, 0, "Evicting body must decrement serial callback sections");
        assert!(sable.level_colliders.get(&id).is_none(), "Eviction must remove ActiveLevelColliderInfo from scene");
    }

    #[test]
    fn mass_properties_identical_regardless_of_dormancy_during_stats_change() {
        let (mut sim1, sable_data1, mut universe1) = scene_data();
        let mut sable1 = sable_data1.write().unwrap();
        let id1 = 201;
        let b1 = body(id1);
        universe1.universe_bodies.insert(id1, b1);

        let inertia = rapier3d::math::Mat3::from_diagonal(rapier3d::math::Vec3::new(10.0, 20.0, 30.0));
        let props = MassProperties::with_inertia_matrix(rapier3d::math::Vec3::ZERO, 50.0, inertia);
        
        // 1. Update stats while dormant, then materialize
        universe1.universe_bodies.get_mut(&id1).unwrap().dynamics.additional_mass_properties = Some(props.clone());
        instantiate_rapier_body(&mut sim1, &mut sable1, &mut universe1, crate::scene::DVec3::zeros(), id1);
        let resident1 = universe1.universe_bodies[&id1].resident.as_ref().unwrap();
        let mp1 = sim1.rigid_body_set[resident1.rigid_body].mass_properties();

        // 2. Materialize first, then update stats while resident
        let (mut sim2, sable_data2, mut universe2) = scene_data();
        let mut sable2 = sable_data2.write().unwrap();
        let id2 = 202;
        let b2 = body(id2);
        universe2.universe_bodies.insert(id2, b2);
        instantiate_rapier_body(&mut sim2, &mut sable2, &mut universe2, crate::scene::DVec3::zeros(), id2);
        let resident2 = universe2.universe_bodies[&id2].resident.as_ref().unwrap();
        sim2.rigid_body_set[resident2.rigid_body].set_additional_mass_properties(props.clone(), true);
        let mp2 = sim2.rigid_body_set[resident2.rigid_body].mass_properties();

        assert_eq!(mp1.mass(), mp2.mass(), "Mass must be identical regardless of dormancy");
        assert_eq!(mp1.local_mprops.local_com, mp2.local_mprops.local_com, "Center of mass must be identical");
        assert_eq!(mp1.local_mprops.reconstruct_inertia_matrix(), mp2.local_mprops.reconstruct_inertia_matrix(), "Inertia must be identical");
    }

    #[test]
    fn re_materialization_before_eviction_drain_preserves_residency() {
        let (mut sim_a, sable_data_a, mut universe) = scene_data();
        let mut sable_a = sable_data_a.write().unwrap();
        let id = 301;
        let b = body(id);
        universe.universe_bodies.insert(id, b);

        // Materialize into Scene A
        instantiate_rapier_body(&mut sim_a, &mut sable_a, &mut universe, crate::scene::DVec3::zeros(), id);
        assert!(universe.universe_bodies[&id].resident.is_some());

        // Evict from Scene A -> records eviction event
        evict_rapier_body(&mut sim_a, &mut sable_a, &mut universe, crate::scene::DVec3::zeros(), id, true, true);
        assert!(universe.universe_bodies[&id].resident.is_none());
        assert_eq!(universe.eviction_events.len(), 1);

        // Materialize into Scene B before event is drained
        let (mut sim_b, sable_data_b, _) = scene_data();
        let mut sable_b = sable_data_b.write().unwrap();
        instantiate_rapier_body(&mut sim_b, &mut sable_b, &mut universe, crate::scene::DVec3::zeros(), id);
        
        // Eviction event for this id must be cleared upon rematerialization
        assert!(universe.eviction_events.is_empty(), "Materialization must clear stale eviction event for body");
        assert!(universe.universe_bodies[&id].resident.is_some());
    }

    #[test]
    fn resident_bounds_expansion_rebuilds_octree_and_preserves_after_evict_rematerialize() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 401;
        let b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(0, 0, 0));
        geom.local_bounds_max = Some(IVec3::new(15, 15, 15));
        let mut chunk_map = HashMap::new();
        let mut section0 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        section0.set_block(4, 4, 4, (1, VoxelPhysicsState::Face));
        chunk_map.insert(pack_section_pos(0, 0, 0), section0);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        // Materialize resident body
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        assert!(sable.level_colliders[&id].octree.is_some());

        // Add block at x=16 in section (1,0,0) to both persistent and resident chunk map
        let sec1_key = pack_section_pos(1, 0, 0);
        let mut section1 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        section1.set_block(0, 4, 4, (1, VoxelPhysicsState::Face)); // local_x=0 in section x=1 is global x=16
        universe.body_geometries.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().insert(sec1_key, section1.clone());
        sable.level_colliders.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().insert(sec1_key, section1);

        // Expand bounds from 0..15 to 0..16
        let min_ivec = IVec3::new(0, 0, 0);
        let max_ivec = IVec3::new(16, 15, 15);
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let SableSceneData {
            level_colliders,
            main_level_chunks,
            ..
        } = &mut *sable;
        let info = level_colliders.get_mut(&id).unwrap();
        info.octree = None;
        info.section_octrees.clear();
        info.octree_origin = None;
        info.dirty_sections.clear();
        info.set_local_bounds(min_ivec, max_ivec, main_level_chunks, collider_map);
        update_collider_aabb(&mut sim, info);

        // Persistent sync
        let geom = universe.body_geometries.entry(id).or_default();
        geom.local_bounds_min = Some(min_ivec);
        geom.local_bounds_max = Some(max_ivec);
        geom.octree = info.octree.clone();
        geom.section_octrees = info.section_octrees.clone();
        geom.octree_origin = info.octree_origin;

        let resident_info = &sable.level_colliders[&id];
        assert!(resident_info.section_octrees.contains_key(&sec1_key), "Expanded resident octree must contain section (1,0,0)");
        assert!(!resident_info.section_octrees[&sec1_key].is_empty(), "Section octree must contain block x=16");
        assert!(resident_info.local_bounds_max.unwrap().x >= 16);

        // Evict and rematerialize
        evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        let remat_info = &sable.level_colliders[&id];
        assert!(remat_info.section_octrees.contains_key(&sec1_key), "Rematerialized octree must retain section (1,0,0)");
        assert!(!remat_info.section_octrees[&sec1_key].is_empty(), "Rematerialized section must contain block x=16");
    }

    #[test]
    fn resident_negative_bounds_expansion_rebuilds_octree() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 402;
        let b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(0, 0, 0));
        geom.local_bounds_max = Some(IVec3::new(15, 15, 15));
        let mut chunk_map = HashMap::new();
        let mut section0 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        section0.set_block(4, 4, 4, (1, VoxelPhysicsState::Face));
        chunk_map.insert(pack_section_pos(0, 0, 0), section0);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        // Materialize resident body
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        // Add block at x=-1 in section (-1,0,0) to chunk maps
        let sec_neg_key = pack_section_pos(-1, 0, 0);
        let mut section_neg = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        section_neg.set_block(15, 4, 4, (1, VoxelPhysicsState::Face)); // local_x=15 in section x=-1 is global x=-1
        universe.body_geometries.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().insert(sec_neg_key, section_neg.clone());
        sable.level_colliders.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().insert(sec_neg_key, section_neg);

        // Expand bounds from 0..15 to -1..15
        let min_ivec = IVec3::new(-1, 0, 0);
        let max_ivec = IVec3::new(15, 15, 15);
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let SableSceneData {
            level_colliders,
            main_level_chunks,
            ..
        } = &mut *sable;
        let info = level_colliders.get_mut(&id).unwrap();
        info.octree = None;
        info.section_octrees.clear();
        info.octree_origin = None;
        info.dirty_sections.clear();
        info.set_local_bounds(min_ivec, max_ivec, main_level_chunks, collider_map);
        update_collider_aabb(&mut sim, info);

        let resident_info = &sable.level_colliders[&id];
        assert!(resident_info.section_octrees.contains_key(&sec_neg_key), "Negative expanded octree must contain section (-1,0,0)");
        assert!(!resident_info.section_octrees[&sec_neg_key].is_empty(), "Section octree must contain block x=-1");
    }

    #[test]
    fn test_1m_dormant_bodies_scale_and_tick() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let total = 1_000_000;
        for id in 0..total {
            let mut b = body(id);
            b.translation = crate::scene::DVec3::new((id % 1000) as f64 * 10.0, ((id / 1000) % 1000) as f64 * 10.0, 0.0);
            b.bounds = crate::scene::UniverseAabb::around(b.translation, crate::scene::DVec3::new(0.5, 0.5, 0.5));
            b.simulation_tier = crate::scene::SimulationTier::Dormant;
            universe.universe_bodies.insert(id, b);
        }
        assert_eq!(universe.universe_bodies.len(), total);
        tick_universe(&mut universe, Vec3::new(0.0, -9.81, 0.0), 0.05, 1);
        assert_eq!(universe.materialization_requests.len(), 0);
    }

    #[test]
    fn test_repeated_wake_sleep_cycle_100_bodies() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let count = 100;
        for id in 0..count {
            let mut b = body(id);
            b.translation = crate::scene::DVec3::new(id as f64 * 20.0, 50.0, 0.0);
            b.bounds = crate::scene::UniverseAabb::around(b.translation, crate::scene::DVec3::new(0.5, 0.5, 0.5));
            b.simulation_tier = crate::scene::SimulationTier::Dormant;
            universe.universe_bodies.insert(id, b);
        }

        for _cycle in 0..10 {
            // Wake / Materialize all 100
            for id in 0..count {
                instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
                assert!(universe.universe_bodies[&id].resident.is_some());
                assert!(sable.rigid_bodies.contains_key(&id));
            }

            // Evict / Sleep all 100
            for id in 0..count {
                let evicted = evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, true);
                assert!(evicted);
                assert!(universe.universe_bodies[&id].resident.is_none());
            }

            // Drain evictions
            let drained: Vec<LevelColliderID> = universe.eviction_events.drain(..).collect();
            assert_eq!(drained.len(), count);
        }
        assert_eq!(sable.rigid_bodies.len(), 0);
        assert_eq!(sim.rigid_body_set.len(), 0);
    }

    #[test]
    fn test_zero_g_1m_static_bodies_no_spurious_wakes() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let total = 1_000_000;
        for id in 0..total {
            let mut b = body(id);
            b.translation = crate::scene::DVec3::new((id % 1000) as f64 * 5.0, 0.0, (id / 1000) as f64 * 5.0);
            b.bounds = crate::scene::UniverseAabb::around(b.translation, crate::scene::DVec3::new(0.5, 0.5, 0.5));
            b.simulation_tier = crate::scene::SimulationTier::Dormant;
            b.dynamics.gravity_scale = 0.0;
            b.linear_velocity = Vec3::ZERO;
            universe.universe_bodies.insert(id, b);
        }
        for tick in 1..=5 {
            tick_universe(&mut universe, Vec3::ZERO, 0.05, tick);
            assert_eq!(universe.materialization_requests.len(), 0);
        }
    }

    #[test]
    fn test_high_speed_ballistic_cross_region_collision_sweep() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let obstacle_id = 99;
        let mut obstacle = body(obstacle_id);
        obstacle.translation = crate::scene::DVec3::new(100.0, 0.0, 0.0);
        obstacle.bounds = crate::scene::UniverseAabb::around(obstacle.translation, crate::scene::DVec3::new(2.0, 2.0, 2.0));
        obstacle.simulation_tier = crate::scene::SimulationTier::Dormant;
        universe.spatial_index.update(obstacle_id, obstacle.bounds);
        universe.universe_bodies.insert(obstacle_id, obstacle);

        let proj_id = 100;
        let mut proj = body(proj_id);
        proj.translation = crate::scene::DVec3::new(0.0, 0.0, 0.0);
        proj.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
        proj.bounds = crate::scene::UniverseAabb::around(proj.translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
        proj.simulation_tier = crate::scene::SimulationTier::Ballistic;
        proj.last_update_tick = 0;
        universe.spatial_index.update(proj_id, proj.bounds);
        universe.universe_bodies.insert(proj_id, proj);
        universe.schedule_body(proj_id, 1);

        tick_universe(&mut universe, Vec3::ZERO, 0.05, 1);
        assert!(
            universe.materialization_requests.iter().any(|req| req.id == proj_id || req.id == obstacle_id),
            "High speed ballistic body must wake upon swept intersection with dormant body"
        );
    }

    #[test]
    fn test_resident_structure_expand_shrink_repeatedly() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 501;
        let b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(0, 0, 0));
        geom.local_bounds_max = Some(IVec3::new(7, 7, 7));
        let mut chunk_map = HashMap::new();
        let mut sec0 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        sec0.set_block(2, 2, 2, (1, VoxelPhysicsState::Face));
        chunk_map.insert(pack_section_pos(0, 0, 0), sec0);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);

        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;

        for _cycle in 0..10 {
            // Expand to 31x15x15
            let min_exp = IVec3::new(0, 0, 0);
            let max_exp = IVec3::new(31, 15, 15);
            let sec1_key = pack_section_pos(1, 0, 0);
            let mut sec1 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
            sec1.set_block(5, 5, 5, (1, VoxelPhysicsState::Face));
            sable.level_colliders.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().insert(sec1_key, sec1);
            
            let SableSceneData { level_colliders, main_level_chunks, .. } = &mut *sable;
            let info = level_colliders.get_mut(&id).unwrap();
            info.octree = None;
            info.section_octrees.clear();
            info.octree_origin = None;
            info.dirty_sections.clear();
            info.set_local_bounds(min_exp, max_exp, main_level_chunks, collider_map);
            update_collider_aabb(&mut sim, info);
            assert!(info.section_octrees.contains_key(&sec1_key));

            // Shrink back to 7x7x7
            let min_sh = IVec3::new(0, 0, 0);
            let max_sh = IVec3::new(7, 7, 7);
            sable.level_colliders.get_mut(&id).unwrap().chunk_map.as_mut().unwrap().remove(&sec1_key);
            let SableSceneData { level_colliders: lc2, main_level_chunks: mlc2, .. } = &mut *sable;
            let info2 = lc2.get_mut(&id).unwrap();
            info2.octree = None;
            info2.section_octrees.clear();
            info2.octree_origin = None;
            info2.dirty_sections.clear();
            info2.set_local_bounds(min_sh, max_sh, mlc2, collider_map);
            update_collider_aabb(&mut sim, info2);
            assert!(!info2.section_octrees.contains_key(&sec1_key));
        }

        // Evict and rematerialize
        evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        assert!(sable.level_colliders[&id].octree.is_some());
    }

    #[test]
    fn test_dormant_block_mutation_materialize_and_collide() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 601;
        let mut b = body(id);
        b.translation = crate::scene::DVec3::new(0.0, 0.0, 0.0);
        b.simulation_tier = crate::scene::SimulationTier::Dormant;
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        geom.local_bounds_min = Some(IVec3::new(0, 0, 0));
        geom.local_bounds_max = Some(IVec3::new(15, 15, 15));
        let mut chunk_map = HashMap::new();
        let mut sec0 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0);
        sec0.set_block(5, 5, 5, (1, VoxelPhysicsState::Face));
        chunk_map.insert(pack_section_pos(0, 0, 0), sec0);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        // Materialize
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
        let resident_info = &sable.level_colliders[&id];
        assert!(resident_info.octree.is_some());
        assert!(!resident_info.section_octrees[&pack_section_pos(0, 0, 0)].is_empty());
    }

    #[test]
    fn test_constraints_between_unresident_bodies_prevent_split() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id1 = 701;
        let id2 = 702;
        universe.universe_bodies.insert(id1, body(id1));
        universe.universe_bodies.insert(id2, body(id2));

        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id1);
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id2);

        let h1 = sable.rigid_bodies[&id1];
        let h2 = sable.rigid_bodies[&id2];

        // Attach spherical joint
        let joint = rapier3d::dynamics::SphericalJointBuilder::new().local_anchor1(Vec3::ZERO).local_anchor2(Vec3::new(0.0, 1.0, 0.0)).build();
        sim.impulse_joint_set.insert(h1, h2, joint, true);

        // Attempt non-forced eviction on body 1
        let evicted = evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id1, false, true);
        assert!(!evicted, "Constrained body must not be evicted without force");
        assert!(universe.universe_bodies[&id1].resident.is_some());
        assert!(universe.universe_bodies[&id2].resident.is_some());
    }

    #[test]
    fn test_extreme_10b_coordinate_move_rotate_rebase() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 801;
        let extreme_pos = crate::scene::DVec3::new(10_000_000_000.0, 500.0, -10_000_000_000.0);
        let mut b = body(id);
        b.translation = extreme_pos;
        b.rotation = rapier3d::math::Rotation::from_rotation_y(0.785398); // 45 deg
        b.bounds = crate::scene::UniverseAabb::around(extreme_pos, crate::scene::DVec3::new(2.0, 2.0, 2.0));
        universe.spatial_index.update(id, b.bounds);
        universe.universe_bodies.insert(id, b);

        // Instantiate with scene origin at extreme_pos
        instantiate_rapier_body(&mut sim, &mut sable, &mut universe, extreme_pos, id);
        let handle = sable.rigid_bodies[&id];
        let local_pos = sim.rigid_body_set[handle].translation();
        assert!(local_pos.length_squared() < 1e-4, "Local position at matching origin should be ~zero");

        // Rebase origin by +50 on X
        let new_origin = extreme_pos + crate::scene::DVec3::new(50.0, 0.0, 0.0);
        let delta = new_origin - extreme_pos;
        let delta_f32 = Vec3::new(delta.x as f32, delta.y as f32, delta.z as f32);
        for (_bid, rb_handle) in &sable.rigid_bodies {
            let rb = sim.rigid_body_set.get_mut(*rb_handle).unwrap();
            let mut pos = rb.translation().clone();
            pos -= delta_f32;
            rb.set_translation(pos, true);
        }

        let rebased_local = sim.rigid_body_set[handle].translation();
        assert!((rebased_local.x - (-50.0)).abs() < 1e-4);

        // Evict with snapshot
        evict_rapier_body(&mut sim, &mut sable, &mut universe, new_origin, id, true, false);
        assert_eq!(universe.universe_bodies[&id].translation, extreme_pos);
    }

    #[test]
    fn test_10k_retained_sleeping_regions_scale() {
        let (_sim, _sable_data, mut universe) = scene_data();
        let total_regions = 10_000;
        for i in 0..total_regions {
            let min = crate::scene::DVec3::new((i as f64) * 200.0, 0.0, 0.0);
            let max = min + crate::scene::DVec3::new(100.0, 100.0, 100.0);
            let aabb = crate::scene::UniverseAabb { min, max };
            universe.spatial_index.update(i, aabb);
        }

        let query_box = crate::scene::UniverseAabb {
            min: crate::scene::DVec3::new(50.0, 0.0, 0.0),
            max: crate::scene::DVec3::new(250.0, 100.0, 100.0),
        };
        let hits = universe.spatial_index.query(query_box, usize::MAX);
        assert_eq!(hits.len(), 2);
        assert!(hits.contains(&0));
        assert!(hits.contains(&1));
    }

    #[test]
    fn test_java_callback_blocks_across_sleep_wake_cycles() {
        let (mut sim, sable_data, mut universe) = scene_data();
        let mut sable = sable_data.write().unwrap();
        let id = 901;
        let mut b = body(id);
        universe.universe_bodies.insert(id, b);

        let mut geom = crate::scene::PersistentBodyGeometry::default();
        let mut chunk_map = HashMap::new();
        let sec0 = ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 3); // 3 callback blocks
        chunk_map.insert(pack_section_pos(0, 0, 0), sec0);
        geom.chunk_map = Some(chunk_map);
        universe.body_geometries.insert(id, geom);

        for _ in 0..20 {
            instantiate_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id);
            assert_eq!(sable.sublevel_serial_callback_sections, 1);
            assert!(!sable.can_parallel_step());

            evict_rapier_body(&mut sim, &mut sable, &mut universe, crate::scene::DVec3::zeros(), id, true, false);
            assert_eq!(sable.sublevel_serial_callback_sections, 0);
            assert!(sable.can_parallel_step());
        }
    }
}



#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_registerUniverseBody<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    pose_arr: jni::sys::jdoubleArray,
) {
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let pose_arr_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(pose_arr) };
    let pose_elems = unsafe { env.get_array_elements_critical(&pose_arr_obj, jni::objects::ReleaseMode::NoCopyBack).unwrap() }; // (&pose_arr_obj, jni::objects::ReleaseMode::NoCopyBack).unwrap();
    let translation = crate::scene::DVec3::new(pose_elems[0], pose_elems[1], pose_elems[2]);
    let rotation = rapier3d::math::Rotation::from_xyzw(
        pose_elems[3] as f32, pose_elems[4] as f32, pose_elems[5] as f32, pose_elems[6] as f32,
    );
    let bounds = crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(0.5, 0.5, 0.5));
    let id_usize = id as usize;
    let initial_tier = crate::scene::SimulationTier::Ballistic;
    let current_tick = universe.current_tick;
    let body = crate::scene::UniverseBody {
        id: id_usize,
        translation,
        rotation,
        linear_velocity: rapier3d::math::Vec3::ZERO,
        angular_velocity: rapier3d::math::Vec3::ZERO,
        dynamics: crate::scene::BodyDynamics {
            additional_mass_properties: None,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            locked_axes: rapier3d::dynamics::LockedAxes::empty(),
            ccd_enabled: false,
        },
        simulation_tier: initial_tier,
        bounds,
        last_update_tick: current_tick,
        next_update_tick: current_tick + 1,
        schedule_generation: 0,
        resident: None,
        assembly_root: id_usize,
        assembly_size: 1,
        command_queue: Vec::new(),
    };
    universe.universe_bodies.insert(id_usize, body);
    universe.spatial_index.update(id_usize, bounds);
    universe.schedule_body(id_usize, current_tick + 1);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_materializeBody<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    region_handle: jni::sys::jlong,
) {
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    
    let scene_ptr = region_handle as *mut crate::scene::PhysicsScene;
    let scene = unsafe { &*scene_ptr };
    let mut sim = scene.sim_data.write().unwrap();
    let mut sable = scene.sable_data.write().unwrap();
    let world_origin = *scene.world_origin.read().unwrap();
    
    crate::instantiate_rapier_body(&mut sim, &mut sable, &mut universe, world_origin, id as usize);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_evictBody<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    region_handle: jni::sys::jlong,
) {
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    
    let scene_ptr = region_handle as *mut crate::scene::PhysicsScene;
    let scene = unsafe { &*scene_ptr };
    let mut sim = scene.sim_data.write().unwrap();
    let mut sable = scene.sable_data.write().unwrap();
    let world_origin = *scene.world_origin.read().unwrap();
    
    crate::evict_rapier_body(&mut sim, &mut sable, &mut universe, world_origin, id as usize, true, true);
}



#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_flushUniverseCommands<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass,
    universe_handle: jni::sys::jlong,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();

    let pending_ids: Vec<LevelColliderID> = universe.pending_command_bodies.iter().copied().collect();
    for id in pending_ids {
        let (resident_opt, is_empty) = if let Some(body) = universe.universe_bodies.get(&id) {
            (body.resident.clone(), body.command_queue.is_empty())
        } else {
            universe.pending_command_bodies.remove(&id);
            continue;
        };

        if is_empty {
            universe.pending_command_bodies.remove(&id);
            continue;
        }

        let resident = match resident_opt {
            Some(res) => res,
            None => {
                universe.request_materialization(id);
                if let Some(body) = universe.universe_bodies.get_mut(&id) {
                    body.simulation_tier = crate::scene::SimulationTier::Active;
                }
                let current_tick = universe.current_tick;
                universe.schedule_body(id, current_tick + 1);
                continue;
            }
        };

        let scene_ptr = resident.scene_handle as *mut crate::scene::PhysicsScene;
        if scene_ptr.is_null() {
            continue;
        }
        let scene = unsafe { &*scene_ptr };
        let mut sim = scene.sim_data.write().unwrap();

        if let Some(rigid_body) = sim.rigid_body_set.get_mut(resident.rigid_body) {
            let commands: Vec<_> = if let Some(body) = universe.universe_bodies.get_mut(&id) {
                body.command_queue.drain(..).collect()
            } else {
                Vec::new()
            };

            for command in commands {
                match command {
                    crate::scene::UniverseCommand::ApplyForce { x, y, z, fx, fy, fz, wake_up } => {
                        let point = rapier3d::math::Vec3::new(x as f32, y as f32, z as f32);
                        let force = rapier3d::math::Vec3::new(fx as f32, fy as f32, fz as f32);
                        rigid_body.add_force_at_point(force, point.into(), wake_up);
                    }
                    crate::scene::UniverseCommand::ApplyForceAndTorque { fx, fy, fz, tx, ty, tz, wake_up } => {
                        let force = rapier3d::math::Vec3::new(fx as f32, fy as f32, fz as f32);
                        let torque = rapier3d::math::Vec3::new(tx as f32, ty as f32, tz as f32);
                        rigid_body.add_force(force, wake_up);
                        rigid_body.add_torque(torque, wake_up);
                    }
                    crate::scene::UniverseCommand::AddLinearAngularVelocities { linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, wake_up } => {
                        let linvel = rigid_body.linvel() + rapier3d::math::Vec3::new(linear_x as f32, linear_y as f32, linear_z as f32);
                        let angvel = rigid_body.angvel() + rapier3d::math::Vec3::new(angular_x as f32, angular_y as f32, angular_z as f32);
                        rigid_body.set_linvel(linvel, wake_up);
                        rigid_body.set_angvel(angvel, wake_up);
                    }
                    crate::scene::UniverseCommand::WakeUp => {
                        rigid_body.wake_up(true);
                    }
                    crate::scene::UniverseCommand::SetKinematicContraptionTransform { center_of_mass: _, pose, velocities } => {
                        let translation = rapier3d::math::Vec3::new(pose[0] as f32, pose[1] as f32, pose[2] as f32);
                        let rotation = rapier3d::glamx::Quat::from_xyzw(pose[3] as f32, pose[4] as f32, pose[5] as f32, pose[6] as f32);
                        let isometry = rapier3d::glamx::Pose3 { translation, rotation };
                        rigid_body.set_next_kinematic_position(isometry);
                        let linvel = rapier3d::math::Vec3::new(velocities[0] as f32, velocities[1] as f32, velocities[2] as f32);
                        let angvel = rapier3d::math::Vec3::new(velocities[3] as f32, velocities[4] as f32, velocities[5] as f32);
                        rigid_body.set_linvel(linvel, true);
                        rigid_body.set_angvel(angvel, true);
                    }
                }
            }
            universe.pending_command_bodies.remove(&id);
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_tickUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    absolute_tick: jni::sys::jlong,
    time_step: jni::sys::jdouble,
    gx: jni::sys::jdouble,
    gy: jni::sys::jdouble,
    gz: jni::sys::jdouble,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let gravity = rapier3d::math::Vec3::new(gx as Real, gy as Real, gz as Real);
    tick_universe(&mut universe, gravity, time_step as marten::Real, absolute_tick as u64);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_writeMaterializationRequests<
    'local,
>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    universe_handle: jlong,
    buffer: jni::objects::JObject<'local>,
    max_requests: jint,
) -> jint {
    if universe_handle == 0 {
        return 0;
    }
    let capacity = env
        .get_direct_buffer_capacity((&buffer).into())
        .unwrap_or(0);
    let data_ptr = env
        .get_direct_buffer_address((&buffer).into())
        .unwrap_or(std::ptr::null_mut());

    if data_ptr.is_null() || capacity == 0 {
        return 0;
    }

    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();

    let request_count = universe.materialization_requests.len();
    if request_count == 0 {
        return 0;
    }

    let max_allowed = std::cmp::min(max_requests.max(0) as usize, capacity / 32);
    if request_count > max_allowed {
        return -(request_count.min(jint::MAX as usize) as jint);
    }

    let requests: Vec<_> = universe.materialization_requests.drain(..).collect();
    universe.pending_materializations.clear();

    let output = unsafe { std::slice::from_raw_parts_mut(data_ptr, requests.len() * 32) };
    for (i, req) in requests.iter().enumerate() {
        let offset = i * 32;
        let id_bytes = (req.id as i32).to_ne_bytes();
        output[offset..offset + 4].copy_from_slice(&id_bytes);
        output[offset + 4..offset + 8].copy_from_slice(&[0u8; 4]);
        let x_bytes = req.position.x.to_ne_bytes();
        output[offset + 8..offset + 16].copy_from_slice(&x_bytes);
        let y_bytes = req.position.y.to_ne_bytes();
        output[offset + 16..offset + 24].copy_from_slice(&y_bytes);
        let z_bytes = req.position.z.to_ne_bytes();
        output[offset + 24..offset + 32].copy_from_slice(&z_bytes);
    }

    requests.len() as jint
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_drainMaterializationRequests<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
) -> jni::sys::jintArray {
    if universe_handle == 0 {
        return env.new_int_array(0).unwrap().into_raw();
    }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let requests: Vec<i32> = universe.materialization_requests.drain(..).map(|req| req.id as i32).collect();
    universe.pending_materializations.clear();
    let array = env.new_int_array(requests.len() as jni::sys::jsize).unwrap();
    env.set_int_array_region(&array, 0, &requests).unwrap();
    array.into_raw()
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_drainEvictionEvents<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
) -> jni::sys::jintArray {
    if universe_handle == 0 {
        return env.new_int_array(0).unwrap().into_raw();
    }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let evictions: Vec<i32> = universe.eviction_events.drain(..).map(|id| id as i32).collect();
    universe.pending_evictions.clear();
    let array = env.new_int_array(evictions.len() as jni::sys::jsize).unwrap();
    env.set_int_array_region(&array, 0, &evictions).unwrap();
    array.into_raw()
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_applyImpulseUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    x: jni::sys::jdouble,
    y: jni::sys::jdouble,
    z: jni::sys::jdouble,
    fx: jni::sys::jdouble,
    fy: jni::sys::jdouble,
    fz: jni::sys::jdouble,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;
    if let Some(body) = universe.universe_bodies.get_mut(&id_usize) {
        body.command_queue.push(crate::scene::UniverseCommand::ApplyForce {
            x: x as f64,
            y: y as f64,
            z: z as f64,
            fx: fx as f64,
            fy: fy as f64,
            fz: fz as f64,
            wake_up: true,
        });
        universe.pending_command_bodies.insert(id_usize);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_applyForceAndTorqueUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    fx: jni::sys::jdouble,
    fy: jni::sys::jdouble,
    fz: jni::sys::jdouble,
    tx: jni::sys::jdouble,
    ty: jni::sys::jdouble,
    tz: jni::sys::jdouble,
    wake_up: jni::sys::jboolean,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;
    if let Some(body) = universe.universe_bodies.get_mut(&id_usize) {
        body.command_queue.push(crate::scene::UniverseCommand::ApplyForceAndTorque {
            fx: fx as f64,
            fy: fy as f64,
            fz: fz as f64,
            tx: tx as f64,
            ty: ty as f64,
            tz: tz as f64,
            wake_up: wake_up != 0,
        });
        universe.pending_command_bodies.insert(id_usize);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addLinearAngularVelocityUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    lx: jni::sys::jdouble,
    ly: jni::sys::jdouble,
    lz: jni::sys::jdouble,
    ax: jni::sys::jdouble,
    ay: jni::sys::jdouble,
    az: jni::sys::jdouble,
    wake_up: jni::sys::jboolean,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;
    if let Some(body) = universe.universe_bodies.get_mut(&id_usize) {
        body.command_queue.push(crate::scene::UniverseCommand::AddLinearAngularVelocities {
            linear_x: lx as f64,
            linear_y: ly as f64,
            linear_z: lz as f64,
            angular_x: ax as f64,
            angular_y: ay as f64,
            angular_z: az as f64,
            wake_up: wake_up != 0,
        });
        universe.pending_command_bodies.insert(id_usize);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_wakeUpUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;
    if let Some(body) = universe.universe_bodies.get_mut(&id_usize) {
        body.command_queue.push(crate::scene::UniverseCommand::WakeUp);
        universe.pending_command_bodies.insert(id_usize);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_teleportUniverse<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    x: jni::sys::jdouble,
    y: jni::sys::jdouble,
    z: jni::sys::jdouble,
    qx: jni::sys::jdouble,
    qy: jni::sys::jdouble,
    qz: jni::sys::jdouble,
    qw: jni::sys::jdouble,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;
    let translation = crate::scene::DVec3::new(x as f64, y as f64, z as f64);
    let rotation = rapier3d::glamx::Quat::from_xyzw(qx as f32, qy as f32, qz as f32, qw as f32);
    
    let (resident_opt, updated) = if let Some(body) = universe.universe_bodies.get_mut(&id_usize) {
        body.translation = translation;
        body.rotation = rotation;
        body.linear_velocity = rapier3d::math::Vec3::ZERO;
        body.angular_velocity = rapier3d::math::Vec3::ZERO;
        (body.resident.clone(), true)
    } else {
        (None, false)
    };

    if updated {
        let new_bounds = if let Some(geom) = universe.body_geometries.get(&id_usize) {
            let min_ivec = geom.local_bounds_min.unwrap_or(IVec3::splat(-1));
            let max_ivec = geom.local_bounds_max.unwrap_or(IVec3::splat(1));
            let com = geom.center_of_mass.unwrap_or(rapier3d::glamx::DVec3::ZERO);
            let aabb = crate::scene::compute_universe_body_aabb(translation, rotation, min_ivec, max_ivec, com);
            let body = universe.universe_bodies.get_mut(&id_usize).unwrap();
            body.bounds = aabb;
            aabb
        } else {
            let aabb = crate::scene::UniverseAabb::around(translation, crate::scene::DVec3::new(1.0, 1.0, 1.0));
            let body = universe.universe_bodies.get_mut(&id_usize).unwrap();
            body.bounds = aabb;
            aabb
        };

        universe.spatial_index.update(id_usize, new_bounds);
        universe.pose_dirty_bodies.insert(id_usize);
    }

    if let Some(resident) = resident_opt {
        let scene_ptr = resident.scene_handle as *mut crate::scene::PhysicsScene;
        if !scene_ptr.is_null() {
            let scene = unsafe { &*scene_ptr };
            let mut sim = scene.sim_data.write().unwrap();
            if let Some(rb) = sim.rigid_body_set.get_mut(resident.rigid_body) {
                let local_pos = scene.global_to_local(translation);
                let pose = rapier3d::glamx::Pose3 { translation: local_pos, rotation };
                rb.set_position(pose, true);
                rb.set_linvel(rapier3d::math::Vec3::ZERO, true);
                rb.set_angvel(rapier3d::math::Vec3::ZERO, true);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeUniverseBody<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let mut sim = scene.sim_data.write().unwrap();
                let mut sable = scene.sable_data.write().unwrap();
                let world_origin = *scene.world_origin.read().unwrap();
                evict_rapier_body(&mut sim, &mut sable, &mut universe, world_origin, id_usize, true, false);
                sable.level_colliders.remove(&id_usize);
            }
        }
    }

    universe.spatial_index.remove(id_usize);
    universe.universe_bodies.remove(&id_usize);
    universe.body_geometries.remove(&id_usize);
    universe.pose_dirty_bodies.remove(&id_usize);
    universe.pending_command_bodies.remove(&id_usize);
    universe.pending_materializations.remove(&id_usize);
    universe.pending_evictions.remove(&id_usize);
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setUniverseBodyStats<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    mass: jni::sys::jdouble,
    center_of_mass_arr: jni::sys::jdoubleArray,
    inertia_tensor_arr: jni::sys::jdoubleArray,
    min_x: jni::sys::jint,
    min_y: jni::sys::jint,
    min_z: jni::sys::jint,
    max_x: jni::sys::jint,
    max_y: jni::sys::jint,
    max_z: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;

    let com = if center_of_mass_arr.is_null() {
        rapier3d::glamx::DVec3::ZERO
    } else {
        let com_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(center_of_mass_arr) };
        let mut com_elems = [0.0f64; 3];
        env.get_double_array_region(&com_obj, 0, &mut com_elems).unwrap();
        rapier3d::glamx::DVec3::new(com_elems[0], com_elems[1], com_elems[2])
    };

    let mass_props = if inertia_tensor_arr.is_null() || mass <= 0.0 {
        None
    } else {
        let inertia_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(inertia_tensor_arr) };
        let mut inertia_elems = [0.0f64; 9];
        env.get_double_array_region(&inertia_obj, 0, &mut inertia_elems).unwrap();
        let inertia_matrix = rapier3d::math::Mat3::from_cols_array(&[
            inertia_elems[0] as f32, inertia_elems[1] as f32, inertia_elems[2] as f32,
            inertia_elems[3] as f32, inertia_elems[4] as f32, inertia_elems[5] as f32,
            inertia_elems[6] as f32, inertia_elems[7] as f32, inertia_elems[8] as f32,
        ]);
        Some(MassProperties::with_inertia_matrix(Vec3::ZERO, mass as Real, inertia_matrix))
    };

    let min_ivec = IVec3::new(min_x, min_y, min_z);
    let max_ivec = IVec3::new(max_x, max_y, max_z);

    let geom = universe.body_geometries.entry(id_usize).or_default();
    let bounds_changed = geom.local_bounds_min != Some(min_ivec) || geom.local_bounds_max != Some(max_ivec);
    geom.center_of_mass = Some(com);
    geom.local_bounds_min = Some(min_ivec);
    geom.local_bounds_max = Some(max_ivec);
    if bounds_changed {
        geom.octree = None;
        geom.section_octrees.clear();
        geom.octree_origin = None;
    }

    let (bounds_opt, resident_opt) = if let Some(ubody) = universe.universe_bodies.get_mut(&id_usize) {
        ubody.dynamics.additional_mass_properties = mass_props.clone();
        ubody.bounds = crate::scene::compute_universe_body_aabb(ubody.translation, ubody.rotation, min_ivec, max_ivec, com);
        let bounds = ubody.bounds;
        let res = ubody.resident.clone();
        (Some(bounds), res)
    } else {
        (None, None)
    };

    if let Some(bounds) = bounds_opt {
        universe.spatial_index.update(id_usize, bounds);
    }

    if let Some(resident) = resident_opt {
        let scene_ptr = resident.scene_handle as *mut PhysicsScene;
        if !scene_ptr.is_null() {
            let scene = unsafe { &*scene_ptr };
            let mut sim = scene.sim_data.write().unwrap();
            let mut sable = scene.sable_data.write().unwrap();
            let SableSceneData {
                level_colliders,
                main_level_chunks,
                ..
            } = &mut *sable;
            if let Some(rb) = sim.rigid_body_set.get_mut(resident.rigid_body) {
                if let Some(props) = &mass_props {
                    rb.set_additional_mass_properties(props.clone(), true);
                } else {
                    rb.set_additional_mass_properties(
                        MassProperties::with_inertia_matrix(Vec3::ZERO, 0.0, rapier3d::math::Mat3::ZERO),
                        true,
                    );
                }
            }
            if let Some(info) = level_colliders.get_mut(&id_usize) {
                info.center_of_mass = Some(com);
                if bounds_changed {
                    let physics_state = get_physics_state();
                    let collider_map = &physics_state.voxel_collider_map;
                    info.octree = None;
                    info.section_octrees.clear();
                    info.octree_origin = None;
                    info.dirty_sections.clear();
                    info.set_local_bounds(min_ivec, max_ivec, main_level_chunks, collider_map);

                    let geom = universe.body_geometries.entry(id_usize).or_default();
                    geom.octree = info.octree.clone();
                    geom.section_octrees = info.section_octrees.clone();
                    geom.octree_origin = info.octree_origin;
                    geom.dirty_sections.clear();
                } else {
                    info.local_bounds_min = Some(min_ivec);
                    info.local_bounds_max = Some(max_ivec);
                }
                update_collider_aabb(&mut sim, info);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addUniverseSubLevelChunk<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    x: jni::sys::jint,
    y: jni::sys::jint,
    z: jni::sys::jint,
    data: jni::objects::JIntArray<'local>,
) {
    if universe_handle == 0 { return; }
    let mut ints: [jni::sys::jint; 4096] = [0; 4096];
    env.get_int_array_region(data, 0, &mut ints).unwrap();

    let mut blocks = Vec::with_capacity(ints.len());
    for block in ints {
        let block_collider_id = (block >> 16) as u16;
        let voxel_state_id = (block & 0xFFFF) as u16;
        blocks.push((
            block_collider_id as u32,
            ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
        ));
    }

    let physics_state = get_physics_state();
    let collider_map = &physics_state.voxel_collider_map;
    let chunk_serial_callback_blocks = blocks
        .iter()
        .filter(|block| collider_map.requires_java_callback(block.0 as usize))
        .count() as u16;
    let chunk = ChunkSection::with_serial_step(blocks, chunk_serial_callback_blocks);

    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;

    let geom = universe.body_geometries.entry(id_usize).or_default();
    if geom.chunk_map.is_none() {
        geom.chunk_map = Some(HashMap::new());
    }
    let key = pack_section_pos(x, y, z);
    geom.chunk_map.as_mut().unwrap().insert(key, chunk.clone());
    geom.mark_section_dirty(x, y, z);

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let mut sable = scene.sable_data.write().unwrap();
                let mut delta_callbacks = 0i32;
                if let Some(body) = sable.level_colliders.get_mut(&id_usize) {
                    if body.chunk_map.is_none() {
                        body.chunk_map = Some(HashMap::new());
                    }
                    body.insert_chunk(&chunk, x, y, z, collider_map);
                    if let Some(old) = body.chunk_map.as_mut().unwrap().insert(key, chunk) {
                        if old.serial_callback_blocks > 0 {
                            delta_callbacks -= 1;
                        }
                    }
                    if chunk_serial_callback_blocks > 0 {
                        delta_callbacks += 1;
                    }
                    body.mark_section_dirty(x, y, z);
                }
                if delta_callbacks < 0 {
                    sable.sublevel_serial_callback_sections = sable.sublevel_serial_callback_sections.saturating_sub((-delta_callbacks) as u32);
                } else if delta_callbacks > 0 {
                    sable.sublevel_serial_callback_sections += delta_callbacks as u32;
                }
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeUniverseSubLevelChunk<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    x: jni::sys::jint,
    y: jni::sys::jint,
    z: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;

    if let Some(geom) = universe.body_geometries.get_mut(&id_usize) {
        if let Some(chunk_map) = &mut geom.chunk_map {
            chunk_map.remove(&pack_section_pos(x, y, z));
        }
        geom.mark_section_dirty(x, y, z);
    }

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let mut sable = scene.sable_data.write().unwrap();
                let mut removed_callbacks = 0u32;
                if let Some(body) = sable.level_colliders.get_mut(&id_usize) {
                    if let Some(chunk_map) = &mut body.chunk_map {
                        if let Some(old) = chunk_map.remove(&pack_section_pos(x, y, z)) {
                            if old.serial_callback_blocks > 0 {
                                removed_callbacks += 1;
                            }
                        }
                    }
                    body.mark_section_dirty(x, y, z);
                }
                if removed_callbacks > 0 {
                    sable.sublevel_serial_callback_sections = sable.sublevel_serial_callback_sections.saturating_sub(removed_callbacks);
                }
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addWorldTerrainChunk<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    x: jni::sys::jint,
    y: jni::sys::jint,
    z: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    if !crate::scene::is_inside_terrain_domain(x, y, z) { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    universe.terrain_sections.insert(pack_section_pos(x, y, z));
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeWorldTerrainChunk<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    x: jni::sys::jint,
    y: jni::sys::jint,
    z: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    if !crate::scene::is_inside_terrain_domain(x, y, z) { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    universe.terrain_sections.remove(&pack_section_pos(x, y, z));
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_writeUniverseDirtyPoses<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    buffer: jni::objects::JObject<'local>,
    max_bodies: jni::sys::jint,
) -> jni::sys::jint {
    if universe_handle == 0 {
        return 0;
    }
    let capacity = env.get_direct_buffer_capacity((&buffer).into()).unwrap_or(0);
    let data_ptr = env.get_direct_buffer_address((&buffer).into()).unwrap_or(std::ptr::null_mut());
    if data_ptr.is_null() || capacity == 0 {
        return 0;
    }

    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();

    let dirty_ids: Vec<LevelColliderID> = universe.pose_dirty_bodies.drain().collect();
    let count = dirty_ids.len();
    if count == 0 {
        return 0;
    }

    let max_allowed = std::cmp::min(max_bodies.max(0) as usize, capacity / 60);
    if count > max_allowed {
        for &id in &dirty_ids {
            universe.pose_dirty_bodies.insert(id);
        }
        return -(count.min(jni::sys::jint::MAX as usize) as jni::sys::jint);
    }

    let mut poses = Vec::with_capacity(count);
    for id in dirty_ids {
        if let Some(body) = universe.universe_bodies.get(&id) {
            poses.push(ExportPose {
                id: id as i32,
                position: body.translation,
                rotation: body.rotation,
            });
        }
    }

    let output = unsafe { std::slice::from_raw_parts_mut(data_ptr, poses.len() * 60) };
    encode_active_poses(&poses, output);
    poses.len() as jni::sys::jint
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_changeUniverseSubLevelBlock<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    x: jni::sys::jint,
    y: jni::sys::jint,
    z: jni::sys::jint,
    packed_state: jni::sys::jint,
) {
    if universe_handle == 0 { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let mut universe = unsafe { &*universe_ptr }.write().unwrap();
    let id_usize = id as LevelColliderID;

    let block_collider_id = (packed_state >> 16) as u16;
    let voxel_state_id = (packed_state & 0xFFFF) as u16;
    let block = (
        block_collider_id as u32,
        ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
    );

    let sec_x = x >> CHUNK_SHIFT;
    let sec_y = y >> CHUNK_SHIFT;
    let sec_z = z >> CHUNK_SHIFT;
    let local_x = (x & 15) as usize;
    let local_y = (y & 15) as usize;
    let local_z = (z & 15) as usize;

    if let Some(geom) = universe.body_geometries.get_mut(&id_usize) {
        let sec_key = pack_section_pos(sec_x, sec_y, sec_z);
        let chunk_map = geom.chunk_map.get_or_insert_with(HashMap::new);
        let section = chunk_map.entry(sec_key).or_insert_with(|| {
            ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0)
        });
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let old = section.get_block(local_x as i32, local_y as i32, local_z as i32).0 as usize;
        let old_cb = collider_map.requires_java_callback(old);
        let new_cb = collider_map.requires_java_callback(block.0 as usize);
        if old_cb != new_cb {
            if new_cb {
                section.serial_callback_blocks += 1;
            } else {
                section.serial_callback_blocks = section.serial_callback_blocks.saturating_sub(1);
            }
        }
        section.set_block(local_x as i32, local_y as i32, local_z as i32, block);
        geom.mark_section_dirty(sec_x, sec_y, sec_z);
    }

    let resident_opt = universe.universe_bodies.get(&id_usize).and_then(|ubody| ubody.resident.clone());

    if let Some(resident) = resident_opt {
        let scene_ptr = resident.scene_handle as *mut PhysicsScene;
        if !scene_ptr.is_null() {
            let scene = unsafe { &*scene_ptr };
            let mut sable = scene.sable_data.write().unwrap();
            let mut sim = scene.sim_data.write().unwrap();
            let physics_state = get_physics_state();
            let collider_map = &physics_state.voxel_collider_map;

            let SableSceneData {
                level_colliders,
                sublevel_serial_callback_sections,
                ..
            } = &mut *sable;

            if let Some(body) = level_colliders.get_mut(&id_usize) {
                let sec_key = pack_section_pos(sec_x, sec_y, sec_z);
                let chunk_map = body.chunk_map.get_or_insert_with(HashMap::new);
                let chunk = chunk_map.entry(sec_key).or_insert_with(|| {
                    ChunkSection::with_serial_step(vec![(0, VoxelPhysicsState::Empty); 4096], 0)
                });
                let old = chunk.get_block(local_x as i32, local_y as i32, local_z as i32).0 as usize;
                let old_cb = collider_map.requires_java_callback(old);
                let new_cb = collider_map.requires_java_callback(block.0 as usize);
                if old_cb != new_cb {
                    if new_cb {
                        if chunk.serial_callback_blocks == 0 {
                            *sublevel_serial_callback_sections += 1;
                        }
                        chunk.serial_callback_blocks += 1;
                    } else {
                        chunk.serial_callback_blocks -= 1;
                        if chunk.serial_callback_blocks == 0 {
                            *sublevel_serial_callback_sections =
                                sublevel_serial_callback_sections.saturating_sub(1);
                        }
                    }
                }
                chunk.set_block(local_x as i32, local_y as i32, local_z as i32, block);
                if body.contains(x, y, z) {
                    body.insert_block(x, y, z, &block, true, collider_map);
                }
                body.mark_section_dirty(sec_x, sec_y, sec_z);
                update_collider_aabb(&mut sim, body);
            }
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getUniversePose<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    pose_arr: jni::sys::jdoubleArray,
) {
    if universe_handle == 0 || pose_arr.is_null() { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let universe = unsafe { &*universe_ptr }.read().unwrap();
    let id_usize = id as LevelColliderID;

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        let (pos, rot) = if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let sim = scene.sim_data.read().unwrap();
                if let Some(rb) = sim.rigid_body_set.get(resident.rigid_body) {
                    let world_pos = scene.local_to_global(rb.translation());
                    (world_pos, *rb.rotation())
                } else {
                    (ubody.translation, ubody.rotation)
                }
            } else {
                (ubody.translation, ubody.rotation)
            }
        } else {
            (ubody.translation, ubody.rotation)
        };

        let values = [
            pos.x, pos.y, pos.z,
            rot.x as f64, rot.y as f64, rot.z as f64, rot.w as f64,
        ];
        let pose_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(pose_arr) };
        env.set_double_array_region(&pose_obj, 0, &values).unwrap();
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getUniverseLinearVelocity<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    vel_arr: jni::sys::jdoubleArray,
) {
    if universe_handle == 0 || vel_arr.is_null() { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let universe = unsafe { &*universe_ptr }.read().unwrap();
    let id_usize = id as LevelColliderID;

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        let vel = if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let sim = scene.sim_data.read().unwrap();
                if let Some(rb) = sim.rigid_body_set.get(resident.rigid_body) {
                    rb.linvel()
                } else {
                    ubody.linear_velocity
                }
            } else {
                ubody.linear_velocity
            }
        } else {
            ubody.linear_velocity
        };

        let values = [vel.x as f64, vel.y as f64, vel.z as f64];
        let vel_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(vel_arr) };
        env.set_double_array_region(&vel_obj, 0, &values).unwrap();
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_getUniverseAngularVelocity<'local>(
    mut env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    universe_handle: jni::sys::jlong,
    id: jni::sys::jint,
    vel_arr: jni::sys::jdoubleArray,
) {
    if universe_handle == 0 || vel_arr.is_null() { return; }
    let universe_ptr = universe_handle as *mut std::sync::RwLock<crate::scene::DimensionUniverse>;
    let universe = unsafe { &*universe_ptr }.read().unwrap();
    let id_usize = id as LevelColliderID;

    if let Some(ubody) = universe.universe_bodies.get(&id_usize) {
        let vel = if let Some(resident) = &ubody.resident {
            let scene_ptr = resident.scene_handle as *mut PhysicsScene;
            if !scene_ptr.is_null() {
                let scene = unsafe { &*scene_ptr };
                let sim = scene.sim_data.read().unwrap();
                if let Some(rb) = sim.rigid_body_set.get(resident.rigid_body) {
                    rb.angvel()
                } else {
                    ubody.angular_velocity
                }
            } else {
                ubody.angular_velocity
            }
        } else {
            ubody.angular_velocity
        };

        let values = [vel.x as f64, vel.y as f64, vel.z as f64];
        let vel_obj = unsafe { jni::objects::JPrimitiveArray::from_raw(vel_arr) };
        env.set_double_array_region(&vel_obj, 0, &values).unwrap();
    }
}

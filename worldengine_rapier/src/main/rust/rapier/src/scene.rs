use crate::event_handler::SableEventHandler;
use crate::hooks::SablePhysicsHooks;
use crate::joints::SableJointSet;
use crate::rope::RopeMap;
use crate::{ActiveLevelColliderInfo, ReportedCollision};
use dashmap::DashMap;
use jni::JavaVM;
use marten::Real;
use marten::level::{ChunkSection, OctreeChunkSection};
use marten::octree::SubLevelOctree;
use rapier3d::dynamics::{
    CCDSolver, ImpulseJointSet, IslandManager, LockedAxes, MassProperties, MultibodyJointSet,
    RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{ColliderSet, DefaultBroadPhase, NarrowPhase};
use rapier3d::glamx::IVec3;
use rapier3d::math::Vec3;
use rapier3d::pipeline::PhysicsPipeline;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, RwLock};

pub type LevelColliderID = usize;

pub trait ChunkAccess {
    #[allow(unused)]
    fn get_chunk_mut(&mut self, x: i32, y: i32, z: i32) -> Option<&mut ChunkSection>;
    fn get_chunk(&self, x: i32, y: i32, z: i32) -> Option<&ChunkSection>;
}

#[inline(always)]
pub fn pack_section_pos(i: i32, j: i32, k: i32) -> i64 {
    let mut l: i64 = 0;
    l |= (i as i64 & 4194303i64) << 42;
    l |= j as i64 & 1048575i64;
    l | (k as i64 & 4194303i64) << 20
}

#[inline(always)]
pub fn is_inside_terrain_domain(x: i32, y: i32, z: i32) -> bool {
    x >= -1_875_000 && x <= 1_875_000
        && z >= -1_875_000 && z <= 1_875_000
        && y >= -128 && y <= 128
}

pub type ChunkMap = HashMap<i64, ChunkSection>;

pub struct ReportedCollisionBuffer(Mutex<Vec<ReportedCollision>>);

impl ReportedCollisionBuffer {
    pub fn new() -> Self {
        Self(Mutex::new(Vec::with_capacity(16)))
    }

    pub fn lock(&self) -> std::sync::MutexGuard<'_, Vec<ReportedCollision>> {
        self.0.lock().unwrap()
    }
}

impl Default for ReportedCollisionBuffer {
    fn default() -> Self {
        Self::new()
    }
}

pub struct SimulationSceneData {
    pub integration_parameters: rapier3d::dynamics::IntegrationParameters,
    pub pipeline: PhysicsPipeline,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub physics_hooks: SablePhysicsHooks,
    pub event_handler: SableEventHandler,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulationTier {
    Dormant,
    Ballistic,
    Active,
    Critical,
}

#[derive(Clone)]
pub struct ResidentPhysicsBody {
    pub rigid_body: RigidBodyHandle,
    pub scene_handle: i64,
}

pub const MACRO_CELL_SIZE: Real = 512.0;
const MAX_GRID_CELLS_PER_BODY: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UniverseAabb {
    pub min: DVec3,
    pub max: DVec3,
}

impl UniverseAabb {
    pub fn around(center: DVec3, half_extents: DVec3) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    pub fn swept(self, displacement: DVec3) -> Self {
        Self {
            min: DVec3::new(
                self.min.x.min(self.min.x + displacement.x),
                self.min.y.min(self.min.y + displacement.y),
                self.min.z.min(self.min.z + displacement.z),
            ),
            max: DVec3::new(
                self.max.x.max(self.max.x + displacement.x),
                self.max.y.max(self.max.y + displacement.y),
                self.max.z.max(self.max.z + displacement.z),
            ),
        }
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min.x <= other.max.x && self.min.y <= other.max.y && self.min.z <= other.max.z &&
        self.max.x >= other.min.x && self.max.y >= other.min.y && self.max.z >= other.min.z
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MacroCell(pub i32, pub i32, pub i32);

#[derive(Default)]
pub struct WorldSpatialIndex {
    cells: HashMap<MacroCell, HashSet<LevelColliderID>>,
    body_cells: HashMap<LevelColliderID, Vec<MacroCell>>,
    bounds: HashMap<LevelColliderID, UniverseAabb>,
    large_bodies: HashSet<LevelColliderID>,
}

impl WorldSpatialIndex {
    fn cell_range(bounds: UniverseAabb) -> (MacroCell, MacroCell) {
        let min = (bounds.min / (MACRO_CELL_SIZE as f64)).map(|x| x.floor() as i32);
        let max = (bounds.max / (MACRO_CELL_SIZE as f64)).map(|x| x.floor() as i32);
        (
            MacroCell(min.x, min.y, min.z),
            MacroCell(max.x, max.y, max.z),
        )
    }

    pub fn update(&mut self, id: LevelColliderID, bounds: UniverseAabb) {
        self.remove(id);
        self.bounds.insert(id, bounds);
        let (min, max) = Self::cell_range(bounds);
        let nx = (max.0 as i64 - min.0 as i64 + 1).max(0) as usize;
        let ny = (max.1 as i64 - min.1 as i64 + 1).max(0) as usize;
        let nz = (max.2 as i64 - min.2 as i64 + 1).max(0) as usize;
        if nx.saturating_mul(ny).saturating_mul(nz) > MAX_GRID_CELLS_PER_BODY {
            self.large_bodies.insert(id);
            return;
        }

        let mut occupied = Vec::with_capacity(nx * ny * nz);
        for x in min.0..=max.0 {
            for y in min.1..=max.1 {
                for z in min.2..=max.2 {
                    let cell = MacroCell(x, y, z);
                    self.cells.entry(cell).or_default().insert(id);
                    occupied.push(cell);
                }
            }
        }
        self.body_cells.insert(id, occupied);
    }

    pub fn remove(&mut self, id: LevelColliderID) {
        if let Some(cells) = self.body_cells.remove(&id) {
            for cell in cells {
                let remove_cell = if let Some(ids) = self.cells.get_mut(&cell) {
                    ids.remove(&id);
                    ids.is_empty()
                } else {
                    false
                };
                if remove_cell {
                    self.cells.remove(&cell);
                }
            }
        }
        self.large_bodies.remove(&id);
        self.bounds.remove(&id);
    }

    pub fn query(&self, query: UniverseAabb, except: LevelColliderID) -> Vec<LevelColliderID> {
        let (min, max) = Self::cell_range(query);
        let mut candidates = HashSet::new();
        for x in min.0..=max.0 {
            for y in min.1..=max.1 {
                for z in min.2..=max.2 {
                    if let Some(ids) = self.cells.get(&MacroCell(x, y, z)) {
                        candidates.extend(ids.iter().copied());
                    }
                }
            }
        }
        candidates.extend(self.large_bodies.iter().copied());
        candidates.remove(&except);
        candidates
            .into_iter()
            .filter(|id| {
                self.bounds
                    .get(id)
                    .is_some_and(|bounds| bounds.intersects(&query))
            })
            .collect()
    }

    #[cfg(test)]
    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ScheduledBody {
    pub tick: u64,
    pub id: LevelColliderID,
    pub generation: u64,
}

#[derive(Clone)]
pub struct BodyDynamics {
    pub additional_mass_properties: Option<MassProperties>,
    pub linear_damping: Real,
    pub angular_damping: Real,
    pub gravity_scale: Real,
    pub locked_axes: LockedAxes,
    pub ccd_enabled: bool,
}

pub type DVec3 = rapier3d::na::Vector3<f64>;

pub enum UniverseCommand {
    ApplyForce { x: f64, y: f64, z: f64, fx: f64, fy: f64, fz: f64, wake_up: bool },
    ApplyForceAndTorque { fx: f64, fy: f64, fz: f64, tx: f64, ty: f64, tz: f64, wake_up: bool },
    AddLinearAngularVelocities { linear_x: f64, linear_y: f64, linear_z: f64, angular_x: f64, angular_y: f64, angular_z: f64, wake_up: bool },
    SetKinematicContraptionTransform { center_of_mass: [f64; 3], pose: [f64; 7], velocities: [f64; 6] },
    WakeUp,
}

pub struct UniverseBody {
    pub id: LevelColliderID,
    pub translation: DVec3,
    pub rotation: rapier3d::math::Rotation,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub dynamics: BodyDynamics,
    pub simulation_tier: SimulationTier,
    pub bounds: UniverseAabb,
    pub last_update_tick: u64,
    pub next_update_tick: u64,
    pub schedule_generation: u64,
    pub resident: Option<ResidentPhysicsBody>,
    /// Stable minimum body id for the fixed-joint assembly containing this body.
    pub assembly_root: LevelColliderID,
    pub assembly_size: u32,
    pub command_queue: Vec<UniverseCommand>,
}

pub fn compute_universe_body_aabb(
    translation: DVec3,
    rotation: rapier3d::math::Rotation,
    min_ivec: IVec3,
    max_ivec: IVec3,
    com: rapier3d::glamx::DVec3,
) -> UniverseAabb {
    let local_min = min_ivec.as_dvec3();
    let local_max = max_ivec.as_dvec3() + rapier3d::glamx::DVec3::ONE;
    let local_half_extents = (local_max - local_min) * 0.5;
    let local_center = (local_min + local_max) * 0.5 - com;

    let rot_mat = rapier3d::glamx::Mat3::from_quat(rotation);
    let rotated_local_center = rot_mat.mul_vec3(local_center.as_vec3()).as_dvec3();

    let abs_mat = rapier3d::glamx::Mat3::from_cols(
        rot_mat.x_axis.abs(),
        rot_mat.y_axis.abs(),
        rot_mat.z_axis.abs(),
    );
    let glam_half_extents = abs_mat.mul_vec3(local_half_extents.as_vec3()).as_dvec3();
    let world_half_extents = DVec3::new(glam_half_extents.x, glam_half_extents.y, glam_half_extents.z);
    let world_center = translation + DVec3::new(rotated_local_center.x, rotated_local_center.y, rotated_local_center.z);

    UniverseAabb::around(world_center, world_half_extents)
}

impl UniverseBody {
    pub fn update_bounds(&mut self, geom: &PersistentBodyGeometry) {
        if let (Some(min), Some(max)) = (geom.local_bounds_min, geom.local_bounds_max) {
            let com = geom.center_of_mass.unwrap_or(rapier3d::glamx::DVec3::ZERO);
            self.bounds = compute_universe_body_aabb(self.translation, self.rotation, min, max, com);
        } else {
            self.bounds = UniverseAabb::around(self.translation, DVec3::new(0.5, 0.5, 0.5));
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaterializationRequest {
    pub id: LevelColliderID,
    pub position: DVec3,
}

#[derive(Debug, Clone, Default)]
pub struct PersistentBodyGeometry {
    pub local_bounds_min: Option<IVec3>,
    pub local_bounds_max: Option<IVec3>,
    pub octree_origin: Option<IVec3>,
    pub center_of_mass: Option<rapier3d::glamx::DVec3>,
    pub octree: Option<SubLevelOctree>,
    pub section_octrees: HashMap<i64, SubLevelOctree>,
    pub chunk_map: Option<ChunkMap>,
    pub geometry_version: u64,
    pub dirty_sections: std::collections::HashSet<i64>,
}

impl PersistentBodyGeometry {
    pub fn mark_section_dirty(&mut self, x: i32, y: i32, z: i32) {
        self.geometry_version = self.geometry_version.wrapping_add(1);
        self.dirty_sections.insert(pack_section_pos(x, y, z));
    }
}

#[derive(Default)]
pub struct DimensionUniverse {
    pub universe_bodies: std::collections::HashMap<LevelColliderID, UniverseBody>,
    pub body_geometries: std::collections::HashMap<LevelColliderID, PersistentBodyGeometry>,
    pub spatial_index: WorldSpatialIndex,
    pub scheduled_bodies: std::collections::BinaryHeap<std::cmp::Reverse<ScheduledBody>>,
    pub pose_dirty_bodies: std::collections::HashSet<LevelColliderID>,
    pub pending_command_bodies: std::collections::HashSet<LevelColliderID>,
    pub materialization_requests: Vec<MaterializationRequest>,
    pub pending_materializations: std::collections::HashSet<LevelColliderID>,
    pub eviction_events: Vec<LevelColliderID>,
    pub pending_evictions: std::collections::HashSet<LevelColliderID>,
    pub terrain_sections: std::collections::HashSet<i64>,
    pub current_tick: u64,
}

pub struct SableSceneData {
    pub scene_handle: i64,
    pub main_level_chunks: ChunkMap,
    pub octree_chunks: std::collections::HashMap<i64, OctreeChunkSection>,
    pub joint_set: SableJointSet,
    pub rope_map: RopeMap,
    pub level_colliders: std::collections::HashMap<LevelColliderID, ActiveLevelColliderInfo>,
    pub rigid_bodies: std::collections::HashMap<LevelColliderID, RigidBodyHandle>,
    pub terrain_serial_callback_sections: u32,
    pub sublevel_serial_callback_sections: u32,
}

impl SableSceneData {
    pub fn can_parallel_step(&self) -> bool {
        self.terrain_serial_callback_sections == 0 && self.sublevel_serial_callback_sections == 0
    }
}

impl DimensionUniverse {
    pub fn next_due_tick(&mut self) -> Option<u64> {
        loop {
            let Reverse(scheduled) = self.scheduled_bodies.peek().copied()?;
            if self.universe_bodies.get(&scheduled.id).is_some_and(|body| {
                body.schedule_generation == scheduled.generation
                    && body.next_update_tick == scheduled.tick
            }) {
                return Some(scheduled.tick);
            }
            self.scheduled_bodies.pop();
        }
    }
}

/// A physics scene
pub struct PhysicsScene {
    pub sim_data: RwLock<SimulationSceneData>,
    pub sable_data: Arc<RwLock<SableSceneData>>,
    pub universe: Arc<RwLock<DimensionUniverse>>,

    /// All collisions substantial enough to be considered for collision events.
    pub reported_collisions: Arc<ReportedCollisionBuffer>,

    pub manifold_info_map: Arc<SableManifoldInfoMap>,

    pub current_step_vm: Option<Arc<JavaVM>>,

    /// The handle to a static rigidbody
    pub ground_handle: Option<RigidBodyHandle>,

    /// The current gravity vector for all bodies. [m/s^2]
    pub gravity: Vec3,

    /// Global block-space origin represented by local Rapier coordinate zero.
    pub world_origin: RwLock<DVec3>,

    /// Universal linear drag applied to all bodies
    pub universal_drag: Real,
}

impl PhysicsScene {
    #[inline]
    pub fn global_to_local(&self, position: DVec3) -> Vec3 {
        let origin = *self.world_origin.read().unwrap();
        let delta = position - origin;
        Vec3::new(delta.x as f32, delta.y as f32, delta.z as f32)
    }

    #[inline]
    pub fn local_to_global(&self, position: Vec3) -> DVec3 {
        let origin = *self.world_origin.read().unwrap();
        let dpos = DVec3::new(position.x as f64, position.y as f64, position.z as f64);
        dpos + origin
    }

    #[inline]
    pub fn origin_section(&self) -> IVec3 {
        let origin = self.world_origin.read().unwrap();
        IVec3::new(origin.x as i32, origin.y as i32, origin.z as i32) >> 4
    }
}

#[derive(Default)]
pub struct SableManifoldInfoMap {
    pub list: DashMap<usize, SableManifoldInfo>,
    pub counter: AtomicUsize,
}

impl SableManifoldInfoMap {
    pub fn clear(&self) {
        self.list.clear();
        self.counter.store(0, Ordering::Relaxed);
    }
}

pub struct SableManifoldInfo {
    pub pos_a: IVec3,
    pub pos_b: IVec3,
    pub col_a: usize,
    pub col_b: usize,
}

impl ChunkAccess for SableSceneData {
    fn get_chunk_mut(&mut self, x: i32, y: i32, z: i32) -> Option<&mut ChunkSection> {
        self.main_level_chunks.get_mut(&pack_section_pos(x, y, z))
    }

    fn get_chunk(&self, x: i32, y: i32, z: i32) -> Option<&ChunkSection> {
        self.main_level_chunks.get(&pack_section_pos(x, y, z))
    }
}

impl DimensionUniverse {
    pub fn schedule_body(&mut self, id: LevelColliderID, tick: u64) {
        let Some(body) = self.universe_bodies.get_mut(&id) else {
            return;
        };
        body.schedule_generation = body.schedule_generation.wrapping_add(1);
        body.next_update_tick = tick;
        self.scheduled_bodies.push(Reverse(ScheduledBody {
            tick,
            id,
            generation: body.schedule_generation,
        }));
    }

    pub fn pop_due_body(&mut self) -> Option<LevelColliderID> {
        while let Some(Reverse(scheduled)) = self.scheduled_bodies.peek().copied() {
            if scheduled.tick > self.current_tick {
                return None;
            }
            self.scheduled_bodies.pop();
            if self.universe_bodies.get(&scheduled.id).is_some_and(|body| {
                body.schedule_generation == scheduled.generation
                    && body.next_update_tick == scheduled.tick
            }) {
                return Some(scheduled.id);
            }
        }
        None
    }

    pub fn ticks_until_next_scheduled_body(&mut self) -> Option<u64> {
        loop {
            let Reverse(scheduled) = self.scheduled_bodies.peek().copied()?;
            if self.universe_bodies.get(&scheduled.id).is_some_and(|body| {
                body.schedule_generation == scheduled.generation
                    && body.next_update_tick == scheduled.tick
            }) {
                return Some(scheduled.tick.saturating_sub(self.current_tick));
            }
            self.scheduled_bodies.pop();
        }
    }

    pub fn request_materialization(&mut self, id: LevelColliderID) {
        if self.pending_materializations.insert(id) {
            if let Some(body) = self.universe_bodies.get(&id) {
                self.materialization_requests.push(MaterializationRequest {
                    id,
                    position: body.translation,
                });
            }
        }
    }

    pub fn record_eviction(&mut self, id: LevelColliderID) {
        if self.pending_evictions.insert(id) {
            self.eviction_events.push(id);
        }
    }

    pub fn swept_intersects_terrain(&self, bounds: UniverseAabb) -> bool {
        if self.terrain_sections.is_empty() {
            return false;
        }
        if bounds.min.x > 30_000_000.0 || bounds.max.x < -30_000_000.0
            || bounds.min.z > 30_000_000.0 || bounds.max.z < -30_000_000.0
            || bounds.min.y > 2048.0 || bounds.max.y < -2048.0
        {
            return false;
        }
        let min_x = (bounds.min.x.floor() as i32) >> 4;
        let min_y = (bounds.min.y.floor() as i32) >> 4;
        let min_z = (bounds.min.z.floor() as i32) >> 4;
        let max_x = (bounds.max.x.ceil() as i32) >> 4;
        let max_y = (bounds.max.y.ceil() as i32) >> 4;
        let max_z = (bounds.max.z.ceil() as i32) >> 4;
        let count = (max_x as i64 - min_x as i64 + 1)
            .saturating_mul(max_y as i64 - min_y as i64 + 1)
            .saturating_mul(max_z as i64 - min_z as i64 + 1);
        if count > 4096 {
            return true;
        }
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                for z in min_z..=max_z {
                    if is_inside_terrain_domain(x, y, z) && self.terrain_sections.contains(&pack_section_pos(x, y, z)) {
                        return true;
                    }
                }
            }
        }
        false
    }
}

impl SableSceneData {
    pub fn get_octree_chunk(&self, x: i32, y: i32, z: i32) -> Option<&OctreeChunkSection> {
        self.octree_chunks.get(&pack_section_pos(x, y, z))
    }

    pub fn get_octree_chunk_mut(
        &mut self,
        x: i32,
        y: i32,
        z: i32,
    ) -> Option<&mut OctreeChunkSection> {
        self.octree_chunks.get_mut(&pack_section_pos(x, y, z))
    }
}

#[cfg(test)]
mod spatial_tests {
    use super::*;

    #[test]
    fn sparse_grid_returns_only_intersecting_candidates() {
        let mut index = WorldSpatialIndex::default();
        index.update(1, UniverseAabb::around(DVec3::zeros(), DVec3::new(2.0, 2.0, 2.0)));
        index.update(
            2,
            UniverseAabb::around(DVec3::new((MACRO_CELL_SIZE - 1.0) as f64, 0.0, 0.0), DVec3::new(4.0, 4.0, 4.0)),
        );
        for id in 3..10_003 {
            index.update(
                id,
                UniverseAabb::around(DVec3::new((id as Real * 1024.0) as f64, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0)),
            );
        }

        assert_eq!(
            index.query(UniverseAabb::around(DVec3::zeros(), DVec3::new(3.0, 3.0, 3.0)), 1),
            Vec::<LevelColliderID>::new()
        );
        assert_eq!(
            index.query(
                UniverseAabb::around(DVec3::new(MACRO_CELL_SIZE as f64, 0.0, 0.0), DVec3::new(8.0, 8.0, 8.0)),
                1,
            ),
            vec![2]
        );
        assert!(index.cell_count() >= 10_000);
    }

    #[test]
    fn oversized_body_uses_large_object_fallback() {
        let mut index = WorldSpatialIndex::default();
        index.update(
            7,
            UniverseAabb::around(DVec3::zeros(), DVec3::new((MACRO_CELL_SIZE * 20.0) as f64, (MACRO_CELL_SIZE * 20.0) as f64, (MACRO_CELL_SIZE * 20.0) as f64)),
        );
        assert_eq!(index.cell_count(), 0);
        assert_eq!(
            index.query(UniverseAabb::around(DVec3::new(10.0, 10.0, 10.0), DVec3::new(1.0, 1.0, 1.0)), 99),
            vec![7]
        );
    }

    #[test]
    fn body_aabb_sweep_is_conservative() {
        let bounds = UniverseAabb::around(DVec3::zeros(), DVec3::new(1.0, 1.0, 1.0));
        let swept = bounds.swept(DVec3::new(10.0, -4.0, 2.0));
        assert_eq!(swept.min, DVec3::new(-1.0, -5.0, -1.0));
        assert_eq!(swept.max, DVec3::new(11.0, 1.0, 3.0));
    }
}
use crate::collider::LevelCollider;
use log::info;
use rapier3d::geometry::{ContactManifoldData, Shape};
use rapier3d::glamx::{DVec3, IVec3, Pose3};
use rapier3d::math::Vec3;
use rapier3d::parry::query::details::{NormalConstraints, contact_manifold_cuboid_cuboid_shapes};
use rapier3d::parry::query::{
    ClosestPoints, Contact, ContactManifold, ContactManifoldsWorkspace, DefaultQueryDispatcher,
    NonlinearRigidMotion, PersistentQueryDispatcher, QueryDispatcher, ShapeCastHit,
    ShapeCastOptions, TypedWorkspaceData, Unsupported, WorkspaceData,
};
use rapier3d::prelude::ShapeType::Custom;
use rapier3d::prelude::{Aabb, Real};

use crate::algo::find_collision_pairs;
use crate::scene::{
    ChunkAccess, LevelColliderID, SableManifoldInfo, SableManifoldInfoMap, SableSceneData,
};
use crate::{ActiveLevelColliderInfo, PhysicsState};
use marten::level::VoxelPhysicsState::{Edge, Face, Interior};
use marten::level::{NEEDS_HOOKS_USER_DATA, VoxelPhysicsState};
use std::sync::atomic::Ordering;
use std::sync::{Arc, RwLock};

/// The distance we scale collision points local to the box collider for before we check interior collisions
/// This helps avoid a missed interior collision when points are slightly outside of their voxel on an axis
/// not aligned with their normal. Example: A horizontal interior collision with a point 0.5001 above the
/// block center.
const INTERIOR_COLLISION_SCALE_FACTOR: Real = 0.99;

/// The distance at which we offset the normal of a collision point to check if it is inside a voxel collider
/// to rule it as an interior collision
const INTERIOR_COLLISION_CHECK_DISTANCE: f64 = 0.015;

#[derive(Clone)]
pub struct SableDispatcher {
    pub sable_data: Arc<RwLock<SableSceneData>>,
    pub manifold_info_map: Arc<SableManifoldInfoMap>,
}

// The index alone is not a contact identity: terrain edits and pair enumeration
// can put a different voxel/box at the same manifold index on the next step.
#[derive(Clone, PartialEq, Eq)]
struct VoxelManifoldKey {
    position_a: IVec3,
    position_b: IVec3,
    collider_a: u32,
    collider_b: u32,
    state_a: VoxelPhysicsState,
    state_b: VoxelPhysicsState,
    box_a: [u32; 6],
    box_b: [u32; 6],
    swapped: bool,
}

#[derive(Clone, Default)]
struct VoxelManifoldWorkspace {
    previous: Vec<VoxelManifoldKey>,
    next: Vec<VoxelManifoldKey>,
    merged_geometry_key: Option<(u64, IVec3, IVec3)>,
    merged_box: Option<MergedVoxelBox>,
}

/// Exact union of a bounded, homogeneous rectangular collection of unit cubes.
/// This does not approximate a concave body, merge materials, or reduce solver quality.
#[derive(Clone, Copy, Debug)]
struct MergedVoxelBox {
    min: IVec3,
    extents: IVec3,
    block_id: u32,
}

impl MergedVoxelBox {
    fn contains_voxel(&self, position: IVec3) -> bool {
        let offset = position.as_i64vec3() - self.min.as_i64vec3();
        offset.cmpge(rapier3d::glamx::I64Vec3::ZERO).all()
            && offset.cmplt(self.extents.as_i64vec3()).all()
    }
}

fn is_plain_unit_cube(data: &marten::level::VoxelColliderData) -> bool {
    !data.dynamic
        && !data.is_fluid
        && data.contact_method.is_none()
        && data.get_user_data() == 0
        && data.collision_boxes.as_slice() == [(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)]
}

fn merge_unit_voxels(
    min: IVec3,
    max: IVec3,
    mut eligible_block: impl FnMut(IVec3) -> Option<Option<u32>>,
) -> Option<MergedVoxelBox> {
    let size = max.as_i64vec3() - min.as_i64vec3() + rapier3d::glamx::I64Vec3::ONE;
    if size.min_element() <= 0
        || size.max_element() > 4096
        || size.x.checked_mul(size.y)?.checked_mul(size.z)? > 4096
    {
        return None;
    }
    let mut material = None;
    let mut occupied_min = IVec3::splat(i32::MAX);
    let mut occupied_max = IVec3::splat(i32::MIN);
    let mut occupied = 0_i64;
    for x in min.x..=max.x {
        for y in min.y..=max.y {
            for z in min.z..=max.z {
                let pos = IVec3::new(x, y, z);
                // Outer plot padding can be air. Invalid/partial/special voxels reject
                // the entire merge; they must never be treated as absent geometry.
                let Some(id) = eligible_block(pos)? else {
                    continue;
                };
                if id == 0 || material.is_some_and(|previous| previous != id) {
                    return None;
                }
                material = Some(id);
                occupied_min = occupied_min.min(pos);
                occupied_max = occupied_max.max(pos);
                occupied += 1;
            }
        }
    }
    let block_id = material?;
    let extents =
        occupied_max.as_i64vec3() - occupied_min.as_i64vec3() + rapier3d::glamx::I64Vec3::ONE;
    if extents.x * extents.y * extents.z != occupied {
        return None;
    }
    Some(MergedVoxelBox {
        min: occupied_min,
        extents: extents.as_ivec3(),
        block_id,
    })
}

// Each output is the exact union of one complete material/layer group in the
// current conservative terrain candidates. Irregular groups remain individual
// voxels; in particular this cannot fill a terrain hole or merge materials.
fn merge_terrain_layers(positions: &[(IVec3, u32)]) -> Vec<MergedVoxelBox> {
    let mut groups = std::collections::BTreeMap::<(i32, u32), std::collections::HashSet<IVec3>>::new();
    for &(pos, id) in positions {
        groups.entry((pos.y, id)).or_default().insert(pos);
    }
    let mut merged = Vec::new();
    for ((_, id), cells) in groups {
        if cells.len() < 2 { continue; }
        let min = cells.iter().copied().reduce(IVec3::min).unwrap();
        let max = cells.iter().copied().reduce(IVec3::max).unwrap();
        if let Some(cuboid) = merge_unit_voxels(min, max, |pos| Some(cells.contains(&pos).then_some(id))) {
            merged.push(cuboid);
        }
    }
    merged
}

impl VoxelManifoldWorkspace {
    fn merged_box_for(
        &mut self,
        version: u64,
        min: IVec3,
        max: IVec3,
        eligible_block: impl FnMut(IVec3) -> Option<Option<u32>>,
    ) -> Option<MergedVoxelBox> {
        let key = (version, min, max);
        if self.merged_geometry_key != Some(key) {
            self.merged_geometry_key = Some(key);
            self.merged_box = merge_unit_voxels(min, max, eligible_block);
            #[cfg(feature = "benchmark-profiler")]
            if std::env::var("WE_NATIVE_PROFILE").as_deref() == Ok("true") {
                eprintln!(
                    "WE_CUBOID_CACHE version={} source_min={:?} source_max={:?} merged={:?}",
                    version, min, max, self.merged_box
                );
            }
        }
        self.merged_box
    }
}

impl WorkspaceData for VoxelManifoldWorkspace {
    fn as_typed_workspace_data(&self) -> TypedWorkspaceData<'_> {
        TypedWorkspaceData::Custom
    }
    fn clone_dyn(&self) -> Box<dyn WorkspaceData> {
        Box::new(self.clone())
    }
}

fn voxel_workspace(
    workspace: &mut Option<ContactManifoldsWorkspace>,
) -> &mut VoxelManifoldWorkspace {
    if !workspace
        .as_ref()
        .is_some_and(|entry| entry.0.is::<VoxelManifoldWorkspace>())
    {
        *workspace = Some(VoxelManifoldWorkspace::default().into());
    }
    workspace
        .as_mut()
        .unwrap()
        .0
        .downcast_mut::<VoxelManifoldWorkspace>()
        .unwrap()
}

fn match_voxel_contacts<ContactData: Default + Copy>(
    key: &VoxelManifoldKey,
    previous_key: Option<&VoxelManifoldKey>,
    old: &ContactManifold<ContactManifoldData, ContactData>,
    fresh: &mut ContactManifold<ContactManifoldData, ContactData>,
) {
    if previous_key == Some(key) {
        // Parry matches feature ids and transfers only tracked point data.
        // Fresh geometry, interior filtering and manifold/hook data stay fresh.
        fresh.match_contacts(&old.points);
    }
}

#[cfg(test)]
mod tracking_tests {
    use super::*;

    #[test]
    fn terrain_rectangles_cover_exactly_the_input_layers_at_large_coordinates() {
        for origin in [IVec3::ZERO, IVec3::splat(-50), IVec3::splat(28_000_000)] {
            let mut cells = Vec::new();
            for y in 0..2 { for x in 0..3 { for z in 0..2 {
                cells.push((origin + IVec3::new(x, y, z), 9 + y as u32));
            }}}
            let boxes = merge_terrain_layers(&cells);
            assert_eq!(boxes.len(), 2);
            for x in -1..=3 { for y in -1..=2 { for z in -1..=2 {
                let pos = origin + IVec3::new(x, y, z);
                let expected = cells.iter().any(|(cell, _)| *cell == pos);
                let actual = boxes.iter().any(|box_| {
                    box_.contains_voxel(pos)
                });
                assert_eq!(actual, expected, "Terrain union differs at {pos:?}");
            }}}
        }
    }

    #[test]
    fn incomplete_or_mixed_terrain_groups_cannot_fill_holes_or_merge_materials() {
        let complete: Vec<_> = (0..3).flat_map(|x| (0..3).map(move |z| (IVec3::new(x, 0, z), 9))).collect();
        let hole: Vec<_> = complete.iter().copied().filter(|(pos, _)| *pos != IVec3::new(1, 0, 1)).collect();
        assert!(merge_terrain_layers(&hole).is_empty());
        let mixed: Vec<_> = complete.iter().map(|(pos, id)| (*pos, if *pos == IVec3::new(1, 0, 1) { 10 } else { *id })).collect();
        assert!(merge_terrain_layers(&mixed).is_empty());
        let mut duplicates = complete.clone();
        duplicates.extend_from_slice(&complete);
        assert_eq!(merge_terrain_layers(&duplicates).len(), 1);
        let huge = [(IVec3::ZERO, 9), (IVec3::new(4096, 0, 0), 9)];
        assert!(merge_terrain_layers(&huge).is_empty());
        let edge = [(IVec3::new(i32::MAX - 1, 0, 0), 9), (IVec3::new(i32::MAX, 0, 0), 9)];
        let rectangle = merge_terrain_layers(&edge).pop().unwrap();
        assert!(rectangle.contains_voxel(edge[0].0));
        assert!(rectangle.contains_voxel(edge[1].0));
        assert!(!rectangle.contains_voxel(IVec3::new(i32::MAX - 2, 0, 0)));
    }

    fn key() -> VoxelManifoldKey {
        VoxelManifoldKey {
            position_a: IVec3::ZERO,
            position_b: IVec3::Y,
            collider_a: 1,
            collider_b: 2,
            state_a: Face,
            state_b: Face,
            box_a: [
                0,
                0,
                0,
                1.0_f32.to_bits(),
                1.0_f32.to_bits(),
                1.0_f32.to_bits(),
            ],
            box_b: [
                0,
                0,
                0,
                1.0_f32.to_bits(),
                1.0_f32.to_bits(),
                1.0_f32.to_bits(),
            ],
            swapped: false,
        }
    }

    fn contacts() -> ContactManifold<ContactManifoldData, u32> {
        let mut manifold = ContactManifold::new();
        let pose = Pose3 {
            translation: Vec3::Y,
            rotation: rapier3d::glamx::Quat::IDENTITY,
        };
        let cube = rapier3d::parry::shape::Cuboid::new(Vec3::splat(0.5));
        contact_manifold_cuboid_cuboid_shapes(&pose, &cube, &cube, 0.01, &mut manifold);
        assert!(!manifold.points.is_empty());
        manifold
    }

    #[test]
    fn merged_shape_cache_invalidates_after_holes_material_or_bounds_changes() {
        let mut workspace = VoxelManifoldWorkspace::default();
        let min = IVec3::ZERO;
        let max = IVec3::new(1, 0, 1);
        assert!(
            workspace
                .merged_box_for(1, min, max, |_| Some(Some(9)))
                .is_some()
        );
        // The same geometry must not rescan its owned chunks each substep.
        assert!(
            workspace
                .merged_box_for(1, min, max, |_| panic!("Unchanged cache rescanned"))
                .is_some()
        );
        assert!(
            workspace
                .merged_box_for(2, min, max, |pos| Some((pos != max).then_some(9)))
                .is_none()
        );
        assert!(
            workspace
                .merged_box_for(3, min, max, |_| Some(Some(9)))
                .is_some()
        );
        assert!(
            workspace
                .merged_box_for(4, min, max, |pos| Some(Some(if pos == max {
                    10
                } else {
                    9
                })))
                .is_none()
        );
        assert!(
            workspace
                .merged_box_for(5, min, max, |_| Some(Some(9)))
                .is_some()
        );
        assert!(
            workspace
                .merged_box_for(5, min, max + IVec3::X, |pos| Some(
                    (pos != IVec3::X).then_some(9)
                ))
                .is_none()
        );
    }

    #[test]
    fn partial_shapes_and_special_materials_are_not_mergeable() {
        let mut data = marten::level::VoxelColliderData {
            collision_boxes: vec![(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)],
            is_fluid: false,
            friction: 1.0,
            volume: 1.0,
            restitution: 0.0,
            contact_events: None,
            contact_method: None,
            dynamic: false,
        };
        assert!(is_plain_unit_cube(&data));
        data.collision_boxes[0].4 = 0.5;
        assert!(!is_plain_unit_cube(&data));
        data.collision_boxes[0].4 = 1.0;
        data.collision_boxes.push(data.collision_boxes[0]);
        assert!(!is_plain_unit_cube(&data));
        data.collision_boxes.pop();
        data.friction = 0.2;
        assert!(!is_plain_unit_cube(&data));
        data.friction = 1.0;
        data.restitution = 0.5;
        assert!(!is_plain_unit_cube(&data));
        data.restitution = 0.0;
        data.is_fluid = true;
        assert!(!is_plain_unit_cube(&data));
        data.is_fluid = false;
        data.dynamic = true;
        assert!(!is_plain_unit_cube(&data));
    }

    #[test]
    fn outer_air_padding_is_trimmed_but_partial_shapes_and_inner_holes_are_not() {
        let min = IVec3::ZERO;
        let max = IVec3::splat(2);
        let cube = merge_unit_voxels(min, max, |pos| {
            Some((pos.y == 1 && pos.x < 2 && pos.z < 2).then_some(9))
        })
        .unwrap();
        assert_eq!(cube.min, IVec3::Y);
        assert_eq!(cube.extents, IVec3::new(2, 1, 2));
        assert!(
            merge_unit_voxels(min, max, |pos| if pos == IVec3::ONE {
                None
            } else {
                Some(Some(9))
            })
            .is_none()
        );
        assert!(
            merge_unit_voxels(min, max, |pos| Some((pos != IVec3::ONE).then_some(9))).is_none()
        );
    }

    #[test]
    fn only_complete_homogeneous_unit_cube_unions_can_merge() {
        for origin in [
            IVec3::ZERO,
            IVec3::new(-50, -60, -70),
            IVec3::splat(28_000_000),
        ] {
            let max = origin + IVec3::new(1, 0, 1);
            let merged = merge_unit_voxels(origin, max, |_| Some(Some(9))).unwrap();
            assert_eq!(merged.min, origin);
            assert_eq!(merged.extents, IVec3::new(2, 1, 2));
            let center = origin.as_dvec3() + merged.extents.as_dvec3() * 0.5;
            let half = merged.extents.as_dvec3() * 0.5;
            // Every source cell lies in the merged box and every merged cell
            // belongs to the source union: no filled hole or expanded volume.
            for x in -1..=2 {
                for y in -1..=1 {
                    for z in -1..=2 {
                        let point = (origin + IVec3::new(x, y, z)).as_dvec3() + DVec3::splat(0.5);
                        let inside = (point - center).abs().cmplt(half).all();
                        assert_eq!(
                            inside,
                            (0..=1).contains(&x) && y == 0 && (0..=1).contains(&z)
                        );
                    }
                }
            }
            assert!(
                merge_unit_voxels(origin, max, |pos| Some((pos != max).then_some(9))).is_none()
            );
            assert!(
                merge_unit_voxels(origin, max, |pos| Some(Some(if pos == max {
                    10
                } else {
                    9
                })))
                .is_none()
            );
        }
        assert!(
            merge_unit_voxels(IVec3::ZERO, IVec3::new(4096, 0, 0), |_| Some(Some(9))).is_none()
        );
        assert!(merge_unit_voxels(IVec3::ZERO, -IVec3::ONE, |_| Some(Some(9))).is_none());
    }

    #[test]
    fn unchanged_voxel_pair_keeps_tracked_data_and_fresh_geometry() {
        let mut old = contacts();
        for point in &mut old.points {
            point.data = 37;
            point.local_p1 += Vec3::splat(100.0);
        }
        let mut fresh = contacts();
        let positions: Vec<_> = fresh
            .points
            .iter()
            .map(|point| (point.local_p1, point.local_p2))
            .collect();
        match_voxel_contacts(&key(), Some(&key()), &old, &mut fresh);
        assert!(fresh.points.iter().all(|point| point.data == 37));
        assert_eq!(
            positions,
            fresh
                .points
                .iter()
                .map(|point| (point.local_p1, point.local_p2))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn reordered_voxel_or_changed_geometry_cannot_inherit_impulses() {
        let mut old = contacts();
        for point in &mut old.points {
            point.data = 37;
        }
        let original = key();
        let mut changed = original.clone();
        changed.position_b.x += 1;
        let mut changed_box = original.clone();
        changed_box.box_b[3] = 0.5_f32.to_bits();
        let mut changed_state = original.clone();
        changed_state.state_a = Interior;
        let mut changed_collider = original.clone();
        changed_collider.collider_b += 1;
        let mut swapped = original.clone();
        swapped.swapped = true;
        for previous in [
            changed,
            changed_box,
            changed_state,
            changed_collider,
            swapped,
        ] {
            let mut fresh = contacts();
            match_voxel_contacts(&original, Some(&previous), &old, &mut fresh);
            assert!(fresh.points.iter().all(|point| point.data == 0));
        }
        let mut fresh = contacts();
        match_voxel_contacts(&original, None, &old, &mut fresh);
        assert!(fresh.points.iter().all(|point| point.data == 0));
    }
}

impl SableDispatcher {
    /// Computes the local, inclusive block bounds of a global aabb with inflation
    #[allow(clippy::cast_possible_truncation)]
    #[allow(unused)]
    fn get_local_block_bounds(mut local_aabb: Aabb, inflation: Real) -> (IVec3, IVec3) {
        // Inflate the aabb by the prediction distance
        local_aabb.maxs += Vec3::splat(inflation);
        local_aabb.mins -= Vec3::splat(inflation);

        let local_min = IVec3::new(
            local_aabb.mins.x.floor() as i32,
            local_aabb.mins.y.floor() as i32,
            local_aabb.mins.z.floor() as i32,
        );

        let local_max = IVec3::new(
            local_aabb.maxs.x.floor() as i32,
            local_aabb.maxs.y.floor() as i32,
            local_aabb.maxs.z.floor() as i32,
        );

        (local_min, local_max)
    }
}

impl QueryDispatcher for SableDispatcher {
    fn intersection_test(
        &self,
        _pos12: &Pose3,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<bool, Unsupported> {
        info!("intersect {:?} <-> {:?}", g1.shape_type(), g2.shape_type());
        Err(Unsupported)
    }

    fn distance(
        &self,
        _pos12: &Pose3,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<Real, Unsupported> {
        info!("distance {:?} <-> {:?}", g1.shape_type(), g2.shape_type());
        Err(Unsupported)
    }

    fn contact(
        &self,
        _pos12: &Pose3,
        g1: &dyn Shape,
        g2: &dyn Shape,
        _prediction: Real,
    ) -> Result<Option<Contact>, Unsupported> {
        info!("contact {:?} <-> {:?}", g1.shape_type(), g2.shape_type());
        Err(Unsupported)
    }

    fn closest_points(
        &self,
        _pos12: &Pose3,
        g1: &dyn Shape,
        g2: &dyn Shape,
        _max_dist: Real,
    ) -> Result<ClosestPoints, Unsupported> {
        info!(
            "closest points {:?} <-> {:?}",
            g1.shape_type(),
            g2.shape_type()
        );
        Err(Unsupported)
    }

    fn cast_shapes(
        &self,
        _pos12: &Pose3,
        _local_vel12: Vec3,
        _g1: &dyn Shape,
        _g2: &dyn Shape,
        _options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Err(Unsupported)
    }

    fn cast_shapes_nonlinear(
        &self,
        _motion1: &NonlinearRigidMotion,
        _g1: &dyn Shape,
        _motion2: &NonlinearRigidMotion,
        _g2: &dyn Shape,
        _start_time: Real,
        _end_time: Real,
        _stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Err(Unsupported)
    }
}

impl<ContactData> PersistentQueryDispatcher<ContactManifoldData, ContactData> for SableDispatcher
where
    ContactData: Default + Copy,
{
    fn contact_manifolds(
        &self,
        pos12: &Pose3,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ContactManifold<ContactManifoldData, ContactData>>,
        workspace: &mut Option<ContactManifoldsWorkspace>,
    ) -> Result<(), Unsupported> {
        if g1.shape_type() != Custom && g2.shape_type() != Custom {
            return Err(Unsupported);
        }

        if g1.shape_type() == Custom && g2.shape_type() != Custom {
            self.static_world_vs_collider::<ContactData>(
                pos12,
                g1.as_shape::<LevelCollider>().unwrap(),
                g2,
                prediction,
                manifolds,
                false,
            );
        } else if g1.shape_type() != Custom && g2.shape_type() == Custom {
            self.static_world_vs_collider::<ContactData>(
                &pos12.inverse(),
                g2.as_shape::<LevelCollider>().unwrap(),
                g1,
                prediction,
                manifolds,
                true,
            );
        } else {
            assert_eq!(g1.shape_type(), Custom);
            assert_eq!(g2.shape_type(), Custom);

            let g1 = g1.as_shape::<LevelCollider>().unwrap();
            let g2 = g2.as_shape::<LevelCollider>().unwrap();

            if g1.is_static && !g2.is_static {
                self.world_vs_world::<ContactData>(
                    pos12,
                    g1,
                    g2,
                    prediction,
                    manifolds,
                    false,
                    voxel_workspace(workspace),
                );
            } else if !g1.is_static && !g2.is_static {
                let swap = {
                    let sable_data = self.sable_data.read().unwrap();
                    let body_1 = g1
                        .id
                        .map(|id| &sable_data.level_colliders[&(id as LevelColliderID)])
                        .unwrap();
                    let body_2 = g2
                        .id
                        .map(|id| &sable_data.level_colliders[&(id as LevelColliderID)])
                        .unwrap();

                    let extents_1 = body_1.local_bounds_max.unwrap()
                        - body_1.local_bounds_min.unwrap()
                        + IVec3::ONE;
                    let extents_2 = body_2.local_bounds_max.unwrap()
                        - body_2.local_bounds_min.unwrap()
                        + IVec3::ONE;

                    let volume_1 = extents_1.x * extents_1.y * extents_1.z;
                    let volume_2 = extents_2.x * extents_2.y * extents_2.z;

                    // Swap the bodies so we're always doing the least amount of work possible for collision detection
                    volume_1 < volume_2
                };

                if swap {
                    self.world_vs_world::<ContactData>(
                        &pos12.inverse(),
                        g2,
                        g1,
                        prediction,
                        manifolds,
                        true,
                        voxel_workspace(workspace),
                    );
                } else {
                    self.world_vs_world::<ContactData>(
                        pos12,
                        g1,
                        g2,
                        prediction,
                        manifolds,
                        false,
                        voxel_workspace(workspace),
                    );
                }
            }
        }

        Ok(())
    }

    fn contact_manifold_convex_convex(
        &self,
        _pos12: &Pose3,
        _g1: &dyn Shape,
        _g2: &dyn Shape,
        _normal_constraints1: Option<&dyn NormalConstraints>,
        _normal_constraints2: Option<&dyn NormalConstraints>,
        _prediction: Real,
        _manifold: &mut ContactManifold<ContactManifoldData, ContactData>,
    ) -> Result<(), Unsupported> {
        info!(
            "manifolds convex convex {:?} <-> {:?}",
            _g1.shape_type(),
            _g2.shape_type()
        );

        Err(Unsupported)
    }
}

impl SableDispatcher {
    fn static_world_vs_collider<ContactData: Default + Copy>(
        &self,
        pos12: &Pose3,
        g1: &LevelCollider,
        g2: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ContactManifold<ContactManifoldData, ContactData>>,
        swap: bool,
    ) {
        let physics_state = crate::get_physics_state();
        let sable_data = self.sable_data.read().unwrap();

        let collider_info = g1
            .id
            .map(|id| &sable_data.level_colliders[&(id as LevelColliderID)]);
        let center_of_mass_1 = collider_info.map_or(DVec3::ZERO, |b| b.center_of_mass.unwrap());

        let mut local_aabb = g2.compute_aabb(pos12);

        let margin: Real = 0.1;
        local_aabb.maxs += Vec3::splat(prediction + margin);
        local_aabb.mins -= Vec3::splat(prediction + margin);
        let local_aabb =
            Self::adjust_aabb_for_body(local_aabb, collider_info, center_of_mass_1, prediction);
        let (local_min, local_max) =
            Self::calculate_local_bounds(local_aabb, center_of_mass_1, prediction);

        let mut manifold_index = 0;

        let chunk_access: &dyn ChunkAccess = if let Some(info) = collider_info
            && info.has_own_chunks()
        {
            info
        } else {
            &*sable_data
        };

        for x in local_min.x..=local_max.x {
            for y in local_min.y..=local_max.y {
                for z in local_min.z..=local_max.z {
                    let Some(chunk) = chunk_access.get_chunk(x >> 4, y >> 4, z >> 4) else {
                        // chunk doesn't exist
                        continue;
                    };
                    let (block_id, _voxel_collider_state) = chunk.get_block(x & 15, y & 15, z & 15);

                    // block id's are unsigned, and offset by 1 to allow for a single "empty" at 0
                    if block_id == 0 {
                        continue;
                    }

                    let voxel_collider_data = &physics_state
                        .voxel_collider_map
                        .get((block_id - 1) as usize, IVec3::new(x, y, z));

                    if voxel_collider_data.is_none() {
                        continue;
                    }

                    let Some(voxel_collider_data) = &voxel_collider_data else {
                        unreachable!()
                    };

                    if voxel_collider_data.is_fluid {
                        continue;
                    }

                    for (min_x, min_y, min_z, max_x, max_y, max_z) in
                        &voxel_collider_data.collision_boxes
                    {
                        if manifolds.len() <= manifold_index {
                            manifolds.push(ContactManifold::new());
                        }

                        let center = DVec3::new(
                            ((min_x + max_x) / 2.0) as f64,
                            ((min_y + max_y) / 2.0) as f64,
                            ((min_z + max_z) / 2.0) as f64,
                        ) + DVec3::new(x as f64, y as f64, z as f64)
                            - center_of_mass_1;
                        let center =
                            Vec3::new(center.x as Real, center.y as Real, center.z as Real);

                        let half_extents = Vec3::new(
                            (max_x - min_x) / 2.0,
                            (max_y - min_y) / 2.0,
                            (max_z - min_z) / 2.0,
                        );

                        // Translate to match the center of the current block
                        let mut block_isometry = *pos12;
                        block_isometry.translation -= center;

                        if !swap {
                            DefaultQueryDispatcher
                                .contact_manifold_convex_convex(
                                    &block_isometry,
                                    &rapier3d::parry::shape::Cuboid::new(Vec3::new(
                                        half_extents.x,
                                        half_extents.y,
                                        half_extents.z,
                                    )),
                                    g2,
                                    None,
                                    None,
                                    prediction,
                                    &mut manifolds[manifold_index],
                                )
                                .expect("uh oh");
                        } else {
                            DefaultQueryDispatcher
                                .contact_manifold_convex_convex(
                                    &block_isometry.inverse(),
                                    g2,
                                    &rapier3d::parry::shape::Cuboid::new(Vec3::new(
                                        half_extents.x,
                                        half_extents.y,
                                        half_extents.z,
                                    )),
                                    None,
                                    None,
                                    prediction,
                                    &mut manifolds[manifold_index],
                                )
                                .expect("uh oh");
                        }

                        if !manifolds[manifold_index].points.is_empty() {
                            let index = self
                                .manifold_info_map
                                .counter
                                .fetch_add(1, Ordering::Relaxed);

                            let block_pos = IVec3::new(x, y, z);
                            self.manifold_info_map.list.insert(
                                index,
                                if swap {
                                    SableManifoldInfo {
                                        pos_a: IVec3::ZERO,
                                        pos_b: block_pos,
                                        col_a: 0,
                                        col_b: block_id as usize,
                                    }
                                } else {
                                    SableManifoldInfo {
                                        pos_a: block_pos,
                                        pos_b: IVec3::ZERO,
                                        col_a: block_id as usize,
                                        col_b: 0,
                                    }
                                },
                            );

                            manifolds[manifold_index].data.user_data =
                                voxel_collider_data.get_user_data() | ((index << 1) as u32);
                        } else {
                            manifolds[manifold_index].data.user_data =
                                voxel_collider_data.get_user_data();
                        }

                        if collider_info.is_some()
                            && let Some(_velocities) = collider_info.unwrap().fake_velocities
                        {
                            manifolds[manifold_index].data.user_data |= NEEDS_HOOKS_USER_DATA;
                        }

                        for point in &mut manifolds[manifold_index].points {
                            let diff = Vec3::new(center.x, center.y, center.z);
                            match swap {
                                true => point.local_p2 -= diff,
                                false => point.local_p1 += diff,
                            }
                        }

                        manifold_index += 1;
                    }
                }
            }
        }

        if manifolds.len() > manifold_index {
            manifolds.truncate(manifold_index);
        }
    }

    fn world_vs_world<ContactData: Default + Copy>(
        &self,
        pos12: &Pose3,
        g1: &LevelCollider,
        g2: &LevelCollider,
        prediction: Real,
        manifolds: &mut Vec<ContactManifold<ContactManifoldData, ContactData>>,
        swap: bool,
        workspace: &mut VoxelManifoldWorkspace,
    ) {
        workspace.next.clear();
        let physics_state = crate::get_physics_state();
        let sable_data = self.sable_data.read().unwrap();

        let collider_info_1 = g1
            .id
            .map(|id| &sable_data.level_colliders[&(id as LevelColliderID)]);
        let collider_info_2 = &sable_data.level_colliders[&(g2.id.unwrap() as LevelColliderID)];
        let center_of_mass_1 = collider_info_1.map_or(DVec3::ZERO, |b| b.center_of_mass.unwrap());
        let center_of_mass_2 = collider_info_2.center_of_mass.unwrap();

        let chunk_access_1: &dyn ChunkAccess = if let Some(info) = collider_info_1
            && info.has_own_chunks()
        {
            info
        } else {
            &*sable_data
        };

        let chunk_access_2: &dyn ChunkAccess = if collider_info_2.has_own_chunks() {
            collider_info_2
        } else {
            &*sable_data
        };

        // let local_aabb = g2.compute_aabb(&pos12);
        // let local_aabb = Self::adjust_aabb_for_body(local_aabb, body_1, center_of_mass_1, prediction);
        // let (local_min, local_max) = Self::calculate_local_bounds(local_aabb, center_of_mass_1, prediction);

        let mut manifold_index = 0;

        let mut pairs = find_collision_pairs(
            collider_info_2,
            collider_info_1,
            pos12,
            prediction,
            256,
            false,
            &sable_data.octree_chunks,
        );

        // Only the terrain/body path is eligible. Own chunks and a geometry
        // version make this cache independent of other bodies and terrain edits.
        let mut merged = None;
        if collider_info_1.is_none()
            && collider_info_2.has_own_chunks()
            && collider_info_2.static_mount.is_none()
            && collider_info_2.fake_velocities.is_none()
        {
            merged = workspace.merged_box_for(
                collider_info_2.geometry_version,
                collider_info_2.local_bounds_min.unwrap(),
                collider_info_2.local_bounds_max.unwrap(),
                |pos| {
                    let chunk = chunk_access_2.get_chunk(pos.x >> 4, pos.y >> 4, pos.z >> 4)?;
                    let (id, state) = chunk.get_block(pos.x & 15, pos.y & 15, pos.z & 15);
                    #[cfg(feature = "benchmark-profiler")]
                    {
                        static VOXEL_SAMPLES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                        if std::env::var("WE_NATIVE_PROFILE").as_deref() == Ok("true")
                            && VOXEL_SAMPLES.fetch_add(1, Ordering::Relaxed) < 16 {
                            let data = id.checked_sub(1).and_then(|index| physics_state.voxel_collider_map.voxel_colliders.get(index as usize)).and_then(Option::as_ref);
                            eprintln!("WE_CUBOID_VOXEL pos={:?} id={} state={:?} data={:?}", pos, id, state, data.map(|data| (&data.collision_boxes, data.dynamic, data.is_fluid, data.friction, data.restitution, data.get_user_data(), data.contact_method.is_some())));
                        }
                    }
                    if id == 0 {
                        return Some(None);
                    }
                    if state == VoxelPhysicsState::Empty {
                        return None;
                    }
                    let registry_data = physics_state
                        .voxel_collider_map
                        .voxel_colliders
                        .get((id - 1) as usize)?
                        .as_ref()?;
                    if !is_plain_unit_cube(registry_data) {
                        return None;
                    }
                    let data = physics_state.voxel_collider_map.get((id - 1) as usize, pos);
                    if !is_plain_unit_cube(data.as_ref()?) {
                        return None;
                    }
                    Some(Some(id))
                },
            );
            // Static blocks may inspect the other voxel's identity in their
            // hooks. Preserve the original per-voxel path for all such pairs.
            if merged.is_some()
                && pairs.iter().any(|(pos, _)| {
                    let Some(chunk) = chunk_access_1.get_chunk(pos.x >> 4, pos.y >> 4, pos.z >> 4)
                    else {
                        return false;
                    };
                    let (id, _) = chunk.get_block(pos.x & 15, pos.y & 15, pos.z & 15);
                    if id != 0
                        && physics_state
                            .voxel_collider_map
                            .voxel_colliders
                            .get((id - 1) as usize)
                            .and_then(Option::as_ref)
                            .is_some_and(|data| {
                                data.dynamic || data.is_fluid || data.get_user_data() != 0
                            })
                    {
                        return true;
                    }
                    id != 0
                        && physics_state
                            .voxel_collider_map
                            .get((id - 1) as usize, *pos)
                            .as_ref()
                            .is_some_and(|data| {
                                data.dynamic || data.is_fluid || data.get_user_data() != 0
                            })
                })
            {
                merged = None;
            }
            if let Some(cuboid) = merged {
                // The original octree query supplies the conservative terrain
                // candidates for the exact cube union. Visit each terrain voxel
                // once against that union, instead of once per constituent cube.
                pairs.sort_unstable_by_key(|(pos, _)| (pos.x, pos.y, pos.z));
                pairs.dedup_by(|a, b| a.0 == b.0);
                for (_, dynamic_pos) in &mut pairs {
                    *dynamic_pos = cuboid.min;
                }
            }
        }
        let merged_boxes = merged.map(|cuboid| {
            [(
                0.0,
                0.0,
                0.0,
                cuboid.extents.x as f32,
                cuboid.extents.y as f32,
                cuboid.extents.z as f32,
            )]
        });

        let mut terrain_cuboids = std::collections::HashMap::new();
        if merged.is_some() {
            let eligible: Vec<_> = pairs.iter().filter_map(|(pos, _)| {
                let chunk = chunk_access_1.get_chunk(pos.x >> 4, pos.y >> 4, pos.z >> 4)?;
                let (id, state) = chunk.get_block(pos.x & 15, pos.y & 15, pos.z & 15);
                if id == 0 || state == VoxelPhysicsState::Empty { return None; }
                let data = physics_state.voxel_collider_map.get((id - 1) as usize, *pos)?;
                is_plain_unit_cube(data).then_some((*pos, id))
            }).collect();
            let rectangles = merge_terrain_layers(&eligible);
            let mut replaced = std::collections::HashSet::new();
            for rectangle in rectangles {
                for &(pos, id) in &eligible {
                    if id == rectangle.block_id && pos != rectangle.min
                        && rectangle.contains_voxel(pos) {
                        replaced.insert(pos);
                    }
                }
                terrain_cuboids.insert(rectangle.min, rectangle);
            }
            pairs.retain(|(pos, _)| !replaced.contains(pos));
        }

        for (static_pos, dynamic_pos) in pairs.iter() {
            let static_x = static_pos.x;
            let static_y = static_pos.y;
            let static_z = static_pos.z;

            let other_bx = dynamic_pos.x;
            let other_by = dynamic_pos.y;
            let other_bz = dynamic_pos.z;

            let Some(chunk) = chunk_access_1.get_chunk(static_x >> 4, static_y >> 4, static_z >> 4)
            else {
                // chunk doesn't exist
                continue;
            };
            let (block_id, voxel_collider_state) =
                chunk.get_block(static_x & 15, static_y & 15, static_z & 15);

            // block id's are unsigned, and offset by 1 to allow for a single "empty" at 0
            if block_id == 0 {
                continue;
            }

            let voxel_collider_data = &physics_state.voxel_collider_map.get(
                (block_id - 1) as usize,
                IVec3::new(static_x, static_y, static_z),
            );

            let Some(voxel_collider_data) = &voxel_collider_data else {
                continue;
            };

            let terrain_box = terrain_cuboids.get(static_pos).map(|cuboid: &MergedVoxelBox| [(
                0.0, 0.0, 0.0, cuboid.extents.x as f32, cuboid.extents.y as f32, cuboid.extents.z as f32,
            )]);
            for (min_x, min_y, min_z, max_x, max_y, max_z) in terrain_box.as_ref().map_or(
                voxel_collider_data.collision_boxes.as_slice(), |boxes| boxes.as_slice(),
            ) {
                if manifolds.len() <= manifold_index {
                    manifolds.push(ContactManifold::new());
                }

                let center = DVec3::new(
                    ((min_x + max_x) / 2.0) as f64,
                    ((min_y + max_y) / 2.0) as f64,
                    ((min_z + max_z) / 2.0) as f64,
                ) + DVec3::new(static_x as f64, static_y as f64, static_z as f64)
                    - center_of_mass_1;
                let center = Vec3::new(center.x as Real, center.y as Real, center.z as Real);

                let half_extents = Vec3::new(
                    (max_x - min_x) / 2.0,
                    (max_y - min_y) / 2.0,
                    (max_z - min_z) / 2.0,
                );

                // Translate to match the center of the current block
                let mut block_isometry = *pos12;
                block_isometry.translation -= center;

                let Some(other_chunk) =
                    chunk_access_2.get_chunk(other_bx >> 4, other_by >> 4, other_bz >> 4)
                else {
                    // chunk doesn't exist
                    continue;
                };
                let (other_block_id, other_voxel_collider_state) =
                    other_chunk.get_block(other_bx & 15, other_by & 15, other_bz & 15);

                // block id's are unsigned, and offset by 1 to allow for a single "empty" at 0
                if other_block_id == 0 {
                    continue;
                }
                debug_assert!(merged.is_none_or(|cuboid| cuboid.block_id == other_block_id));

                if if terrain_box.is_some() {
                    false // Its internal voxel faces have disappeared into the exact union.
                } else if merged.is_some() {
                    voxel_collider_state == Interior
                } else {
                    Self::can_ignore_collision(voxel_collider_state, other_voxel_collider_state)
                } {
                    continue;
                }

                let other_voxel_collider_data = &physics_state.voxel_collider_map.get(
                    (other_block_id - 1) as usize,
                    IVec3::new(other_bx, other_by, other_bz),
                );

                let Some(other_voxel_collider_data) = &other_voxel_collider_data else {
                    continue;
                };

                for (
                    other_min_x,
                    other_min_y,
                    other_min_z,
                    other_max_x,
                    other_max_y,
                    other_max_z,
                ) in merged_boxes.as_ref().map_or(
                    other_voxel_collider_data.collision_boxes.as_slice(),
                    |boxes| boxes.as_slice(),
                ) {
                    if manifolds.len() <= manifold_index {
                        manifolds.push(ContactManifold::new());
                    }

                    let other_center =
                        DVec3::new(
                            ((other_min_x + other_max_x) / 2.0) as f64,
                            ((other_min_y + other_max_y) / 2.0) as f64,
                            ((other_min_z + other_max_z) / 2.0) as f64,
                        ) + DVec3::new(other_bx as f64, other_by as f64, other_bz as f64)
                            - center_of_mass_2;
                    let other_center = Vec3::new(
                        other_center.x as Real,
                        other_center.y as Real,
                        other_center.z as Real,
                    );

                    let other_half_extents = Vec3::new(
                        (other_max_x - other_min_x) / 2.0,
                        (other_max_y - other_min_y) / 2.0,
                        (other_max_z - other_min_z) / 2.0,
                    );

                    // combine block isometries
                    let mut combined_block_isometry = block_isometry;

                    let transformed = combined_block_isometry.rotation.mul_vec3(other_center);

                    combined_block_isometry.translation += transformed;

                    let mut new_manifold: ContactManifold<ContactManifoldData, ContactData> =
                        ContactManifold::new();
                    contact_manifold_cuboid_cuboid_shapes(
                        &combined_block_isometry,
                        &rapier3d::parry::shape::Cuboid::new(half_extents),
                        &rapier3d::parry::shape::Cuboid::new(other_half_extents),
                        prediction,
                        &mut new_manifold,
                    );

                    if !is_interior_collision(
                        chunk_access_1,
                        chunk_access_2,
                        collider_info_1,
                        collider_info_2,
                        IVec3::new(static_x, static_y, static_z),
                        IVec3::new(other_bx, other_by, other_bz),
                        center,
                        other_center,
                        center_of_mass_1,
                        center_of_mass_2,
                        &mut new_manifold,
                        merged.is_some(),
                        terrain_box.is_some(),
                    ) {
                        // No points means no solver constraint or collision event.
                        // Avoid retaining an empty manifold and allocating hook
                        // metadata for a voxel pair rejected by contact generation
                        // or the interior-face filter.
                        if new_manifold.points.is_empty() {
                            continue;
                        }
                        let key = VoxelManifoldKey {
                            position_a: IVec3::new(static_x, static_y, static_z),
                            position_b: IVec3::new(other_bx, other_by, other_bz),
                            collider_a: block_id,
                            collider_b: other_block_id,
                            state_a: voxel_collider_state,
                            state_b: other_voxel_collider_state,
                            box_a: [
                                min_x.to_bits(),
                                min_y.to_bits(),
                                min_z.to_bits(),
                                max_x.to_bits(),
                                max_y.to_bits(),
                                max_z.to_bits(),
                            ],
                            box_b: [
                                other_min_x.to_bits(),
                                other_min_y.to_bits(),
                                other_min_z.to_bits(),
                                other_max_x.to_bits(),
                                other_max_y.to_bits(),
                                other_max_z.to_bits(),
                            ],
                            swapped: swap,
                        };
                        match_voxel_contacts(
                            &key,
                            workspace.previous.get(manifold_index),
                            &manifolds[manifold_index],
                            &mut new_manifold,
                        );
                        workspace.next.push(key);
                        let index = self
                            .manifold_info_map
                            .counter
                            .fetch_add(1, Ordering::Relaxed);

                        self.manifold_info_map.list.insert(
                            index,
                            if swap {
                                SableManifoldInfo {
                                    pos_a: IVec3::new(other_bx, other_by, other_bz),
                                    pos_b: IVec3::new(static_x, static_y, static_z),
                                    col_a: other_block_id as usize,
                                    col_b: block_id as usize,
                                }
                            } else {
                                SableManifoldInfo {
                                    pos_a: IVec3::new(static_x, static_y, static_z),
                                    pos_b: IVec3::new(other_bx, other_by, other_bz),
                                    col_a: block_id as usize,
                                    col_b: other_block_id as usize,
                                }
                            },
                        );

                        new_manifold.data.user_data = voxel_collider_data.get_user_data()
                            | other_voxel_collider_data.get_user_data()
                            | ((index << 1) as u32);

                        if let Some(_velocities) = collider_info_2.fake_velocities {
                            new_manifold.data.user_data |= NEEDS_HOOKS_USER_DATA;
                        }
                        if let Some(info) = collider_info_1
                            && info.fake_velocities.is_some()
                        {
                            new_manifold.data.user_data |= NEEDS_HOOKS_USER_DATA;
                        }

                        manifolds[manifold_index] = new_manifold;

                        for point in &mut manifolds[manifold_index].points {
                            point.local_p1 += center;
                            point.local_p2 += other_center;
                        }

                        manifold_index += 1;
                    }
                }
            }
        }

        // swap bodies in the manifolds
        if swap {
            for manifold in manifolds.iter_mut() {
                for point in &mut manifold.points {
                    // swap positions
                    std::mem::swap(&mut point.local_p1, &mut point.local_p2);
                }

                // swap normals
                std::mem::swap(&mut manifold.local_n1, &mut manifold.local_n2);
            }
        }

        if manifolds.len() > manifold_index {
            manifolds.truncate(manifold_index);
        }
        std::mem::swap(&mut workspace.previous, &mut workspace.next);
    }

    /// Adjusts the AABB for a given body
    #[inline(always)]
    fn adjust_aabb_for_body(
        mut local_aabb: Aabb,
        body: Option<&ActiveLevelColliderInfo>,
        center_of_mass: DVec3,
        prediction: Real,
    ) -> Aabb {
        if let Some(body) = body {
            let local_bounds_min = body.local_bounds_min.unwrap();
            let local_bounds_max = body.local_bounds_max.unwrap();

            let body_aabb = Aabb::new(
                (local_bounds_min.as_dvec3() - center_of_mass).as_vec3() - prediction,
                (local_bounds_max.as_dvec3() - center_of_mass).as_vec3() - prediction,
            );

            local_aabb = local_aabb.intersection(&body_aabb).unwrap_or(local_aabb);
        }

        local_aabb
    }

    /// Calculates local bounds based on AABB and prediction
    #[inline(always)]
    fn calculate_local_bounds(
        aabb: Aabb,
        center_of_mass: DVec3,
        prediction: Real,
    ) -> (IVec3, IVec3) {
        let mins = aabb.mins.as_dvec3() + center_of_mass - prediction as f64;
        let maxs = aabb.maxs.as_dvec3() + center_of_mass + prediction as f64;

        let local_min = mins.floor().as_ivec3();
        let local_max = maxs.floor().as_ivec3();

        (local_min, local_max)
    }

    #[inline(always)]
    #[must_use]
    fn can_ignore_collision(
        voxel_collider_state: VoxelPhysicsState,
        other_voxel_collider_state: VoxelPhysicsState,
    ) -> bool {
        (other_voxel_collider_state == voxel_collider_state && voxel_collider_state == Face)
            || voxel_collider_state == Interior
            || other_voxel_collider_state == Interior
            || (voxel_collider_state == Edge && other_voxel_collider_state == Face)
            || (voxel_collider_state == Face && other_voxel_collider_state == Edge)
    }
}

fn get_block_pos(vec: DVec3) -> IVec3 {
    IVec3::new(
        vec.x.floor() as i32,
        vec.y.floor() as i32,
        vec.z.floor() as i32,
    )
}

fn is_interior_collision<ManifoldData: Default + Clone, ContactData: Default + Copy>(
    chunk_access_1: &dyn ChunkAccess,
    chunk_access_2: &dyn ChunkAccess,
    collider_info_1: Option<&ActiveLevelColliderInfo>,
    collider_info_2: &ActiveLevelColliderInfo,
    block_a: IVec3,
    block_b: IVec3,
    center: Vec3,
    other_center: Vec3,
    center_of_mass_1: DVec3,
    center_of_mass_2: DVec3,
    manifold: &mut ContactManifold<ManifoldData, ContactData>,
    other_is_solid_cuboid: bool,
    terrain_is_solid_cuboid: bool,
) -> bool {
    let physics_state = crate::get_physics_state();

    manifold.points.retain(|point| {
        if collider_info_1.is_none()
            || (collider_info_1.unwrap().local_bounds_min.unwrap()
                != collider_info_1.unwrap().local_bounds_max.unwrap())
        {
            // A voxel-size inward scale would erase real outer faces on a
            // larger rectangle. Check its actual surface and the fresh terrain
            // beyond it, including cells outside the conservative query clip.
            let local_p1 = if terrain_is_solid_cuboid { point.local_p1 } else { point.local_p1 * 0.997 };
            let world_p1 = (local_p1 + center).as_dvec3() + center_of_mass_1;
            let normal1 = manifold.local_n1.as_dvec3();

            let displaced_p1 = world_p1 + normal1 * 0.01;

            if is_inside_voxel_collider(chunk_access_1, &physics_state, block_a, displaced_p1) {
                return false;
            }
        }

        // A merged cuboid has only outer faces. Applying a voxel-size inward
        // offset to a larger cuboid could incorrectly reject its real surface.
        if !other_is_solid_cuboid
            && collider_info_2.local_bounds_min.unwrap()
                != collider_info_2.local_bounds_max.unwrap()
        {
            let normal2 = manifold.local_n2.as_dvec3();

            // we have to "pull in the points" a tiny bit incase they're outside of the block slightly off-normal
            let world_p2 = (point.local_p2 * INTERIOR_COLLISION_SCALE_FACTOR + other_center)
                .as_dvec3()
                + center_of_mass_2;

            let displaced_p2 = world_p2 + normal2 * INTERIOR_COLLISION_CHECK_DISTANCE;

            if is_inside_voxel_collider(chunk_access_2, &physics_state, block_b, displaced_p2) {
                return false;
            }
        }

        true
    });

    false
}

fn is_inside_voxel_collider(
    chunk_access: &dyn ChunkAccess,
    physics_state: &PhysicsState,
    ignore_block: IVec3,
    world_pos: DVec3,
) -> bool {
    let block_pos = get_block_pos(world_pos);

    if ignore_block == block_pos {
        return false;
    }

    let Some(chunk) = chunk_access.get_chunk(block_pos.x >> 4, block_pos.y >> 4, block_pos.z >> 4)
    else {
        return false;
    };
    let (block_id, _) = chunk.get_block(block_pos.x & 15, block_pos.y & 15, block_pos.z & 15);

    if block_id == 0 {
        return false;
    }

    let voxel_data = physics_state
        .voxel_collider_map
        .get((block_id - 1) as usize, block_pos);

    let Some(voxel_data) = voxel_data else {
        return false;
    };

    if voxel_data.is_fluid {
        return false;
    }

    let local_pos = world_pos - block_pos.as_dvec3();

    for &(min_x, min_y, min_z, max_x, max_y, max_z) in &voxel_data.collision_boxes {
        if local_pos.x as Real >= min_x
            && local_pos.x as Real <= max_x
            && local_pos.y as Real >= min_y
            && local_pos.y as Real <= max_y
            && local_pos.z as Real >= min_z
            && local_pos.z as Real <= max_z
        {
            return true;
        }
    }

    false
}

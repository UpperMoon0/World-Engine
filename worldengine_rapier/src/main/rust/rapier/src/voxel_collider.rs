use std::collections::HashMap;

use jni::JNIEnv;
use jni::objects::{JClass, JDoubleArray, JObject};
use jni::sys::{jboolean, jdouble, jint};
use marten::Real;
use marten::level::{SableMethodID, VoxelColliderData};
use rapier3d::glamx::IVec3;

use crate::get_physics_state_mut;

/// The physics data of a blockstate
#[derive(Debug)]
pub struct VoxelColliderMap {
    pub(crate) voxel_colliders: Vec<Option<VoxelColliderData>>,
    dynamic_colliders: HashMap<IVec3, Option<VoxelColliderData>>,
}

// Chunk block IDs reserve zero for air and encode a registry index plus one.
// Callback guards receive those IDs, unlike get(), which receives an index.
fn registered_block<T>(entries: &[Option<T>], block_id: usize) -> Option<&T> {
    entries.get(block_id.checked_sub(1)?).and_then(Option::as_ref)
}

impl VoxelColliderMap {
    pub fn new() -> Self {
        Self {
            voxel_colliders: Vec::new(),
            dynamic_colliders: HashMap::new(),
        }
    }

    pub fn get(&self, index: usize, block_pos: IVec3) -> Option<&VoxelColliderData> {
        let collider = &self.voxel_colliders[index];

        if collider.is_some() && collider.as_ref().unwrap().dynamic {
            let dynamic_collider = self.dynamic_colliders.get(&block_pos);

            if let Some(data) = dynamic_collider {
                return data.as_ref();
            }
        }

        collider.as_ref()
    }

    pub fn requires_java_callback(&self, block_id: usize) -> bool {
        registered_block(&self.voxel_colliders, block_id)
            .is_some_and(|data| data.contact_events.is_some())
    }
}

#[cfg(test)]
mod tests {
    use super::registered_block;

    #[test]
    fn packed_block_ids_resolve_callbacks_without_air_or_neighbor_aliasing() {
        // Independent registry: the first and fourth entries have callbacks;
        // the second is ordinary and the third is absent. Chunk IDs are 1..4.
        let entries = [Some(true), Some(false), None, Some(true)];
        let packed_ids = [0, 1, 2, 3, 4, 0, 2, 5, usize::MAX];
        let observed: Vec<_> = packed_ids.iter().map(|id| {
            registered_block(&entries, *id).copied().unwrap_or(false)
        }).collect();
        assert_eq!(observed, [false, true, false, false, true, false, false, false, false]);
        assert_eq!(observed.iter().filter(|requires_callback| **requires_callback).count(), 2);
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_newVoxelCollider<
    'local,
>(
    mut env: JNIEnv<'static>,
    _class: JClass<'local>,
    friction: jdouble,
    volume: jdouble,
    restitution: jdouble,
    is_fluid: jboolean,
    contact_events: JObject,
) -> jint {
    let mut state = get_physics_state_mut();

    let next_index = state.voxel_collider_map.voxel_colliders.len();

    let global_ref = if contact_events.is_null() {
        None
    } else {
        Some(env.new_global_ref(contact_events).unwrap())
    };

    let global_method = if let Some(global_ref_value) = &global_ref {
        let class = env.get_object_class(global_ref_value).unwrap();

        let id = SableMethodID(
            env.get_method_id(
                class,
                String::from("onCollision"),
                String::from("(IIIIIIDDDDZ)[D"),
            )
            .unwrap(),
        );
        Some(id)
    } else {
        None
    };

    state
        .voxel_collider_map
        .voxel_colliders
        .push(Some(VoxelColliderData {
            collision_boxes: Vec::new(),
            is_fluid: is_fluid > 0,
            friction: friction as Real,
            volume: volume as Real,
            restitution: restitution as Real,
            contact_events: global_ref,
            contact_method: global_method,
            // Java's newVoxelCollider has five arguments. These entries are
            // cached block-state shapes, not per-position dynamic overrides.
            dynamic: false,
        }));

    next_index as jint
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addVoxelColliderBox<
    'local,
>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    index: jint,
    box_bounds: JDoubleArray<'local>,
) {
    let mut state = get_physics_state_mut();

    let mut bounds: [jdouble; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    env.get_double_array_region(box_bounds, 0, &mut bounds)
        .unwrap();

    if let Some(data) = &mut state.voxel_collider_map.voxel_colliders[index as usize] {
        data.collision_boxes.push((
            bounds[0] as f32,
            bounds[1] as f32,
            bounds[2] as f32,
            bounds[3] as f32,
            bounds[4] as f32,
            bounds[5] as f32,
        ));
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_clearVoxelColliderBoxes<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    index: jint,
) {
    let mut state = get_physics_state_mut();

    if let Some(data) = &mut state.voxel_collider_map.voxel_colliders[index as usize] {
        data.collision_boxes.clear()
    }
}

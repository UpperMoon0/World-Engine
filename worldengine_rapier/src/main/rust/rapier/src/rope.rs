use std::collections::HashMap;

use jni::JNIEnv;
use jni::objects::{JClass, JDoubleArray};
use jni::sys::{jboolean, jdouble, jint, jlong, jsize};
use marten::Real;
use rapier3d::dynamics::{GenericJointBuilder, JointAxis, RigidBodyBuilder, SpringCoefficients};
use rapier3d::geometry::{ColliderBuilder, SharedShape};
use rapier3d::glamx::DVec3;
use rapier3d::math::Vec3;
use rapier3d::prelude::{
    ImpulseJointHandle, ImpulseJointSet, JointAxesMask, RigidBodyHandle, RopeJointBuilder,
};

use crate::config::{JOINT_SPRING_DAMPING_RATIO, JOINT_SPRING_FREQUENCY};
use crate::groups::ROPE_GROUP;
use crate::scene::{LevelColliderID, PhysicsScene, SableSceneData, SimulationSceneData};
use crate::with_handle;

const MIN_BOUND_STIFFNESS: Real = 150.0;
const MIN_BOUND_DAMPING: Real = 10.0;

struct RopeAttachment {
    joint: Option<ImpulseJointHandle>,
    sub_level_id: Option<LevelColliderID>,
    location: DVec3,
}

struct RopeStrand {
    points: Vec<RigidBodyHandle>,
    joints: Vec<(ImpulseJointHandle, ImpulseJointHandle)>,

    point_radius: Real,
    first_joint_length: Real,

    start_attachment: Option<RopeAttachment>,
    end_attachment: Option<RopeAttachment>,
}
#[derive(Default)]
pub struct RopeMap {
    counting_id: usize,
    ropes: HashMap<usize, RopeStrand>,
}

impl RopeMap {
    pub fn is_empty(&self) -> bool {
        self.ropes.is_empty()
    }

    pub fn merge_from(&mut self, other: &mut Self, delta: DVec3) {
        for (id, mut rope) in other.ropes.drain() {
            if let Some(start) = &mut rope.start_attachment {
                start.location -= delta;
            }
            if let Some(end) = &mut rope.end_attachment {
                end.location -= delta;
            }
            self.ropes.insert(id, rope);
        }
    }
}

// Attachment intent survives load ordering and body removal. Missing native bodies
// are retried on the next tick instead of panicking across the JNI boundary.
fn refresh_attachment(
    attachment: &mut RopeAttachment,
    point: Option<RigidBodyHandle>,
    ground: Option<RigidBodyHandle>,
    bodies: &HashMap<LevelColliderID, RigidBodyHandle>,
    colliders: &HashMap<LevelColliderID, crate::ActiveLevelColliderInfo>,
    rigid_bodies: &rapier3d::prelude::RigidBodySet,
    joints: &mut ImpulseJointSet,
) {
    let target = attachment
        .sub_level_id
        .and_then(|id| bodies.get(&id).copied())
        .or_else(|| {
            if attachment.sub_level_id.is_none() {
                ground
            } else {
                None
            }
        });
    let anchor = match attachment.sub_level_id {
        Some(id) => colliders
            .get(&id)
            .and_then(|c| c.center_of_mass)
            .map(|com| attachment.location - com),
        None => Some(attachment.location),
    };
    let valid = target
        .zip(point)
        .zip(anchor)
        .filter(|((target, point), _)| {
            rigid_bodies.contains(*target) && rigid_bodies.contains(*point)
        });
    let Some(((target, point), anchor)) = valid else {
        if let Some(joint) = attachment.joint.take() {
            joints.remove(joint, true);
        }
        return;
    };
    if let Some(joint) = attachment.joint.and_then(|h| joints.get_mut(h, false)) {
        if joint.body1 == target && joint.body2 == point {
            joint.data.set_local_anchor1(anchor.as_vec3());
            return;
        }
    }
    if let Some(old) = attachment.joint.take() {
        joints.remove(old, true);
    }
    let joint = RopeJointBuilder::new(0.0)
        .local_anchor1(anchor.as_vec3())
        .local_anchor2(Vec3::ZERO)
        .softness(SpringCoefficients::new(
            JOINT_SPRING_FREQUENCY,
            JOINT_SPRING_DAMPING_RATIO,
        ));
    attachment.joint = Some(joints.insert(target, point, joint.build(), true));
}

fn refresh_ropes(
    ropes: &mut RopeMap,
    ground: Option<RigidBodyHandle>,
    bodies: &HashMap<LevelColliderID, RigidBodyHandle>,
    colliders: &HashMap<LevelColliderID, crate::ActiveLevelColliderInfo>,
    rigid_bodies: &rapier3d::prelude::RigidBodySet,
    joints: &mut ImpulseJointSet,
) {
    for rope in ropes.ropes.values_mut() {
        if let Some(attachment) = &mut rope.start_attachment {
            refresh_attachment(
                attachment,
                rope.points.first().copied(),
                ground,
                bodies,
                colliders,
                rigid_bodies,
                joints,
            );
        }
        if let Some(attachment) = &mut rope.end_attachment {
            refresh_attachment(
                attachment,
                rope.points.last().copied(),
                ground,
                bodies,
                colliders,
                rigid_bodies,
                joints,
            );
        }
    }
}

pub fn tick(scene: &PhysicsScene) {
    let mut sable_data = scene.sable_data.write().unwrap();
    let mut sim = scene.sim_data.write().unwrap();
    let SableSceneData {
        rope_map,
        rigid_bodies,
        level_colliders,
        ..
    } = &mut *sable_data;
    let SimulationSceneData {
        rigid_body_set,
        impulse_joint_set,
        ..
    } = &mut *sim;
    refresh_ropes(
        rope_map,
        scene.ground_handle,
        rigid_bodies,
        level_colliders,
        rigid_body_set,
        impulse_joint_set,
    );
}

fn set_attachment(
    ropes: &mut RopeMap,
    joints: &mut ImpulseJointSet,
    rope_id: usize,
    sub_level_id: Option<LevelColliderID>,
    location: DVec3,
    end: bool,
) {
    let Some(strand) = ropes.ropes.get_mut(&rope_id) else {
        return;
    };
    let slot = if end {
        &mut strand.end_attachment
    } else {
        &mut strand.start_attachment
    };
    if let Some(joint) = slot.take().and_then(|old| old.joint) {
        joints.remove(joint, true);
    }
    *slot = Some(RopeAttachment {
        joint: None,
        sub_level_id,
        location,
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_createRope<'local>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    point_radius: jdouble,
    first_joint_length: jdouble,
    points: JDoubleArray<'local>,
    num_points: jint,
) -> jlong {
    let mut coordinates = vec![0.0; (num_points * 3) as usize];
    env.get_double_array_region(points, 0, &mut coordinates)
        .unwrap();

    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let universal_drag = scene.universal_drag;

        let mut vec = Vec::with_capacity(num_points as usize);
        for i in 0..(num_points as usize) {
            let coordinate = Vec3::new(
                coordinates[i * 3] as Real,
                coordinates[i * 3 + 1] as Real,
                coordinates[i * 3 + 2] as Real,
            );

            let handle = create_rope_body(
                &mut sim_data,
                universal_drag,
                coordinate,
                point_radius as Real,
            );

            vec.push(handle);
        }

        let mut joints: Vec<(ImpulseJointHandle, ImpulseJointHandle)> =
            Vec::with_capacity(vec.len() - 1);
        for i in 0..vec.len() - 1 {
            let point_handle_0 = &vec[i];
            let point_handle_1 = &vec[i + 1];

            let length = if i == 0 {
                first_joint_length as Real
            } else {
                1.0
            };
            joints.push(add_rope_joint(
                &mut sim_data.impulse_joint_set,
                point_handle_0,
                point_handle_1,
                length,
            ));
        }

        let strand = RopeStrand {
            points: vec,
            point_radius: point_radius as Real,
            first_joint_length: first_joint_length as Real,
            start_attachment: None,
            end_attachment: None,
            joints,
        };

        sable_data.rope_map.counting_id += 1;
        let id = sable_data.rope_map.counting_id;

        sable_data.rope_map.ropes.insert(id, strand);

        id as jlong
    })
}

fn add_rope_joint(
    impulse_joint_set: &mut ImpulseJointSet,
    point_handle_0: &RigidBodyHandle,
    point_handle_1: &RigidBodyHandle,
    length: Real,
) -> (ImpulseJointHandle, ImpulseJointHandle) {
    let mut joint = RopeJointBuilder::new(length)
        .local_anchor1(Vec3::ZERO)
        .local_anchor2(Vec3::ZERO)
        .softness(SpringCoefficients::new(
            JOINT_SPRING_FREQUENCY,
            JOINT_SPRING_DAMPING_RATIO,
        ));
    joint.0.data.set_limits(JointAxis::LinX, [0.0, length]);
    joint.0.data.set_motor_position(
        JointAxis::LinX,
        length,
        MIN_BOUND_STIFFNESS,
        MIN_BOUND_DAMPING,
    );
    let handle = impulse_joint_set.insert(*point_handle_0, *point_handle_1, joint.build(), true);
    let damp_handle = impulse_joint_set.insert(
        *point_handle_0,
        *point_handle_1,
        GenericJointBuilder::new(JointAxesMask::empty())
            .softness(SpringCoefficients::new(
                JOINT_SPRING_FREQUENCY,
                JOINT_SPRING_DAMPING_RATIO,
            ))
            .build(),
        true,
    );

    let damp_joint = &mut impulse_joint_set.get_mut(damp_handle, false).unwrap().data;

    let damping_strength = 18.0;
    damp_joint.set_motor_velocity(JointAxis::LinX, 0.0, damping_strength);
    damp_joint.set_motor_velocity(JointAxis::LinY, 0.0, damping_strength);
    damp_joint.set_motor_velocity(JointAxis::LinZ, 0.0, damping_strength);

    (handle, damp_handle)
}

fn create_rope_body(
    sim_data: &mut SimulationSceneData,
    universal_drag: Real,
    coordinate: Vec3,
    point_radius: Real,
) -> RigidBodyHandle {
    let mut rigid_body = RigidBodyBuilder::dynamic()
        .translation(coordinate)
        .lock_rotations()
        .build();

    rigid_body.set_linear_damping(universal_drag);
    rigid_body.set_angular_damping(universal_drag);

    let handle = sim_data.rigid_body_set.insert(rigid_body);
    let collider = ColliderBuilder::new(SharedShape::cuboid(
        point_radius as Real,
        point_radius as Real,
        point_radius as Real,
    ))
    .friction(0.15)
    .mass(0.35)
    .collision_groups(ROPE_GROUP)
    .build();

    sim_data
        .collider_set
        .insert_with_parent(collider, handle, &mut sim_data.rigid_body_set);

    handle
}

/// Removes a rope
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_queryRope<'local>(
    env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jlong,
) -> JDoubleArray<'local> {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let strand = sable_data.rope_map.ropes.get(&(id as usize)).unwrap();

        let flattened: Vec<jdouble> = strand
            .points
            .iter()
            .flat_map(|x| {
                let pos = sim_data
                    .rigid_body_set
                    .get(*x)
                    .unwrap()
                    .position()
                    .translation;
                vec![pos.x as f64, pos.y as f64, pos.z as f64]
            })
            .collect();

        let double_array = env
            .new_double_array((strand.points.len() * 3) as jsize)
            .unwrap();
        env.set_double_array_region(&double_array, 0, &flattened)
            .unwrap();
        double_array
    })
}

/// Removes a rope
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeRope<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jlong,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let sim_data = &mut *sim_data;

        let strand = sable_data.rope_map.ropes.remove(&(id as usize)).unwrap();
        for handle in strand.points {
            sim_data.rigid_body_set.remove(
                handle,
                &mut sim_data.island_manager,
                &mut sim_data.collider_set,
                &mut sim_data.impulse_joint_set,
                &mut sim_data.multibody_joint_set,
                true,
            );
        }
    })
}

/// Sets the joint
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setRopeFirstSegmentLength<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jlong,
    length: jdouble,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let strand = sable_data.rope_map.ropes.get_mut(&(id as usize)).unwrap();

        strand.first_joint_length = length as Real;
        let first_joint = &mut sim_data
            .impulse_joint_set
            .get_mut(strand.joints.first().unwrap().0, true)
            .unwrap()
            .data;
        first_joint.set_limits(JointAxis::LinX, [0.0, length as Real]);
        first_joint.set_motor_position(
            JointAxis::LinX,
            length as Real,
            MIN_BOUND_STIFFNESS,
            MIN_BOUND_DAMPING,
        );
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_removeRopePointAtStart<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jlong,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let sim_data = &mut *sim_data;

        let strand = sable_data.rope_map.ropes.get_mut(&(id as usize)).unwrap();
        let point = strand.points.remove(0);
        strand.joints.remove(0);
        sim_data.rigid_body_set.remove(
            point,
            &mut sim_data.island_manager,
            &mut sim_data.collider_set,
            &mut sim_data.impulse_joint_set,
            &mut sim_data.multibody_joint_set,
            true,
        );

        let new_first_joint = &mut sim_data
            .impulse_joint_set
            .get_mut(strand.joints.first().unwrap().0, false)
            .unwrap()
            .data;
        new_first_joint.set_limits(JointAxis::LinX, [0.0, strand.first_joint_length]);
        new_first_joint.set_motor_position(
            JointAxis::LinX,
            strand.first_joint_length,
            MIN_BOUND_STIFFNESS,
            MIN_BOUND_DAMPING,
        );

        if strand.start_attachment.is_some() {
            if let Some(joint) = strand.start_attachment.take().and_then(|a| a.joint) {
                sim_data.impulse_joint_set.remove(joint, true);
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_addRopePointAtStart<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    id: jlong,
    x: jdouble,
    y: jdouble,
    z: jdouble,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let universal_drag = scene.universal_drag;

        let strand = sable_data.rope_map.ropes.get_mut(&(id as usize)).unwrap();
        let point_radius = strand.point_radius;

        // set joint that will no longer be the first
        let old_joint = &mut sim_data
            .impulse_joint_set
            .get_mut(strand.joints.first().unwrap().0, false)
            .unwrap()
            .data;
        old_joint.set_limits(JointAxis::LinX, [0.0, 1.0]);
        old_joint.set_motor_position(JointAxis::LinX, 1.0, MIN_BOUND_STIFFNESS, MIN_BOUND_DAMPING);

        let handle = create_rope_body(
            &mut sim_data,
            universal_drag,
            Vec3::new(x as Real, y as Real, z as Real),
            point_radius,
        );
        strand.joints.insert(
            0,
            add_rope_joint(
                &mut sim_data.impulse_joint_set,
                &handle,
                strand.points.first().unwrap(),
                strand.first_joint_length,
            ),
        );
        strand.points.insert(0, handle);

        if strand.start_attachment.is_some() {
            if let Some(joint) = strand.start_attachment.take().and_then(|a| a.joint) {
                sim_data.impulse_joint_set.remove(joint, true);
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_wakeUpRope<'local>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    rope_id: jlong,
) {
    with_handle(handle, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut universe = scene.universe.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let strand = sable_data.rope_map.ropes.get(&(rope_id as usize)).unwrap();

        for point in &strand.points {
            sim_data
                .rigid_body_set
                .get_mut(*point)
                .unwrap()
                .wake_up(true);
        }
    })
}

/// Sets the attachment at a given end
#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_setRopeAttachment<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    rope_id: jlong,
    sub_level_id: jint,
    x: jdouble,
    y: jdouble,
    z: jdouble,
    end: jboolean,
) {
    with_handle(handle, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim = scene.sim_data.write().unwrap();
        let SableSceneData {
            rope_map,
            rigid_bodies,
            level_colliders,
            ..
        } = &mut *sable_data;
        let SimulationSceneData {
            rigid_body_set,
            impulse_joint_set,
            ..
        } = &mut *sim;
        set_attachment(
            rope_map,
            impulse_joint_set,
            rope_id as usize,
            if sub_level_id == -1 {
                None
            } else {
                Some(sub_level_id as LevelColliderID)
            },
            DVec3::new(x, y, z),
            end > 0,
        );
        refresh_ropes(
            rope_map,
            scene.ground_handle,
            rigid_bodies,
            level_colliders,
            rigid_body_set,
            impulse_joint_set,
        );
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rapier3d::prelude::RigidBodySet;

    struct Fixture {
        ropes: RopeMap,
        bodies: HashMap<LevelColliderID, RigidBodyHandle>,
        colliders: HashMap<LevelColliderID, crate::ActiveLevelColliderInfo>,
        rigid_bodies: RigidBodySet,
        joints: ImpulseJointSet,
        ground: Option<RigidBodyHandle>,
    }

    impl Fixture {
        fn new() -> Self {
            let mut rigid_bodies = RigidBodySet::new();
            let ground = Some(rigid_bodies.insert(RigidBodyBuilder::fixed().build()));
            let points = (0..3)
                .map(|_| rigid_bodies.insert(RigidBodyBuilder::dynamic().build()))
                .collect();
            let mut ropes = RopeMap::default();
            ropes.ropes.insert(
                1,
                RopeStrand {
                    points,
                    joints: Vec::new(),
                    point_radius: 0.1,
                    first_joint_length: 1.0,
                    start_attachment: None,
                    end_attachment: None,
                },
            );
            Self {
                ropes,
                bodies: HashMap::new(),
                colliders: HashMap::new(),
                rigid_bodies,
                joints: ImpulseJointSet::new(),
                ground,
            }
        }
        fn attach(&mut self, id: usize, body: Option<usize>, end: bool) {
            set_attachment(
                &mut self.ropes,
                &mut self.joints,
                id,
                body,
                DVec3::new(4.0, 5.0, 6.0),
                end,
            );
            self.tick();
        }
        fn tick(&mut self) {
            refresh_ropes(
                &mut self.ropes,
                self.ground,
                &self.bodies,
                &self.colliders,
                &self.rigid_bodies,
                &mut self.joints,
            );
        }
        fn add_body(&mut self, id: usize) -> RigidBodyHandle {
            let handle = self
                .rigid_bodies
                .insert(RigidBodyBuilder::dynamic().build());
            self.bodies.insert(id, handle);
            let mut info = crate::ActiveLevelColliderInfo::new(None);
            info.center_of_mass = Some(DVec3::new(1.0, 2.0, 3.0));
            self.colliders.insert(id, info);
            handle
        }
        fn attachment(&self, end: bool) -> &RopeAttachment {
            let rope = &self.ropes.ropes[&1];
            (if end {
                &rope.end_attachment
            } else {
                &rope.start_attachment
            })
            .as_ref()
            .unwrap()
        }
    }

    #[test]
    fn missing_sublevel_is_pending_then_attaches_at_both_ends() {
        for end in [false, true] {
            let mut f = Fixture::new();
            // Issue #3: target exists outside this scene (or has not loaded yet).
            f.attach(1, Some(42), end);
            assert!(f.attachment(end).joint.is_none());
            assert_eq!(f.joints.len(), 0);
            f.tick();
            assert!(f.attachment(end).joint.is_none());
            let target = f.add_body(42);
            f.tick();
            let joint = f.joints.get(f.attachment(end).joint.unwrap()).unwrap();
            assert_eq!(joint.body1, target);
            assert_eq!(
                joint.body2,
                f.ropes.ropes[&1].points[if end { 2 } else { 0 }]
            );
            assert_eq!(joint.data.local_anchor1(), Vec3::new(3.0, 3.0, 3.0));
            let first = f.attachment(end).joint;
            f.tick();
            assert_eq!(f.attachment(end).joint, first);
            assert_eq!(f.joints.len(), 1);
        }
    }

    #[test]
    fn ground_attachment_works_without_sublevels_or_center_of_mass() {
        let mut f = Fixture::new();
        for end in [false, true] {
            f.attach(1, None, end);
            let joint = f.joints.get(f.attachment(end).joint.unwrap()).unwrap();
            assert_eq!(Some(joint.body1), f.ground);
            assert_eq!(joint.data.local_anchor1(), Vec3::new(4.0, 5.0, 6.0));
        }
        assert_eq!(f.joints.len(), 2);
    }

    #[test]
    fn missing_collider_or_center_of_mass_is_retried() {
        let mut f = Fixture::new();
        f.add_body(42);
        let mut info = f.colliders.remove(&42).unwrap();
        f.attach(1, Some(42), false);
        assert!(f.attachment(false).joint.is_none());
        info.center_of_mass = None;
        f.colliders.insert(42, info);
        f.tick();
        assert!(f.attachment(false).joint.is_none());
        f.colliders.get_mut(&42).unwrap().center_of_mass = Some(DVec3::ZERO);
        f.tick();
        assert!(f.attachment(false).joint.is_some());
    }

    #[test]
    fn replacing_attachment_removes_old_joint_and_preserves_other_end() {
        let mut f = Fixture::new();
        f.attach(1, None, false);
        f.attach(1, None, true);
        let old = f.attachment(false).joint.unwrap();
        let end = f.attachment(true).joint;
        f.attach(1, Some(42), false);
        assert!(!f.joints.contains(old));
        assert_eq!(f.joints.len(), 1);
        assert_eq!(f.attachment(true).joint, end);
        f.add_body(42);
        f.tick();
        assert_eq!(f.joints.len(), 2);
    }

    #[test]
    fn removed_target_or_joint_is_recovered_without_stale_handle_use() {
        let mut f = Fixture::new();
        f.add_body(42);
        f.attach(1, Some(42), false);
        let old = f.attachment(false).joint.unwrap();
        f.bodies.remove(&42);
        f.tick();
        assert!(!f.joints.contains(old));
        assert!(f.attachment(false).joint.is_none());
        let target = f.add_body(42);
        f.tick();
        let joint = f.attachment(false).joint.unwrap();
        assert_eq!(f.joints.get(joint).unwrap().body1, target);
        f.joints.remove(joint, true);
        f.tick();
        assert_eq!(f.joints.len(), 1);
        assert!(f.joints.contains(f.attachment(false).joint.unwrap()));
    }

    #[test]
    fn missing_rope_empty_points_and_missing_ground_do_not_panic() {
        let mut f = Fixture::new();
        f.attach(999, Some(42), false);
        assert_eq!(f.joints.len(), 0);
        f.ground = None;
        f.attach(1, None, false);
        assert!(f.attachment(false).joint.is_none());
        f.ground = Some(f.rigid_bodies.insert(RigidBodyBuilder::fixed().build()));
        f.ropes.ropes.get_mut(&1).unwrap().points.clear();
        f.tick();
        assert!(f.attachment(false).joint.is_none());
    }
}

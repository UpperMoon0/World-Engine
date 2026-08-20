use jni::JNIEnv;
use jni::objects::JClass;
use jni::sys::{jdouble, jint, jlong};
use marten::Real;

use crate::with_handle;

/// Global spring frequency for joints (Hz)
pub const JOINT_SPRING_FREQUENCY: Real = 550.0;

/// Global damping ratio for joints
pub const JOINT_SPRING_DAMPING_RATIO: Real = 4.0;

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_configFrequencyAndDamping<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    collision_natural_frequency: jdouble,
    collision_damping_ratio: jdouble,
) {
    with_handle(handle, |scene| {
        let mut sim = scene.sim_data.write().unwrap();
        sim.integration_parameters
            .contact_softness
            .natural_frequency = collision_natural_frequency as Real;
        sim.integration_parameters.contact_softness.damping_ratio = collision_damping_ratio as Real;
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_configSolverIterations<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    num_solver_iterations: jint,
    num_internal_pgs_iterations: jint,
    num_internal_stabilization_iterations: jint,
) {
    with_handle(handle, |scene| {
        let mut sim = scene.sim_data.write().unwrap();
        sim.integration_parameters.num_solver_iterations = num_solver_iterations as usize;
        sim.integration_parameters.num_internal_pgs_iterations =
            num_internal_pgs_iterations as usize;
        sim.integration_parameters
            .num_internal_stabilization_iterations = num_internal_stabilization_iterations as usize;
    });
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_com_nstut_worldengine_physics_rapier_Rapier3D_configMinIslandSize<
    'local,
>(
    _env: JNIEnv<'local>,
    _class: JClass<'local>,
    handle: jlong,
    island_size: jint,
) {
    with_handle(handle, |scene| {
        scene
            .sim_data
            .write()
            .unwrap()
            .integration_parameters
            .min_island_size = island_size as usize;
    });
}

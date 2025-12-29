use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};

use crate::{
    agent::{SteeringAgent, SteeringLocomotionMode},
    control::CombinedSteeringTarget,
    speed::SpeedMask,
};

const DANGER_SENSITIVITY: f32 = 0.05;

#[derive(QueryData)]
#[query_data(mutable)]
pub(crate) struct MoveAgentSystemQuery {
    agent: &'static SteeringAgent,
    forces: Forces,
    global_transform: &'static GlobalTransform,
    mass: &'static ComputedMass,
    moment: &'static ComputedAngularInertia,
    target: &'static CombinedSteeringTarget,
    speed_mask: &'static SpeedMask,
}

pub(crate) fn move_agent(mut query: Query<MoveAgentSystemQuery>) {
    for mut query_item in query.iter_mut() {
        let target_heading = query_item.target.into_heading(DANGER_SENSITIVITY);
        let target_speed = query_item.speed_mask.get_speed(target_heading);
        let target_velocity = query_item.agent.max_speed * target_speed * target_heading;
        let agent_forward = query_item.global_transform.forward();
        let agent_right = query_item.global_transform.right();
        let current_velocity = query_item.forces.linear_velocity();
        let velocity_error = target_velocity - current_velocity;
        let mass = query_item.mass.value();

        let wn = 8.0; // natural frequency (TODO: allow tuning this)
        let k = wn * wn * mass;
        let mut force = velocity_error * k;

        if let SteeringLocomotionMode::Directional = query_item.agent.locomotion_mode {
            let mut forward_force = force.dot(agent_forward.into());
            let lateral_force = force.dot(agent_right.into());
            // Use the lateral demand to brake if we need to turn, i.e., if
            // 80% of the force is lateral, don't barrel forward without turning
            let total_demand = forward_force.abs() + lateral_force.abs();
            let lateral_demand = if total_demand > f32::EPSILON {
                lateral_force / total_demand
            } else {
                0.0
            };
            let forward_suppression = 1.0 - lateral_demand * query_item.agent.turn_priority;
            forward_force *= forward_suppression;

            let lateral_velocity = current_velocity.dot(Vec3::from(agent_right));
            let lateral_impulse = -lateral_velocity * Vec3::from(agent_right) * mass;

            let angular_velocity = query_item.forces.angular_velocity();
            // Compute 3D angular error using cross product
            // For small angles, cross product ≈ axis * angle
            let angle_error_vec = Dir3::new(target_heading)
                .map(|target_dir| agent_forward.cross(*target_dir))
                .unwrap_or(Vec3::ZERO);
            let moment_diag = query_item.moment.value().diagonal();
            let wn_angular = 6.0;
            let k_angular = wn_angular * wn_angular * moment_diag;
            let c_angular = 2.0 * wn_angular * moment_diag;
            let max_torque = query_item.agent.max_torque;
            let torque = k_angular * angle_error_vec - c_angular * angular_velocity;
            let torque = torque.clamp(-max_torque, max_torque);
            let alignment = agent_forward.dot(target_velocity.normalize_or_zero());
            if alignment >= 0.0 {
                forward_force *= alignment;
            }

            query_item.forces.apply_linear_impulse(lateral_impulse);
            query_item.forces.apply_torque(torque);
            force = forward_force * Vec3::from(agent_forward);
        }

        force = force.clamp(-query_item.agent.max_force, query_item.agent.max_force);
        query_item.forces.apply_force(force);
    }
}

/// Debug visualization for agent movement. Shows the target heading,
/// current velocity, and velocity error (last_error) as colored arrows.
pub(crate) fn debug_movement(
    mut gizmos: Gizmos,
    query: Query<(
        &GlobalTransform,
        &SteeringAgent,
        &CombinedSteeringTarget,
        &LinearVelocity,
    )>,
) {
    for (transform, agent, target, velocity) in query.iter() {
        let position = transform.translation();
        let target_heading = target.into_heading(DANGER_SENSITIVITY);
        let current_velocity = **velocity;

        // Draw target heading (cyan arrow)
        let target_end = position + target_heading * agent.max_speed;
        gizmos.arrow(position, target_end, Color::srgb(0.0, 1.0, 1.0));

        // Draw current velocity (green arrow)
        let velocity_end = position + current_velocity;
        gizmos.arrow(position, velocity_end, Color::srgb(0.0, 1.0, 0.0));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::{
        MinimalPlugins,
        asset::{AssetEvent, AssetPlugin, Assets},
        prelude::{App, Mesh, Transform, Vec3},
        scene::ScenePlugin,
        transform::TransformPlugin,
    };

    fn basic_agent(position: Vec3, vel: Vec3, _target: Vec3) -> impl Bundle {
        let agent = SteeringAgent::default()
            .with_max_force(Vec3::splat(100.0))
            .with_max_torque(Vec3::splat(100.0));
        (
            Transform::from_translation(position),
            LinearVelocity::from(vel),
            RigidBody::Dynamic,
            GravityScale::from(0.0),
            Collider::sphere(1.0),
            agent,
        )
    }

    fn run_app_test<T>(setup: impl FnOnce(&mut App) -> T) -> (App, T) {
        let mut app = App::new();

        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            ScenePlugin,
            TransformPlugin,
            // Force the physics onto the Update schedule during tests
            PhysicsPlugins::new(Update),
        ));

        // Initialize required events and resources
        app.init_resource::<Assets<Mesh>>();
        app.add_message::<AssetEvent<Mesh>>();

        let result = setup(&mut app);

        for _ in 0..2 {
            app.finish();
            app.cleanup();
            app.update();
        }

        (app, result)
    }

    #[test]
    fn test_linear_agent_movement() {}

    #[test]
    fn test_directional_agent_movement() {}
}

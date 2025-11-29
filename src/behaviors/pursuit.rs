use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// Pursuit behavior that predicts the future position of a moving target
/// and steers toward it. More effective than simple seek when chasing
/// moving targets. Supports offset pursuit for fly-by maneuvers.
#[derive(Component, Debug, Copy, Clone, Reflect)]
pub struct Pursuit {
    /// The entity being pursued
    target: Entity,
    /// Maximum time to predict ahead (in seconds)
    max_prediction_time: f32,
    /// Radius within which to stop pursuing (arrival threshold)
    radius: f32,
    /// Offset radius for offset pursuit (0.0 = direct pursuit)
    offset: f32,
}

impl Pursuit {
    /// Create a new Pursuit behavior targeting the specified entity
    pub fn new(target: Entity) -> Self {
        Self {
            target,
            max_prediction_time: 1.0,
            radius: 1.0,
            offset: 0.0,
        }
    }

    /// Set the maximum prediction time. The pursuit prediction won't
    /// look further than this. (Default: 1.0)
    pub fn with_max_prediction(self, max_prediction_time: f32) -> Self {
        Self {
            max_prediction_time,
            ..self
        }
    }

    /// Set the radius within which to stop pursuing. (Default: 1.0)
    pub fn with_radius(self, radius: f32) -> Self {
        Self { radius, ..self }
    }

    /// Set the offset radius for offset pursuit (fly-by). (Default: 0.0)
    /// When set to a non-zero value, the pursuer will aim for a point
    /// offset from the target's predicted position, useful for fly-by
    /// maneuvers or maintaining a safe distance.
    pub fn with_offset(self, offset: f32) -> Self {
        Self { offset, ..self }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct PursuitBehaviorAgentQuery {
    entity: Entity,
    agent: &'static SteeringAgent,
    pursuit: &'static Pursuit,
    global_transform: &'static GlobalTransform,
    linear_velocity: &'static LinearVelocity,
    outputs: &'static mut SteeringOutputs,
}

/// Pursuit behavior predicts where the target will be based on distance
/// and the target's velocity, then steers toward that predicted position.
pub(crate) fn run(
    mut agent_query: Query<PursuitBehaviorAgentQuery>,
    target_query: Query<(&GlobalTransform, &LinearVelocity)>,
) {
    // Collect all pursuers first to avoid borrow checker issues
    let pursuers: Vec<_> = agent_query
        .iter()
        .map(|item| {
            (
                item.entity,
                item.pursuit.target,
                item.global_transform.translation(),
                item.agent.max_speed,
                item.pursuit.max_prediction_time,
                item.pursuit.radius,
                item.pursuit.offset,
            )
        })
        .collect();

    // Process each pursuer
    for (entity, target_entity, agent_pos, max_speed, max_prediction, arrival_radius, offset) in
        pursuers
    {
        // Look up the target's transform and velocity
        let Ok((target_transform, target_velocity)) = target_query.get(target_entity) else {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Pursuit);
            }
            continue;
        };

        let target_pos = target_transform.translation();
        let target_velocity = **target_velocity;

        let to_target = target_pos - agent_pos;
        let distance = to_target.length();

        // Stop pursuing when within arrival radius
        if distance < arrival_radius {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Pursuit);
            }
            continue;
        }

        // Simple prediction: T = distance / max_speed
        // T is larger when far from target, smaller when near
        let prediction_time = if max_speed > 0.001 {
            (distance / max_speed).min(max_prediction)
        } else {
            max_prediction
        };

        // Predict target's future position
        let mut predicted_pos = target_pos + target_velocity * prediction_time;

        // Apply offset for offset pursuit (fly-by maneuver)
        if offset.abs() > 0.001 {
            // Calculate perpendicular offset direction on the horizontal plane
            let to_predicted = predicted_pos - agent_pos;
            let horizontal_dir = Vec3::new(to_predicted.x, 0.0, to_predicted.z).normalize_or_zero();

            if horizontal_dir.length_squared() > 0.001 {
                // Get perpendicular direction (rotate 90 degrees around Y axis)
                let perpendicular = Vec3::new(-horizontal_dir.z, 0.0, horizontal_dir.x);
                // Apply offset to create fly-by point
                predicted_pos += perpendicular * offset;
            }
        }

        // Steer toward predicted (and possibly offset) position
        let to_predicted = predicted_pos - agent_pos;
        let direction = to_predicted.normalize_or_zero();

        if direction.length_squared() < 0.001 {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Pursuit);
            }
            continue;
        }

        if let Ok(mut item) = agent_query.get_mut(entity) {
            let mut steering_target = item.outputs.get(BehaviorType::Pursuit).unwrap_or_default();
            steering_target.set_interest(direction);
            item.outputs.set(BehaviorType::Pursuit, steering_target);
        }
    }
}

pub(crate) fn debug_pursuit(
    mut gizmos: Gizmos,
    pursuer_query: Query<(&GlobalTransform, &Pursuit)>,
    target_query: Query<&GlobalTransform>,
) {
    for (pursuer_transform, pursuit) in pursuer_query.iter() {
        let pursuer_pos = pursuer_transform.translation();

        // Draw pursuit radius circle
        let segments = 32;
        for i in 0..segments {
            let angle1 = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let angle2 = ((i + 1) as f32 / segments as f32) * std::f32::consts::TAU;
            let p1 = pursuer_pos
                + Vec3::new(
                    angle1.cos() * pursuit.radius,
                    0.0,
                    angle1.sin() * pursuit.radius,
                );
            let p2 = pursuer_pos
                + Vec3::new(
                    angle2.cos() * pursuit.radius,
                    0.0,
                    angle2.sin() * pursuit.radius,
                );
            gizmos.line(p1, p2, Color::srgb(0.3, 1.0, 0.3).with_alpha(0.3));
        }

        // Draw line to target with arrow
        if let Ok(target_transform) = target_query.get(pursuit.target) {
            let target_pos = target_transform.translation();

            // Line from pursuer to target
            gizmos.line(
                pursuer_pos,
                target_pos,
                Color::srgb(1.0, 1.0, 0.0).with_alpha(0.4),
            );

            // Arrow head at target
            let direction = (target_pos - pursuer_pos).normalize_or_zero();
            let arrow_length = 0.5;
            let arrow_angle = 0.3;

            let right = Quat::from_rotation_y(arrow_angle) * (-direction) * arrow_length;
            let left = Quat::from_rotation_y(-arrow_angle) * (-direction) * arrow_length;

            gizmos.line(
                target_pos,
                target_pos + right,
                Color::srgb(1.0, 1.0, 0.0).with_alpha(0.6),
            );
            gizmos.line(
                target_pos,
                target_pos + left,
                Color::srgb(1.0, 1.0, 0.0).with_alpha(0.6),
            );
        }
    }
}

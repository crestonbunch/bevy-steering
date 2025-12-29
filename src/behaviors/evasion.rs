use avian3d::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, query::QueryData, world::DeferredWorld},
    prelude::*,
};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    SMALL_THRESHOLD,
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// Evasion behavior that predicts the future position of a pursuing target
/// and steers away from it. The evasive counterpart to Pursuit.
#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[derivative(Default)]
#[component(on_remove = on_evasion_remove)]
pub struct Evasion {
    /// The entity to evade from
    pub target: Option<Entity>,
    /// Maximum time to predict ahead (in seconds)
    #[derivative(Default(value = "1.0"))]
    pub max_prediction_time: f32,
    /// Radius within which to evade (only evade when threat is predicted to
    /// get closer than this)
    #[derivative(Default(value = "f32::MAX"))]
    pub radius: f32,
}

impl Evasion {
    /// Create a new Evasion behavior targeting the specified entity
    pub fn new(target: Entity) -> Self {
        Self {
            target: Some(target),
            ..Default::default()
        }
    }

    /// Set the maximum prediction time. (Default: 1.0)
    pub fn with_max_prediction(self, max_prediction_time: f32) -> Self {
        Self {
            max_prediction_time,
            ..self
        }
    }

    /// Set the radius within which to evade. (Default: f32::MAX)
    /// Only evades when the target is within this distance.
    pub fn with_radius(self, radius: f32) -> Self {
        Self { radius, ..self }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct EvasionBehaviorAgentQuery {
    entity: Entity,
    agent: &'static SteeringAgent,
    evasion: &'static Evasion,
    global_transform: &'static GlobalTransform,
    outputs: &'static mut SteeringOutputs,
}

/// Evasion behavior predicts where the pursuer will be and steers away from
/// that predicted position.
pub(crate) fn run(
    mut agent_query: Query<EvasionBehaviorAgentQuery>,
    target_query: Query<(&GlobalTransform, &LinearVelocity)>,
) {
    // Collect all evading agents first to avoid borrow checker issues
    let evaders: Vec<_> = agent_query
        .iter()
        .map(|item| {
            (
                item.entity,
                item.evasion.target,
                item.global_transform.translation(),
                item.agent.max_speed,
                item.evasion.max_prediction_time,
                item.evasion.radius,
            )
        })
        .collect();

    // Process each evader
    for (entity, target_entity, agent_pos, max_speed, max_prediction, evasion_radius) in evaders {
        // Look up the threat's transform and velocity
        let Some(target_entity) = target_entity else {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Evasion);
            }
            continue;
        };
        let Ok((target_transform, target_velocity)) = target_query.get(target_entity) else {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Evasion);
            }
            continue;
        };

        let target_pos = target_transform.translation();
        let target_velocity = **target_velocity;

        let to_target = target_pos - agent_pos;
        let distance = to_target.length();

        // Simple prediction: T = distance / max_speed
        // T is larger when far from threat, smaller when near
        let prediction_time = if max_speed > SMALL_THRESHOLD {
            (distance / max_speed).min(max_prediction)
        } else {
            max_prediction
        };

        // Predict threat's future position
        let predicted_pos = target_pos + target_velocity * prediction_time;

        // Only evade if within evasion radius
        let predicted_distance = (predicted_pos - agent_pos).length();
        if predicted_distance >= evasion_radius {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Evasion);
            }
            continue;
        }

        // Flee away from predicted position
        let from_predicted = agent_pos - predicted_pos;
        let direction = from_predicted.normalize_or_zero();

        if direction.length_squared() < SMALL_THRESHOLD {
            if let Ok(mut item) = agent_query.get_mut(entity) {
                item.outputs.clear(BehaviorType::Evasion);
            }
            continue;
        }

        if let Ok(mut item) = agent_query.get_mut(entity) {
            let mut steering_target = item.outputs.get(BehaviorType::Evasion).unwrap_or_default();
            steering_target.set_interest(direction);
            item.outputs.set(BehaviorType::Evasion, steering_target);
        }
    }
}

pub(crate) fn debug_evasion(
    mut gizmos: Gizmos,
    evader_query: Query<(&GlobalTransform, &Evasion)>,
    target_query: Query<&GlobalTransform>,
) {
    for (evader_transform, evasion) in evader_query.iter() {
        let evader_pos = evader_transform.translation();

        // Draw evasion radius circle (if not MAX)
        if evasion.radius < f32::MAX {
            let segments = 32;
            for i in 0..segments {
                let angle1 = (i as f32 / segments as f32) * std::f32::consts::TAU;
                let angle2 = ((i + 1) as f32 / segments as f32) * std::f32::consts::TAU;
                let p1 = evader_pos
                    + Vec3::new(
                        angle1.cos() * evasion.radius,
                        0.0,
                        angle1.sin() * evasion.radius,
                    );
                let p2 = evader_pos
                    + Vec3::new(
                        angle2.cos() * evasion.radius,
                        0.0,
                        angle2.sin() * evasion.radius,
                    );
                gizmos.line(p1, p2, Color::srgb(1.0, 0.5, 0.0).with_alpha(0.3));
            }
        }

        let Some(target_entity) = evasion.target else {
            continue;
        };

        // Draw line from evader to threat with arrow pointing away
        if let Ok(target_transform) = target_query.get(target_entity) {
            let target_pos = target_transform.translation();

            // Line from threat to evader (showing evasion)
            gizmos.line(
                target_pos,
                evader_pos,
                Color::srgb(1.0, 0.3, 0.0).with_alpha(0.4),
            );

            // Arrow head at evader pointing away from threat
            let direction = (evader_pos - target_pos).normalize_or_zero();
            let arrow_length = 0.5;
            let arrow_angle = 0.3;

            let right = Quat::from_rotation_y(arrow_angle) * direction * arrow_length;
            let left = Quat::from_rotation_y(-arrow_angle) * direction * arrow_length;

            gizmos.line(
                evader_pos,
                evader_pos + right,
                Color::srgb(1.0, 0.3, 0.0).with_alpha(0.6),
            );
            gizmos.line(
                evader_pos,
                evader_pos + left,
                Color::srgb(1.0, 0.3, 0.0).with_alpha(0.6),
            );
        }
    }
}

fn on_evasion_remove(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    if let Some(mut outputs) = world.get_mut::<SteeringOutputs>(entity) {
        outputs.clear(BehaviorType::Evasion);
    }
}

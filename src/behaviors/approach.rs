use avian3d::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, query::QueryData, world::DeferredWorld},
    prelude::*,
};
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
    speed::SpeedOverride,
};

/// Approach behavior attempts to move the agent towards a target position.
/// Similar to Seek, except the agent slows down as it gets closer to the target.
#[derive(Component, Debug, Copy, Clone, Reflect)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[component(on_remove = on_approach_remove)]
pub struct Approach {
    /// The radius within which the agent will approach the target.
    pub target_radius: f32,
    /// The target position to approach.
    pub target: Vec3,
    /// Distance at which the agent begins slowing down when approaching at max speed.
    /// At this distance, speed = 1.0; closer distances result in proportionally lower speeds.
    pub slowdown_distance: f32,
}

impl Default for Approach {
    fn default() -> Self {
        Self {
            target_radius: 0.0,
            target: Vec3::ZERO,
            slowdown_distance: 10.0,
        }
    }
}

impl Approach {
    pub fn new(target: Vec3, target_radius: f32) -> Self {
        Self {
            target,
            target_radius,
            ..Default::default()
        }
    }

    ///  Set the slowdown distance for the approach behavior. Depending on
    ///  the agent's max speed and force, you may need to increase or
    ///  decrease this to avoid overshooting the target.
    pub fn with_slowdown_distance(mut self, distance: f32) -> Self {
        self.slowdown_distance = distance;
        self
    }

    pub fn set_target(&mut self, target: Vec3) {
        self.target = target;
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct ApproachBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    approach: &'static Approach,
    global_transform: &'static GlobalTransform,
    forces: Forces,
    outputs: &'static mut SteeringOutputs,
    speed_override: &'static mut SpeedOverride,
}

/// Approach behavior moves the agent towards the target position. At
/// far distances it behaves the same as [Seek], but slows down as
/// it approaches the target using kinematic arrival.
pub(crate) fn run(mut query: Query<ApproachBehaviorAgentQuery>) {
    for mut item in query.iter_mut() {
        let to_target = item.approach.target - item.global_transform.translation();
        let distance = to_target.length();

        // If we've arrived, express no interest in movement
        if distance < item.approach.target_radius {
            item.outputs.clear(BehaviorType::Approach);
            continue;
        }

        // Constant deceleration as the agent approaches the target.
        let speed = (distance / item.approach.slowdown_distance)
            .sqrt()
            .clamp(0.0, 1.0);

        let mut steering_target = item.outputs.get(BehaviorType::Approach).unwrap_or_default();
        steering_target.set_interest(to_target.normalize());
        item.outputs.set(BehaviorType::Approach, steering_target);
        item.speed_override.set(speed);
    }
}

/// Debug visualization for approach behavior. Draws a yellow arrow to the
/// target and a yellow circle around the target radius.
pub(crate) fn debug_approach(mut gizmos: Gizmos, query: Query<(&GlobalTransform, &Approach)>) {
    for (transform, approach) in query.iter() {
        let position = transform.translation();
        let yellow = Color::srgb(1.0, 1.0, 0.0);

        // Draw arrow to target
        gizmos.arrow(position, approach.target, yellow);

        // Draw circle around target radius
        if approach.target_radius > 0.0 {
            gizmos.circle(
                Isometry3d::from_translation(approach.target),
                approach.target_radius,
                yellow,
            );
        }
    }
}

fn on_approach_remove(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    if let Some(mut outputs) = world.get_mut::<SteeringOutputs>(entity) {
        outputs.clear(BehaviorType::Approach);
    }
}

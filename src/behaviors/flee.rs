use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use derivative::Derivative;

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[derivative(Default)]
pub struct Flee {
    #[derivative(Default(value = "f32::MAX"))]
    radius: f32,
    target: Vec3,
}

impl Flee {
    pub fn new(target: Vec3) -> Self {
        Self {
            target,
            ..Default::default()
        }
    }

    pub fn with_radius(self, radius: f32) -> Self {
        Self { radius, ..self }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct FleeBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    flee: &'static Flee,
    global_transform: &'static GlobalTransform,
    forces: Forces,
    outputs: &'static mut SteeringOutputs,
}

/// Flee behavior steers the agent away from the target position.
/// Only flees when within the specified radius.
pub(crate) fn run(mut query: Query<FleeBehaviorAgentQuery>) {
    for mut item in query.iter_mut() {
        let agent_pos = item.global_transform.translation();
        let target_pos = item.flee.target;
        let to_target = target_pos - agent_pos;
        let distance = to_target.length();

        // If we're far enough away, express no interest in movement
        if distance >= item.flee.radius {
            item.outputs.clear(BehaviorType::Flee);
            return;
        }

        let mut steering_target = item.outputs.get(BehaviorType::Flee).unwrap_or_default();
        // Flee away from the target (opposite direction of seek)
        steering_target.set_interest(-to_target.normalize());
        item.outputs.set(BehaviorType::Flee, steering_target);
    }
}


use avian3d::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, query::QueryData, world::DeferredWorld},
    prelude::*,
};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// Flee behavior attempts to put distance between the agent and a target position.
/// Essentially the opposite of Seek.
#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[derivative(Default)]
#[component(on_remove = on_flee_remove)]
pub struct Flee {
    /// The radius within which the agent will flee from the target.
    #[derivative(Default(value = "f32::MAX"))]
    pub radius: f32,
    /// The target position to flee from.
    pub target: Vec3,
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

fn on_flee_remove(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    if let Some(mut outputs) = world.get_mut::<SteeringOutputs>(entity) {
        outputs.clear(BehaviorType::Flee);
    }
}

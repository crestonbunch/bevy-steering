use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// Seek behavior moves the agent towards the target position. Does not
/// slow down on approach (see Approach for a smoother approach.)
#[derive(Component, Debug, Default, Copy, Clone, Reflect)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct Seek {
    /// The radius within which the agent will consider itself to be at the target.
    pub target_radius: f32,
    /// The position the agent is trying to reach.
    pub target: Vec3,
}

impl Seek {
    pub fn new(target: Vec3, target_radius: f32) -> Self {
        Self {
            target,
            target_radius,
        }
    }

    pub fn set_target(&mut self, target: Vec3) {
        self.target = target;
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct SeekBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    seek: &'static Seek,
    global_transform: &'static GlobalTransform,
    forces: Forces,
    outputs: &'static mut SteeringOutputs,
}

/// Seek behavior moves the agent towards the target position. Does not
/// slow down on approach.
pub(crate) fn run(mut query: Query<SeekBehaviorAgentQuery>) {
    for mut item in query.iter_mut() {
        let agent_pos = item.global_transform.translation();
        let target_pos = item.seek.target;
        let to_target = target_pos - agent_pos;
        let distance = to_target.length();

        // If we've arrived, express no interest in movement
        if distance < item.seek.target_radius {
            item.outputs.clear(BehaviorType::Seek);
            return;
        }

        let mut steering_target = item.outputs.get(BehaviorType::Seek).unwrap_or_default();
        steering_target.set_interest(to_target.normalize());
        item.outputs.set(BehaviorType::Seek, steering_target);
    }
}

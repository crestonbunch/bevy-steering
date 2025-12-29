use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs, SteeringTarget},
    prelude::{NearbyObstacles, TrackNearbyObstacles},
};

/// Avoid obstacles. This is not a replacement for navigation
/// (e.g. A* or similar.) An agent can get stuck even with this
/// behavior. Use this behavior so that the agent routes around
/// obstacles that are in the way. Configure obstacle detection
/// by adding a [TrackNearbyObstacles] component, otherwise it uses
/// a default configuration.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[require(TrackNearbyObstacles)]
pub struct Avoid {
    /// Only avoid obstacles whose nearest point is at most this distance away.
    #[derivative(Default(value = "1.0"))]
    pub distance: f32,
}

impl Avoid {
    pub fn with_distance(mut self, distance: f32) -> Self {
        self.distance = distance;
        self
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct AvoidBehaviorAgentQuery {
    entity: Entity,
    agent: &'static SteeringAgent,
    avoid: &'static Avoid,
    velocity: &'static LinearVelocity,
    global_transform: &'static GlobalTransform,
    obstacles: &'static NearbyObstacles,
    track: &'static TrackNearbyObstacles,
    outputs: &'static mut SteeringOutputs,
}

pub(crate) fn run(mut query: Query<AvoidBehaviorAgentQuery>) {
    for mut agent in query.iter_mut() {
        let mut target = SteeringTarget::default();

        for obstacle in agent.obstacles.values() {
            if obstacle.distance > agent.avoid.distance {
                continue;
            }
            let Some((agent_normal, impact_normal)) = obstacle.impact_normals else {
                continue;
            };
            let dir = -(agent_normal + impact_normal) / 2.0;
            target.set_danger(dir);
        }

        agent.outputs.set(BehaviorType::Avoid, target);
    }
}

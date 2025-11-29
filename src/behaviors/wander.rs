use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use rand::{Rng, SeedableRng, rngs::StdRng};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// Wander behavior creates random but smooth steering by projecting a sphere
/// ahead of the character and randomly walking a target point around its surface.
/// This produces more interesting, sustained turns compared to purely random steering.
#[derive(Component, Debug, Clone, Reflect)]
pub struct Wander {
    wander_distance: f32,
    wander_radius: f32,
    wander_rate: f32,
    /// Current angle on the wander circle (persisted state)
    wander_angle: f32,
}

impl Default for Wander {
    fn default() -> Self {
        Self {
            wander_distance: 2.0,
            wander_radius: 1.0,
            wander_rate: 0.5,
            wander_angle: 0.0,
        }
    }
}

impl Wander {
    /// Distance ahead of the character where the wander sphere is projected
    pub fn with_distance(mut self, distance: f32) -> Self {
        self.wander_distance = distance;
        self
    }

    /// Radius of the wander sphere (determines maximum wandering strength)
    pub fn with_radius(mut self, radius: f32) -> Self {
        self.wander_radius = radius;
        self
    }

    /// Maximum change in wander angle per frame (determines wander rate/jitter)
    pub fn with_rate(mut self, rate: f32) -> Self {
        self.wander_rate = rate;
        self
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct WanderBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    wander: &'static mut Wander,
    global_transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
    outputs: &'static mut SteeringOutputs,
}

/// Wander behavior creates smooth random steering by maintaining a target
/// point on a sphere projected ahead of the character. The target point
/// performs a random walk on the sphere's surface.
pub(crate) fn run(mut query: Query<WanderBehaviorAgentQuery>, mut rng: Local<Option<StdRng>>) {
    // Initialize RNG on first run using a simple seed
    let rng = rng.get_or_insert_with(|| StdRng::seed_from_u64(42));

    for mut item in query.iter_mut() {
        let agent_pos = item.global_transform.translation();
        let forward = item.global_transform.forward().as_vec3();

        // Add random displacement to the wander angle
        let displacement = rng.random_range(-item.wander.wander_rate..=item.wander.wander_rate);
        item.wander.wander_angle += displacement;

        // Calculate the wander target point
        // 1. Start with a point ahead of the character
        let circle_center = agent_pos + forward * item.wander.wander_distance;

        // 2. Calculate a point on the circle using the wander angle
        // We need to create a perpendicular vector to 'forward' for the circle
        let up = Vec3::Y;
        let right = forward.cross(up).normalize_or_zero();
        let actual_up = right.cross(forward).normalize_or_zero();

        // Create the displacement on the circle
        let circle_offset = (right * item.wander.wander_angle.cos()
            + actual_up * item.wander.wander_angle.sin())
            * item.wander.wander_radius;

        let wander_target = circle_center + circle_offset;

        // Steer towards the wander target
        let to_target = wander_target - agent_pos;
        let mut steering_target = item.outputs.get(BehaviorType::Wander).unwrap_or_default();
        steering_target.set_interest(to_target.normalize());
        item.outputs.set(BehaviorType::Wander, steering_target);
    }
}

use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::prelude::NearbyObstacles;

const NUM_SLOTS: usize = 16;

/// Maximum speed from 0.0-1.0 for each direction around the agent.
/// This is computed by the [SpeedController] to help avoid running
/// into obstacles.
#[derive(Component, Debug, Copy, Clone, Reflect, Default, Deref)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub(crate) struct SpeedMask([f32; NUM_SLOTS]);

impl SpeedMask {
    pub(crate) fn slot_to_dir(slot: usize) -> Vec3 {
        const ANGLE_PER_SLOT: f32 = 2.0 * PI / NUM_SLOTS as f32;
        let angle = slot as f32 * ANGLE_PER_SLOT;
        Vec3::new(angle.cos(), 0.0, angle.sin())
    }

    pub(crate) fn subtract_speed(&mut self, dir: Vec3, amount: f32) {
        for i in 0..NUM_SLOTS {
            let slot_dir = Self::slot_to_dir(i);
            let factor = slot_dir.dot(dir).max(0.0);
            self.0[i] = (self.0[i] - amount * factor).max(0.0);
        }
    }

    pub(crate) fn get_speed(&self, dir: Vec3) -> f32 {
        const ANGLE_PER_SLOT: f32 = 2.0 * PI / NUM_SLOTS as f32;

        // Get the angle from the direction vector
        let angle = dir.z.atan2(dir.x);
        // Normalize to [0, 2π)
        let angle = if angle < 0.0 { angle + 2.0 * PI } else { angle };

        // Find fractional slot position
        let slot_f = angle / ANGLE_PER_SLOT;
        let slot_low = slot_f.floor() as usize % NUM_SLOTS;
        let slot_high = (slot_low + 1) % NUM_SLOTS;
        let t = slot_f.fract();

        // Linearly interpolate between the two adjacent slots
        self.0[slot_low] * (1.0 - t) + self.0[slot_high] * t
    }
}

/// Set the desired maximum speed as a fraction from [0.0-1.0]. This
/// will act as a global speed limit by anyone that sets it. If multiple
/// systems try to set this, only the minimum is kept.
#[derive(Component, Debug, Copy, Clone, Reflect, Derivative, Deref)]
#[derivative(Default)]
#[reflect(Component)]
#[require(SpeedMask)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct SpeedOverride(#[derivative(Default(value = "1.0"))] f32);

impl SpeedOverride {
    /// Set a new speed override value.
    pub fn set(&mut self, value: f32) {
        self.0 = self.0.min(value);
    }

    /// Reset the speed override to 1.0. This happens
    /// automatically during FixedPreUpdate.
    pub fn reset(&mut self) {
        self.0 = 1.0;
    }
}

/// The speed controller component. This component
/// manages the [TargetSpeed] of an entity, to avoid
/// crashing into obstacles.
#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[reflect(Component)]
#[require(SpeedMask, SpeedOverride)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct SpeedController {
    /// The weight of the obstacle avoidance behavior. If
    /// it's greater than 0.0, the agent will slow down
    /// near obstacles.
    #[derivative(Default(value = "1.0"))]
    pub obstacle_weight: f32,

    /// The distance at which the agent will start slowing down
    /// if it's in the direction of an obstacle.
    #[derivative(Default(value = "10.0"))]
    pub stopping_distance: f32,
}

impl SpeedController {
    pub fn with_obstacle_weight(mut self, weight: f32) -> Self {
        self.obstacle_weight = weight;
        self
    }

    pub fn with_stopping_distance(mut self, distance: f32) -> Self {
        self.stopping_distance = distance;
        self
    }
}

#[derive(QueryData)]
pub(crate) struct SpeedControllerQuery {
    entity: Entity,
    obstacles: Option<&'static NearbyObstacles>,
    speed_controller: &'static SpeedController,
    speed_override: &'static SpeedOverride,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

pub(crate) fn speed_control(query: Query<SpeedControllerQuery>, mut commands: Commands) {
    for item in query.iter() {
        let speed_override = item.speed_override;
        let mut speed_mask = SpeedMask([speed_override.0; NUM_SLOTS]);

        let Some(obstacles) = item.obstacles else {
            commands.entity(item.entity).insert(speed_mask);
            continue;
        };

        let reference_distance = item.speed_controller.stopping_distance;
        for obstacle in obstacles.values() {
            // Slow down if the direction is towards the obstacle
            let Some((agent_normal, impact_normal)) = obstacle.impact_normals else {
                continue;
            };
            let to_obstacle_dir = -(agent_normal + impact_normal) / 2.0;

            // Distance factor: how close is the obstacle?
            // 1.0 when at contact (distance=0), approaches 0.0 at reference_distance
            // Stops quickly using quadratic falloff
            let dist = obstacle.distance;
            let normalized_dist = (dist / reference_distance).clamp(0.0, 1.0);
            let threat = -normalized_dist.powi(2) + 1.0;
            let slowdown = item.speed_controller.obstacle_weight * threat;

            speed_mask.subtract_speed(to_obstacle_dir, slowdown);
            // speed_mask.subtract_speed(from_obstacle_dir, slowdown);
        }

        commands.entity(item.entity).insert(speed_mask);
    }
}

pub(crate) fn reset_speed_override(mut query: Query<&mut SpeedOverride>) {
    for mut speed_override in query.iter_mut() {
        speed_override.reset();
    }
}

pub(crate) fn debug_speed(mut gizmos: Gizmos, query: Query<(&GlobalTransform, &SpeedMask)>) {
    const BASE_LINE_LENGTH: f32 = 8.0;

    for (transform, speed_mask) in query.iter() {
        let agent_position = transform.translation();

        // Draw speed mask (yellow, offset slightly)
        for i in 0..NUM_SLOTS {
            let direction = SpeedMask::slot_to_dir(i);

            let mask_value = speed_mask[i];
            if mask_value > 0.01 {
                let danger_length = BASE_LINE_LENGTH * mask_value;
                let end_point = agent_position + direction * danger_length;
                let offset = Vec3::new(-0.1, 0.0, -0.1);
                gizmos.line(
                    agent_position + offset,
                    end_point + offset,
                    Color::srgb(1.0, 1.0, 0.0),
                );
            }
        }
    }
}

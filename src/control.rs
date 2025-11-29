use bevy::ecs::query::QueryData;
use enum_map::{Enum, EnumMap};
use itertools::Itertools;
use std::f32::consts::PI;

use bevy::prelude::*;
use derivative::Derivative;

const NUM_SLOTS: usize = 16;

/// Enum representing the different types of steering behaviors.
#[derive(Debug, Copy, Clone, Enum, Hash, PartialEq, Eq)]
pub enum BehaviorType {
    Alignment,
    Approach,
    Avoid,
    Cohere,
    Evasion,
    Flee,
    PathFollowing,
    Pursuit,
    Seek,
    Separation,
    Wander,
}

/// Insert a TemporalSmoothing resource to enable temporal smoothing.
/// The default is to have no temporal smoothing, but it may help for
/// some games. This blends the previous frame's behaviors into the next
/// frames by some fraction, which can reduce jitter when behaviors
/// frequently create ties for target directions.
#[derive(Resource, Debug, Default, Copy, Clone)]
pub struct TemporalSmoothing(f32);

impl TemporalSmoothing {
    /// Create a new TemporalSmoothing resource with the given blend factor.
    /// Blend should be in the range [0.0, 1.0], where 0.0 means no smoothing
    /// and 1.0 means full smoothing (only previous frame). A good starting
    /// value is around 0.2.
    pub fn new(blend: f32) -> Self {
        Self(blend.clamp(0.0, 1.0))
    }
}

#[derive(Component, Default, Debug, Copy, Clone, Deref, DerefMut)]
pub(crate) struct PreviousSteeringOutputs(SteeringOutputs);

/// Represents the outputs of the steering behaviors. This is used to
/// store the targets of the steering behaviors. They can be
/// combined to create a single target, which will be used to
/// move the agent.
#[derive(Component, Default, Debug, Copy, Clone)]
#[require(PreviousSteeringOutputs)]
pub struct SteeringOutputs {
    values: EnumMap<BehaviorType, Option<SteeringTarget>>,
}

impl SteeringOutputs {
    #[allow(dead_code)]
    pub(crate) fn get(&self, behavior: BehaviorType) -> Option<SteeringTarget> {
        self.values[behavior]
    }

    pub(crate) fn set(&mut self, behavior: BehaviorType, target: SteeringTarget) {
        self.values[behavior] = Some(target);
    }

    pub(crate) fn clear(&mut self, behavior: BehaviorType) {
        self.values[behavior] = None;
    }

    /// Returns a vector of behaviors with their targets that have been
    /// set by a system. (I.e., filters out None targets.)
    fn only_some(&self) -> Vec<(BehaviorType, SteeringTarget)> {
        self.values
            .iter()
            .filter_map(|(behavior, target)| target.map(|t| (behavior, t)))
            .collect()
    }

    /// Returns true if any of the behavior targets are set.
    fn has_some(&self) -> bool {
        self.values.iter().any(|(_, target)| target.is_some())
    }

    fn lerp(&self, other: &SteeringOutputs, blend: f32) -> SteeringOutputs {
        let mut result = SteeringOutputs::default();
        let keys = self
            .values
            .iter()
            .zip(other.values.iter())
            .flat_map(|((a, _), (b, _))| [a, b])
            .unique();
        for behavior in keys {
            let a = self.get(behavior).unwrap_or_default();
            let b = other.get(behavior).unwrap_or_default();
            result.set(behavior, a.lerp(&b, blend));
        }
        result
    }
}

/// Represents the target computed by a steering behavior using
/// context maps.
#[derive(Debug, Copy, Clone, PartialEq, Reflect, Derivative)]
#[derivative(Default)]
pub struct SteeringTarget {
    /// How the interest of the behavior is distributed.
    /// Split into 360° / N sectors (N = 16). The more
    /// a behavior wants to move along a heading, the
    /// higher the interest [0.0, 1.0] for that sector.
    pub(crate) interest_map: [f32; NUM_SLOTS],
    /// How the danger of the behavior is distributed.
    /// Split into 360° / N sectors (N = 16). This is
    /// used to mask interest to avoid dangerous
    /// directions when deciding a final heading.
    pub(crate) danger_map: [f32; NUM_SLOTS],
    /// How quickly [0.0-1.0] to move. Agent will
    /// try to reach its maximum velocity when
    /// speed is 1.0, and will stop at 0.0. The slowest
    /// behavior will dictate the speed.
    #[derivative(Default(value = "1.0"))]
    pub(crate) speed: f32,
    /// How urgent the agent needs to move. This can
    /// override the speed to force movement. The
    /// max urgency of all behaviors will be used.
    #[derivative(Default(value = "0.0"))]
    pub(crate) urgency: f32,
}

impl SteeringTarget {
    const ZERO: Self = Self {
        interest_map: [0.0; NUM_SLOTS],
        danger_map: [0.0; NUM_SLOTS],
        speed: 0.0,
        urgency: 0.0,
    };

    pub(crate) fn slot_to_dir(slot: usize) -> Vec3 {
        const ANGLE_PER_SLOT: f32 = 2.0 * PI / NUM_SLOTS as f32;
        let angle = slot as f32 * ANGLE_PER_SLOT;
        Vec3::new(angle.cos(), 0.0, angle.sin())
    }

    /// Interpolates around the peak slot using a local weighted average.
    /// Uses the interest values of the peak and its neighbors as weights
    /// to blend their directions, producing a smooth sub-slot direction.
    fn interpolate_peak(values: &[f32], peak_slot: usize) -> Vec3 {
        let mut sum_dir = Vec3::ZERO;
        let mut sum_weight = 0.0;

        // Consider the peak and its immediate neighbors
        for offset in -2i32..=2 {
            let n = NUM_SLOTS as i32;
            let slot = ((peak_slot as i32 + offset + n) % n) as usize;
            let weight = values[slot];
            sum_dir += weight * Self::slot_to_dir(slot);
            sum_weight += weight;
        }

        if sum_weight > f32::EPSILON {
            sum_dir.normalize_or_zero()
        } else {
            Self::slot_to_dir(peak_slot)
        }
    }

    /// Assigns weights to each direction based on the given interest vector.
    /// Uses the dot product of each slot's direction to produce the interest
    /// from [0.0-1.0] such that a slot exactly matching [direction] would
    /// be 1.0, and slots perpendicular would be 0.0. Anything less than 0.0
    /// is clamped to 0.0
    pub fn set_interest(&mut self, direction: Vec3) {
        for i in 0..NUM_SLOTS {
            let slot_dir = SteeringTarget::slot_to_dir(i);
            self.interest_map[i] = slot_dir.dot(direction).max(0.0);
        }
    }

    /// Assigns weights to each direction based on the given danger vector.
    /// Uses the same method as `set_interest` to produce the danger
    /// from [0.0-1.0] such that a slot exactly matching [direction] would
    /// be 1.0, and slots perpendicular would be 0.0. Anything less than 0.0
    /// is clamped to 0.0. Accumulates danger using max, so multiple calls
    /// will take the maximum danger value for each slot.
    pub fn add_danger(&mut self, direction: Vec3, intensity: f32) {
        for i in 0..NUM_SLOTS {
            let slot_dir = SteeringTarget::slot_to_dir(i);
            let new_danger = (slot_dir.dot(direction) * intensity).max(0.0);
            self.danger_map[i] = self.danger_map[i].max(new_danger);
        }
    }

    /// Sets the speed of the agent.
    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed.clamp(0.0, 1.0);
    }

    /// Sets the urgency of the agent.
    pub fn set_urgency(&mut self, urgency: f32) {
        self.urgency = urgency.clamp(0.0, 1.0);
    }

    /// Linearly interpolates between this steering target and another.
    /// Blends interest maps, danger maps, speed, and urgency.
    fn lerp(&self, other: &SteeringTarget, blend: f32) -> SteeringTarget {
        let mut result = SteeringTarget::default();

        // Lerp each slot in the interest and danger maps
        for i in 0..NUM_SLOTS {
            result.interest_map[i] = self.interest_map[i].lerp(other.interest_map[i], blend);
            result.danger_map[i] = self.danger_map[i].lerp(other.danger_map[i], blend);
        }

        // Lerp speed and urgency
        result.speed = self.speed.lerp(other.speed, blend);
        result.urgency = self.urgency.lerp(other.urgency, blend);

        result
    }
}

#[derive(Component, Debug, Copy, Clone)]
pub(crate) struct CombinedSteeringTarget(SteeringTarget);

impl CombinedSteeringTarget {
    const ZERO: Self = Self(SteeringTarget::ZERO);

    pub fn new(targets: impl Iterator<Item = SteeringTarget>) -> Self {
        let mut final_target = SteeringTarget::default();
        for target in targets {
            for i in 0..NUM_SLOTS {
                let target_interest = target.interest_map[i];
                let target_danger = target.danger_map[i];
                final_target.interest_map[i] = target_interest.max(final_target.interest_map[i]);
                final_target.danger_map[i] = target_danger.max(final_target.danger_map[i]);
                final_target.speed = final_target.speed.min(target.speed);
                final_target.urgency = final_target.urgency.max(target.urgency);
            }
        }
        CombinedSteeringTarget(final_target)
    }

    /// Convert the combined steering target into a target heading vector.
    /// This is normalized such that multiplying it by the agent's maximum
    /// veolcit will produce a heading vector that can be used for movement.
    /// Danger sensitivity allows for trading off danger vs. interest. The
    /// higher the sensitivity, the more likely a direction is unmasked. A
    /// good starting value is 0.05.
    pub fn into_heading(self, danger_sensitivity: f32) -> Vec3 {
        let inner = self.0;
        let min_danger = inner.danger_map.iter().fold(f32::MAX, |a, &b| a.min(b));
        let danger_threshold = min_danger + danger_sensitivity;
        let mask = inner
            .danger_map
            .into_iter()
            .map(|x| if x <= danger_threshold { 1.0 } else { 0.0 });
        let masked_interest = inner
            .interest_map
            .into_iter()
            .zip(mask)
            .map(|(x, y)| x * y)
            .collect::<Vec<_>>();
        let max_interest = masked_interest.iter().fold(f32::MIN, |a, &b| a.max(b));
        let target_slot = masked_interest
            .iter()
            .position(|x| *x >= max_interest)
            .unwrap_or(0);
        let heading = SteeringTarget::interpolate_peak(&masked_interest, target_slot);

        let speed = if inner.urgency > 0.0 {
            // Urgency overrides speed to force movement
            inner.speed.lerp(1.0, inner.urgency)
        } else {
            inner.speed
        };

        heading * speed
    }
}

#[derive(QueryData)]
pub(crate) struct CombinedSteeringTargetQuery {
    agent: Entity,
    steering_outputs: &'static SteeringOutputs,
}

pub(crate) fn combine_steering_targets(
    mut commands: Commands,
    query: Query<CombinedSteeringTargetQuery>,
) {
    for query_item in query.iter() {
        let steering_outputs = query_item.steering_outputs;
        let mut entity = commands.entity(query_item.agent);
        if steering_outputs.has_some() {
            let targets = steering_outputs
                .only_some()
                .into_iter()
                .map(|(_, target)| target);
            entity.insert(CombinedSteeringTarget::new(targets));
        } else {
            // If there are no steering targets, then the default behavior
            // should be stop moving by setting target speed to 0.0.
            entity.insert(CombinedSteeringTarget::ZERO);
        }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub(crate) struct TemporalSmoothingQuery {
    previous_outputs: &'static PreviousSteeringOutputs,
    outputs: &'static mut SteeringOutputs,
}

pub(crate) fn temporal_smoothing(
    mut query: Query<TemporalSmoothingQuery>,
    res_smoothing: Option<Res<TemporalSmoothing>>,
) {
    let Some(factor) = res_smoothing else {
        return;
    };

    for query_item in query.iter_mut() {
        let previous_outputs = query_item.previous_outputs;
        let mut outputs = query_item.outputs;
        *outputs = outputs.lerp(&previous_outputs.0, factor.0);
    }
}

pub(crate) fn update_previous_steering_outputs(
    mut query: Query<(&SteeringOutputs, &mut PreviousSteeringOutputs)>,
) {
    for (outputs, mut previous) in query.iter_mut() {
        previous.0 = *outputs;
    }
}

/// Debug visualization for combined steering targets. Draws interest and danger
/// maps as colored lines radiating from each agent, weighted by urgency/speed.
pub(crate) fn debug_combined_steering(
    mut gizmos: Gizmos,
    query: Query<(&GlobalTransform, &CombinedSteeringTarget)>,
) {
    const BASE_LINE_LENGTH: f32 = 2.0;

    for (transform, combined_target) in query.iter() {
        let agent_position = transform.translation();
        let target = &combined_target.0;

        // Calculate the weighting factor from urgency and speed
        // Urgency overrides speed when present
        let weight = if target.urgency > 0.0 {
            target.speed.lerp(1.0, target.urgency)
        } else {
            target.speed
        };

        // Draw interest map (green) and danger map (red) for each slot
        for i in 0..NUM_SLOTS {
            let direction = SteeringTarget::slot_to_dir(i);

            // Draw interest as green lines
            let interest_value = target.interest_map[i];
            if interest_value > 0.01 {
                let interest_length = BASE_LINE_LENGTH * interest_value * weight;
                let end_point = agent_position + direction * interest_length;
                gizmos.line(
                    agent_position,
                    end_point,
                    Color::srgb(0.0, 1.0, 0.0).with_alpha(interest_value * 0.8),
                );
            }

            // Draw danger as red lines
            let danger_value = target.danger_map[i];
            if danger_value > 0.01 {
                let danger_length = BASE_LINE_LENGTH * danger_value * weight;
                let end_point = agent_position + direction * danger_length;
                gizmos.line(
                    agent_position,
                    end_point,
                    Color::srgb(1.0, 0.0, 0.0).with_alpha(danger_value * 0.8),
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_slot_to_dir() {
        let test_cases = [
            (0, Vec3::new(1.0, 0.0, 0.0)),
            (4, Vec3::new(0.0, 0.0, 1.0)),
            (8, Vec3::new(-1.0, 0.0, 0.0)),
            (12, Vec3::new(0.0, 0.0, -1.0)),
        ];
        for (input, expect) in test_cases {
            let output = SteeringTarget::slot_to_dir(input);
            assert!(
                output.abs_diff_eq(expect, 0.0001),
                "Failed for input: {}",
                input
            );
        }
    }

    #[test]
    fn test_into_heading_surrounded_agent() {
        // Test the surrounded agent scenario where every direction has some danger.
        // The agent should choose the least dangerous direction with reasonable interest.
        let mut target = SteeringTarget::default();

        // All 16 slots have meaningful danger values to avoid minDanger = 0.0
        let danger = [
            0.8, 0.6, 0.3, 0.4, 0.5, 0.4, 0.3, 0.7, 0.9, 0.8, 0.6, 0.7, 0.8, 0.7, 0.6, 0.9,
        ];
        let interest = [
            0.9, 0.7, 0.4, 0.2, 0.1, 0.2, 0.5, 0.8, 0.3, 0.2, 0.1, 0.1, 0.2, 0.1, 0.2, 0.3,
        ];

        target.danger_map = danger;
        target.interest_map = interest;
        target.speed = 1.0;

        let combined = CombinedSteeringTarget(target);
        let heading = combined.into_heading(0.05);

        // With minimum-based masking:
        // minDanger = 0.3 (slots 2 and 6)
        // threshold = 0.3 + 0.05 = 0.35
        // Slots with danger <= 0.35: slots 2 (0.3) and 6 (0.3)
        // Their interests are: slot 2 = 0.4, slot 6 = 0.5
        // Winner should be slot 6 with the highest interest (0.5)

        // Slot 6 direction: angle = 6 * (2π / 16) = 3π/4 radians
        let expected_angle = 6.0 * (2.0 * PI / 16.0);
        let expected_direction = Vec3::new(expected_angle.cos(), 0.0, expected_angle.sin());

        // The heading should point roughly in the direction of slot 6
        // (with some interpolation from neighboring slots)
        assert!(
            heading.normalize().dot(expected_direction) > 0.9,
            "Expected heading to be close to slot 6 direction. Got: {:?}, expected direction: {:?}",
            heading,
            expected_direction
        );

        // Speed should be 1.0 since we set it to 1.0 and urgency is 0.0
        assert!(
            (heading.length() - 1.0).abs() < 0.1,
            "Expected speed close to 1.0, got: {}",
            heading.length()
        );
    }

    #[test]
    fn test_into_heading_clear_path() {
        // Test a simple case with one clear direction of high interest and no danger
        let mut target = SteeringTarget::default();

        // Slot 0 has high interest, no danger
        target.interest_map[0] = 1.0;
        target.danger_map[0] = 0.0;
        target.speed = 1.0;

        let combined = CombinedSteeringTarget(target);
        let heading = combined.into_heading(0.05);

        // Should point in direction of slot 0 (angle = 0, direction = +X)
        let expected_direction = Vec3::new(1.0, 0.0, 0.0);
        assert!(
            heading.normalize().dot(expected_direction) > 0.95,
            "Expected heading to point in +X direction. Got: {:?}",
            heading
        );
    }

    #[test]
    fn test_into_heading_with_urgency() {
        // Test that urgency overrides low speed
        let mut target = SteeringTarget::default();

        target.interest_map[0] = 1.0;
        target.danger_map[0] = 0.0;
        target.speed = 0.2;
        target.urgency = 0.8;

        let combined = CombinedSteeringTarget(target);
        let heading = combined.into_heading(0.05);

        // Speed should be lerped from 0.2 towards 1.0 by urgency factor 0.8
        // lerp(0.2, 1.0, 0.8) = 0.2 + (1.0 - 0.2) * 0.8 = 0.2 + 0.64 = 0.84
        let expected_speed = 0.2 + (1.0 - 0.2) * 0.8;
        assert!(
            (heading.length() - expected_speed).abs() < 0.01,
            "Expected speed to be {}, got: {}",
            expected_speed,
            heading.length()
        );
    }
}

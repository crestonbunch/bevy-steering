use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs},
};

/// PathFollowing enables a character to steer along a predetermined path.
/// The path is represented as a spine (polyline of Vec3 points) and a radius,
/// forming a "tube" or "generalized cylinder" that the character should stay within.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct PathFollowing {
    /// The points that make up the path spine (polyline)
    pub path: Vec<Vec3>,
    /// The radius of the path tube
    #[derivative(Default(value = "1.0"))]
    pub path_radius: f32,
    /// How far ahead to predict the character's future position (in time units)
    #[derivative(Default(value = "1.0"))]
    pub prediction_time: f32,
    /// How far ahead along the path to look for the target point
    #[derivative(Default(value = "1.0"))]
    pub target_distance: f32,
}

impl PathFollowing {
    /// Create a new PathFollowing behavior with the given path and radius
    pub fn new(path: Vec<Vec3>, path_radius: f32) -> Self {
        Self {
            path,
            path_radius,
            prediction_time: 1.0,
            target_distance: 3.0,
        }
    }

    /// Set the prediction time for the path following behavior. This
    /// controls how far along the path the agent should look for targets.
    /// If this is too far ahead, the agent might overshoot the path.
    pub fn with_prediction_time(mut self, prediction_time: f32) -> Self {
        self.prediction_time = prediction_time.max(0.01);
        self
    }

    /// Set the target distance for the path following behavior. This
    /// controls how far ahead the agent should look for the next point
    /// on the path.
    pub fn with_target_distance(mut self, target_distance: f32) -> Self {
        self.target_distance = target_distance;
        self
    }

    /// Set the path points for the path following behavior.
    pub fn set_path(&mut self, path: Vec<Vec3>) {
        self.path = path;
    }

    /// Get a reference to the path points.
    pub fn path(&self) -> &[Vec3] {
        &self.path
    }

    /// Get the path radius.
    pub fn path_radius(&self) -> f32 {
        self.path_radius
    }

    /// Finds the nearest point on the path spine to the given position.
    /// Returns the nearest point, the squared distance to it, and the segment index.
    fn nearest_point_on_path(&self, position: Vec3) -> Option<(Vec3, f32, usize)> {
        if self.path.is_empty() {
            return None;
        }

        if self.path.len() == 1 {
            let nearest = self.path[0];
            let dist_sq = position.distance_squared(nearest);
            return Some((nearest, dist_sq, 0));
        }

        let mut nearest_point = self.path[0];
        let mut min_dist_sq = position.distance_squared(nearest_point);
        let mut nearest_segment = 0;

        // Check each line segment in the path
        for i in 0..self.path.len() - 1 {
            let segment_start = self.path[i];
            let segment_end = self.path[i + 1];

            let nearest_on_segment = nearest_point_on_segment(position, segment_start, segment_end);
            let dist_sq = position.distance_squared(nearest_on_segment);

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                nearest_point = nearest_on_segment;
                nearest_segment = i;
            }
        }

        Some((nearest_point, min_dist_sq, nearest_segment))
    }

    /// Finds a target point ahead on the path from the given nearest point.
    /// This gives the agent forward momentum along the path.
    fn target_point_ahead(&self, nearest_point: Vec3, segment_index: usize) -> Vec3 {
        if self.path.is_empty() {
            return nearest_point;
        }

        if self.path.len() == 1 {
            return self.path[0];
        }

        let mut remaining_distance = self.target_distance;
        let mut current_point = nearest_point;
        let mut current_segment = segment_index;

        // Walk forward along the path
        while remaining_distance > 0.0 && current_segment < self.path.len() - 1 {
            let segment_end = self.path[current_segment + 1];
            let to_end = segment_end - current_point;
            let segment_length = to_end.length();

            if segment_length <= remaining_distance {
                // Move to the next segment
                remaining_distance -= segment_length;
                current_point = segment_end;
                current_segment += 1;
            } else {
                // Target point is within this segment
                current_point += to_end.normalize() * remaining_distance;
                break;
            }
        }

        current_point
    }
}

/// Finds the nearest point on a line segment to a given point.
fn nearest_point_on_segment(point: Vec3, segment_start: Vec3, segment_end: Vec3) -> Vec3 {
    let segment = segment_end - segment_start;
    let segment_length_sq = segment.length_squared();

    // Handle degenerate segment (start == end)
    if segment_length_sq < f32::EPSILON {
        return segment_start;
    }

    // Project point onto the line defined by the segment
    let to_point = point - segment_start;
    let t = to_point.dot(segment) / segment_length_sq;

    // Clamp t to [0, 1] to stay within the segment
    let t_clamped = t.clamp(0.0, 1.0);

    segment_start + segment * t_clamped
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct PathFollowingBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    path_following: &'static PathFollowing,
    global_transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
    outputs: &'static mut SteeringOutputs,
}

/// Path following behavior steers the agent along a predetermined path.
/// The agent predicts its future position based on velocity, projects that
/// onto the nearest point on the path spine, and steers towards that point
/// if the distance exceeds the path radius.
pub(crate) fn run(mut query: Query<PathFollowingBehaviorAgentQuery>) {
    for mut item in query.iter_mut() {
        let agent_pos = item.global_transform.translation();
        let agent_velocity = **item.velocity;

        // If path is empty, clear the behavior
        if item.path_following.path.is_empty() {
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        }

        // Predict future position based on current velocity
        let predicted_pos = agent_pos + agent_velocity * item.path_following.prediction_time;

        // Find nearest point on path to the predicted position
        let Some((nearest_point, dist_sq, segment_index)) =
            item.path_following.nearest_point_on_path(predicted_pos)
        else {
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        };

        let distance = dist_sq.sqrt();

        // Find a target point ahead on the path to give forward momentum
        let target_point = item
            .path_following
            .target_point_ahead(nearest_point, segment_index);

        // If we're far from the path, seek to the nearest point to get back on track
        // Otherwise, seek to the target point ahead to follow the path forward
        let seek_target = if distance > item.path_following.path_radius {
            nearest_point
        } else {
            target_point
        };

        // Steer towards the selected target
        let to_target = seek_target - agent_pos;
        if to_target.length_squared() < 0.01 {
            // Very close to target, no steering needed
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        }

        let mut steering_target = item
            .outputs
            .get(BehaviorType::PathFollowing)
            .unwrap_or_default();
        steering_target.set_interest(to_target.normalize());
        item.outputs
            .set(BehaviorType::PathFollowing, steering_target);
    }
}

/// Debug visualization for path following behavior.
/// Shows waypoints, path connections, path radius, predicted position, and target points.
pub(crate) fn debug_path_following(
    mut gizmos: Gizmos,
    query: Query<(&PathFollowing, &GlobalTransform, &LinearVelocity)>,
) {
    for (path_following, global_transform, velocity) in query.iter() {
        if path_following.path.is_empty() {
            continue;
        }

        let agent_pos = global_transform.translation();
        let agent_velocity = **velocity;

        // Draw waypoints as small spheres
        for waypoint in &path_following.path {
            gizmos.sphere(*waypoint, 0.2, Color::srgb(0.0, 1.0, 1.0)); // Cyan waypoints
        }

        // Draw path connections
        for i in 0..path_following.path.len() - 1 {
            let start = path_following.path[i];
            let end = path_following.path[i + 1];
            gizmos.line(start, end, Color::srgb(0.0, 0.8, 0.8)); // Cyan line
        }

        // Draw path radius as circles at each waypoint
        for waypoint in &path_following.path {
            gizmos.circle(
                Isometry3d::new(*waypoint, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                path_following.path_radius,
                Color::srgb(0.0, 1.0, 1.0).with_alpha(0.3), // Semi-transparent cyan
            );
        }

        // Draw path radius as circles along segments
        for i in 0..path_following.path.len() - 1 {
            let start = path_following.path[i];
            let end = path_following.path[i + 1];
            let segment_length = (end - start).length();
            let num_circles = (segment_length / path_following.path_radius).max(1.0) as usize;

            for j in 0..num_circles {
                let t = j as f32 / num_circles as f32;
                let point = start + (end - start) * t;
                gizmos.circle(
                    Isometry3d::new(point, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
                    path_following.path_radius,
                    Color::srgb(0.0, 1.0, 1.0).with_alpha(0.1), // Very transparent cyan
                );
            }
        }

        // Draw predicted position
        let predicted_pos = agent_pos + agent_velocity * path_following.prediction_time;
        gizmos.sphere(predicted_pos, 0.3, Color::srgb(1.0, 1.0, 0.0)); // Yellow predicted position
        gizmos.line(agent_pos, predicted_pos, Color::srgba(1.0, 1.0, 0.0, 0.5)); // Yellow line to predicted

        // Find and draw nearest point on path
        if let Some((nearest_point, dist_sq, segment_index)) =
            path_following.nearest_point_on_path(predicted_pos)
        {
            let distance = dist_sq.sqrt();

            // Draw nearest point
            gizmos.sphere(nearest_point, 0.25, Color::srgb(1.0, 0.5, 0.0)); // Orange nearest point
            gizmos.line(predicted_pos, nearest_point, Color::srgba(1.0, 0.5, 0.0, 0.5)); // Orange line

            // Draw target point ahead
            let target_point = path_following.target_point_ahead(nearest_point, segment_index);
            gizmos.sphere(target_point, 0.3, Color::srgb(0.0, 1.0, 0.0)); // Green target point
            gizmos.line(nearest_point, target_point, Color::srgba(0.0, 1.0, 0.0, 0.5)); // Green line

            // Draw the seek target (what the agent is actually steering towards)
            let seek_target = if distance > path_following.path_radius {
                nearest_point
            } else {
                target_point
            };
            gizmos.line(agent_pos, seek_target, Color::srgb(1.0, 0.0, 1.0)); // Magenta line to seek target

            // Draw a larger sphere around the active seek target
            gizmos.sphere(seek_target, 0.4, Color::srgba(1.0, 0.0, 1.0, 0.3)); // Semi-transparent magenta
        }
    }
}

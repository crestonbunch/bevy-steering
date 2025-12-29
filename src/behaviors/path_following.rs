use bevy::{
    ecs::{lifecycle::HookContext, query::QueryData, world::DeferredWorld},
    prelude::*,
};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::control::{BehaviorType, SteeringOutputs};

/// PathFollowing steers an agent along a path using the "carrot on a stick" approach.
/// The agent always seeks toward a target point that is `lookahead_distance` ahead
/// on the path from the closest point to the agent.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[component(on_remove = on_path_following_remove)]
pub struct PathFollowing {
    /// The points that make up the path (polyline)
    pub path: Vec<Vec3>,
    /// How far ahead along the path to place the target "carrot"
    #[derivative(Default(value = "3.0"))]
    pub lookahead_distance: f32,
}

impl PathFollowing {
    /// Create a new PathFollowing behavior with the given path
    pub fn new(path: Vec<Vec3>) -> Self {
        Self {
            path,
            lookahead_distance: 3.0,
        }
    }

    /// Set the lookahead distance (how far ahead the "carrot" is placed)
    pub fn with_lookahead_distance(mut self, distance: f32) -> Self {
        self.lookahead_distance = distance.max(0.1);
        self
    }

    /// Set the path points
    pub fn set_path(&mut self, path: Vec<Vec3>) {
        self.path = path;
    }

    /// Get a reference to the path points
    pub fn path(&self) -> &[Vec3] {
        &self.path
    }

    /// Finds the nearest point on the path to the given position.
    /// Returns the nearest point and the segment index.
    fn nearest_point_on_path(&self, position: Vec3) -> Option<(Vec3, usize)> {
        if self.path.is_empty() {
            return None;
        }

        if self.path.len() == 1 {
            return Some((self.path[0], 0));
        }

        let mut nearest_point = self.path[0];
        let mut min_dist_sq = position.distance_squared(nearest_point);
        let mut nearest_segment = 0;

        for i in 0..self.path.len() - 1 {
            let segment_start = self.path[i];
            let segment_end = self.path[i + 1];

            let point_on_segment = nearest_point_on_segment(position, segment_start, segment_end);
            let dist_sq = position.distance_squared(point_on_segment);

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                nearest_point = point_on_segment;
                nearest_segment = i;
            }
        }

        Some((nearest_point, nearest_segment))
    }

    /// Finds the "carrot" point that is lookahead_distance ahead on the path
    /// from the given nearest point.
    fn carrot_point(&self, nearest_point: Vec3, segment_index: usize) -> Vec3 {
        if self.path.is_empty() {
            return nearest_point;
        }

        if self.path.len() == 1 {
            return self.path[0];
        }

        let mut remaining = self.lookahead_distance;
        let mut current = nearest_point;
        let mut segment = segment_index;

        while remaining > 0.0 && segment < self.path.len() - 1 {
            let segment_end = self.path[segment + 1];
            let to_end = segment_end - current;
            let len = to_end.length();

            if len <= remaining {
                remaining -= len;
                current = segment_end;
                segment += 1;
            } else {
                current += to_end.normalize() * remaining;
                break;
            }
        }

        current
    }
}

/// Finds the nearest point on a line segment to a given point.
fn nearest_point_on_segment(point: Vec3, start: Vec3, end: Vec3) -> Vec3 {
    let segment = end - start;
    let len_sq = segment.length_squared();

    if len_sq < f32::EPSILON {
        return start;
    }

    let t = (point - start).dot(segment) / len_sq;
    start + segment * t.clamp(0.0, 1.0)
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct PathFollowingBehaviorAgentQuery {
    path_following: &'static PathFollowing,
    global_transform: &'static GlobalTransform,
    outputs: &'static mut SteeringOutputs,
}

/// Path following behavior: find closest point on path, seek toward carrot ahead.
pub(crate) fn run(mut query: Query<PathFollowingBehaviorAgentQuery>) {
    for mut item in query.iter_mut() {
        let agent_pos = item.global_transform.translation();

        if item.path_following.path.is_empty() {
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        }

        let Some((nearest, segment)) = item.path_following.nearest_point_on_path(agent_pos) else {
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        };

        let carrot = item.path_following.carrot_point(nearest, segment);
        let to_carrot = carrot - agent_pos;

        if to_carrot.length_squared() < 0.01 {
            item.outputs.clear(BehaviorType::PathFollowing);
            continue;
        }

        let mut target = item
            .outputs
            .get(BehaviorType::PathFollowing)
            .unwrap_or_default();
        target.set_interest(to_carrot.normalize());
        item.outputs.set(BehaviorType::PathFollowing, target);
    }
}

/// Debug visualization: path line, nearest point, carrot, and steering line.
pub(crate) fn debug_path_following(
    mut gizmos: Gizmos,
    query: Query<(&PathFollowing, &GlobalTransform)>,
) {
    for (path_following, transform) in query.iter() {
        if path_following.path.is_empty() {
            continue;
        }

        let agent_pos = transform.translation();

        // Draw path
        for i in 0..path_following.path.len() - 1 {
            gizmos.line(
                path_following.path[i],
                path_following.path[i + 1],
                Color::srgb(0.0, 0.8, 0.8),
            );
        }

        // Draw waypoints
        for waypoint in &path_following.path {
            gizmos.sphere(*waypoint, 0.2, Color::srgb(0.0, 1.0, 1.0));
        }

        // Draw nearest point and carrot
        if let Some((nearest, segment)) = path_following.nearest_point_on_path(agent_pos) {
            gizmos.sphere(nearest, 0.25, Color::srgb(1.0, 0.5, 0.0)); // Orange

            let carrot = path_following.carrot_point(nearest, segment);
            gizmos.sphere(carrot, 0.3, Color::srgb(0.0, 1.0, 0.0)); // Green

            // Line from agent to carrot
            gizmos.line(agent_pos, carrot, Color::srgb(1.0, 0.0, 1.0)); // Magenta
        }
    }
}

fn on_path_following_remove(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    if let Some(mut outputs) = world.get_mut::<SteeringOutputs>(entity) {
        outputs.clear(BehaviorType::PathFollowing);
    }
}

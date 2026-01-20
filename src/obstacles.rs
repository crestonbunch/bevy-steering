use avian3d::{collision::collider::contact_query::distance, prelude::*};
use bevy::{ecs::query::QueryData, platform::collections::HashMap, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{SMALL_THRESHOLD, agent::SteeringAgent};

/// A component that finds obstacles within a certain distance. Nearby
/// obstacles will be stored in the [NearbyObstacles] component.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[reflect(Component)]
#[require(NearbyObstacles)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct TrackNearbyObstacles {
    /// How far to look for obstacles. Any entities further away will
    /// not be included in the computed obstacles.
    #[derivative(Default(value = "10.0"))]
    pub distance: f32,
    /// The layer mask used for obstacle detection
    #[derivative(Default(value = "LayerMask::DEFAULT"))]
    pub avoid_mask: LayerMask,
    /// Custom list of precise entities to ignore as obstacles
    pub ignore_entites: Vec<Entity>,
    /// The maximum number of obstacles to track. Because collision
    /// detection is fairly expensive, this number should remain small
    /// to maintain high FPS in crowded scenes. The default 5 should
    /// be effective in most cases.
    #[derivative(Default(value = "5"))]
    pub max_hits: usize,
}

impl TrackNearbyObstacles {
    /// Set the distance to look for obstacles.
    pub fn with_distance(mut self, distance: f32) -> Self {
        self.distance = distance;
        self
    }

    /// Set the layer mask used for obstacle detection.
    pub fn with_avoid_mask(mut self, mask: LayerMask) -> Self {
        self.avoid_mask = mask;
        self
    }

    /// Add entities to ignore as obstacles.
    pub fn with_ignored_entities(mut self, entities: Vec<Entity>) -> Self {
        self.ignore_entites = entities;
        self
    }

    /// Set the maximum number of obstacles to track.
    pub fn with_max_hits(mut self, max_hits: usize) -> Self {
        self.max_hits = max_hits;
        self
    }
}

/// A struct that stores the entities, position, and velocity of
/// a nearby obstacle. This makes it easier to get common properties
/// for obstacles without performing extra queries. To track these entities,
/// add a [TrackNearbyObstacles] component. Obstacles will be added to a
/// [NearbyObstacles] component.
#[derive(Component, Debug, Clone, Reflect)]
#[reflect(Component)]
pub struct ComputedObstacle {
    pub entity: Entity,
    pub transform: GlobalTransform,
    pub velocity: Vec3,
    pub distance: f32,
    pub impact_points: Option<(Vec3, Vec3)>,
    pub impact_normals: Option<(Vec3, Vec3)>,
}

/// A struct that stores the nearby obstacles of an agent. Automatically
/// updated when [TrackNearbyObstacles] is added to a component.
#[derive(Component, Debug, Clone, Reflect, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct NearbyObstacles {
    pub obstacles: HashMap<Entity, ComputedObstacle>,
}

#[derive(QueryData)]
#[query_data(mutable)]
pub(crate) struct UpdateNearbyObstaclesQuery {
    entity: Entity,
    colliders: &'static RigidBodyColliders,
    rigid_body: &'static RigidBody,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
    nearby_obstacles: &'static mut NearbyObstacles,
    track_obstacles: &'static TrackNearbyObstacles,
}

#[derive(QueryData)]
pub(crate) struct UpdateNearbyObstaclesColliderQuery {
    entity: Entity,
    collider: &'static Collider,
    collider_of: &'static ColliderOf,
    transform: &'static GlobalTransform,
}

#[derive(QueryData)]
pub(crate) struct UpdateNearbyObstaclesRigidBodyQuery {
    entity: Entity,
    rigid_body: &'static RigidBody,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

pub(crate) fn update_nearby_obstacles(
    mut agent_query: Query<UpdateNearbyObstaclesQuery>,
    collider_query: Query<UpdateNearbyObstaclesColliderQuery>,
    rigid_body_query: Query<UpdateNearbyObstaclesRigidBodyQuery>,
    spatial_query: SpatialQuery,
) {
    for agent_query_item in agent_query.iter_mut() {
        let mut obstacles = agent_query_item.nearby_obstacles;
        let origin = agent_query_item.transform.translation();
        let rot = Quat::IDENTITY;
        let radius = agent_query_item.track_obstacles.distance;
        let mask = agent_query_item.track_obstacles.avoid_mask;
        let exclude = [
            agent_query_item.track_obstacles.ignore_entites.clone(),
            agent_query_item.colliders.iter().collect(),
        ]
        .concat();
        let shape = Collider::sphere(radius);
        let filter = SpatialQueryFilter::default()
            .with_mask(mask)
            .with_excluded_entities(exclude);

        let hits = spatial_query.shape_intersections(&shape, origin, rot, &filter);
        obstacles.clear();

        let agent_velocity = agent_query_item.velocity;
        let agent_dir = if agent_velocity.length_squared() > SMALL_THRESHOLD {
            Dir3::new_unchecked(agent_velocity.normalize())
        } else {
            agent_query_item.transform.forward()
        };

        for hit in hits {
            if obstacles.len() >= agent_query_item.track_obstacles.max_hits {
                // We've hit the maximum number of obstacles to track.
                break;
            }
            let Ok(collider_item) = collider_query.get(hit) else {
                continue;
            };
            let Ok(rigid_body) = rigid_body_query.get(collider_item.collider_of.body) else {
                continue;
            };
            if rigid_body.entity == agent_query_item.entity {
                continue;
            }
            let mut closest_distance = f32::INFINITY;
            let mut impact_points = None;
            let mut impact_normals = None;
            let agent_colliders = agent_query_item
                .colliders
                .iter()
                .flat_map(|e| collider_query.get(e));
            // Find the closest distance between all agent colliders and the candidate collider.
            for agent_collider in agent_colliders {
                let d = distance(
                    agent_collider.collider,
                    agent_collider.transform.translation(),
                    agent_collider.transform.rotation(),
                    collider_item.collider,
                    collider_item.transform.translation(),
                    collider_item.transform.rotation(),
                );
                let Ok(d) = d else {
                    warn!("Unsupported distance calculation in obstacles");
                    continue;
                };
                if d < closest_distance {
                    closest_distance = d;
                }
                let impacts = spatial_query.cast_shape(
                    agent_collider.collider,
                    agent_collider.transform.translation(),
                    agent_collider.transform.rotation(),
                    agent_dir,
                    &ShapeCastConfig {
                        max_distance: radius,
                        ..Default::default()
                    },
                    &filter,
                );
                let Some(impact) = impacts else {
                    continue;
                };
                if impact.entity == collider_item.entity {
                    let a = agent_collider.transform.transform_point(impact.point2);
                    impact_points = Some((a, impact.point1));
                    impact_normals = Some((impact.normal2, impact.normal1));
                };
            }
            if let Some(existing_obstacle) = obstacles.obstacles.get(&rigid_body.entity) {
                // Only update the obstacle if the new collider distance is closer.
                if existing_obstacle.distance < closest_distance {
                    continue;
                }
            }
            let computed_obstacle = ComputedObstacle {
                entity: rigid_body.entity,
                transform: *rigid_body.transform,
                velocity: rigid_body.velocity.xyz(),
                // closest_points: closest_points_result,
                distance: closest_distance,
                impact_points,
                impact_normals,
            };
            obstacles
                .obstacles
                .insert(rigid_body.entity, computed_obstacle);
        }
    }
}

#[derive(QueryData)]
pub(crate) struct DebugObstaclesQuery {
    agent: &'static SteeringAgent,
    transform: &'static GlobalTransform,
    track: &'static TrackNearbyObstacles,
    obstacles: &'static NearbyObstacles,
}

pub(crate) fn debug_obstacles(mut gizmos: Gizmos, agent_query: Query<DebugObstaclesQuery>) {
    const SPHERE_COLOR: Color = Color::srgb(0.933, 0.0, 1.0);
    const LINE_COLOR: Color = Color::srgb(0.933, 0.0, 1.0);
    const LINE_COLOR_2: Color = Color::srgb(0.0, 0.933, 1.0);

    for result in agent_query.iter() {
        let agent_position = result.transform.translation();
        gizmos.sphere(agent_position, result.track.distance, SPHERE_COLOR);
        for obstacle in result.obstacles.values() {
            if let Some((point_a, point_b)) = obstacle.impact_points {
                gizmos.line_gradient(point_a, point_b, LINE_COLOR, LINE_COLOR_2);
            }
        }
    }
}

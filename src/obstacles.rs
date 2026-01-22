use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, platform::collections::HashMap, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{agent::SteeringAgent, control::ForwardDir};

/// Sentinel component that triggers obstacle tracking setup.
/// Automatically added when [TrackNearbyObstacles] is added, and removed
/// once setup is complete.
#[derive(Component, Debug, Clone, Default, Reflect)]
#[reflect(Component)]
pub struct SetupObstacleTracking;

/// A component that finds obstacles within a certain distance. Nearby
/// obstacles will be stored in the [NearbyObstacles] component.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[reflect(Component)]
#[require(NearbyObstacles, SetupObstacleTracking)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct TrackNearbyObstacles {
    /// How far to look for obstacles. Any entities further away will
    /// not be included in the computed obstacles.
    #[derivative(Default(value = "10.0"))]
    pub distance: f32,
    /// The radius of the sphere collider used for obstacle detection.
    #[derivative(Default(value = "1.0"))]
    pub radius: f32,
    /// The layer mask used for obstacle detection
    #[derivative(Default(value = "LayerMask::DEFAULT"))]
    pub avoid_mask: LayerMask,
    /// Custom list of precise entities to ignore as obstacles
    pub ignore_entites: Vec<Entity>,
}

impl TrackNearbyObstacles {
    /// Set the distance to look for obstacles.
    pub fn with_distance(mut self, distance: f32) -> Self {
        self.distance = distance;
        self
    }

    /// Set the radius of the sphere collider used for detection.
    pub fn with_radius(mut self, radius: f32) -> Self {
        self.radius = radius;
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
}

/// A struct that stores the entities, position, and velocity of
/// a nearby obstacle. This makes it easier to get common properties
/// for obstacles without performing extra queries. To track these entities,
/// add a [TrackNearbyObstacles] component. Obstacles will be added to a
/// [NearbyObstacles] component.
#[derive(Component, Debug, Clone, Reflect)]
#[reflect(Component)]
pub struct ComputedObstacle {
    /// The entity of the obstacle.
    pub entity: Entity,
    /// The current global transform of the obstacle.
    pub transform: GlobalTransform,
    /// The current velocity of the obstacle.
    pub velocity: Vec3,
    /// The distance the detection sphere traveled before it collided with
    /// an obstacle.
    pub distance: f32,
    /// Points in global space. The first point is the _current_ global-space
    /// point that will be impacted _in the future_. (This will change if the
    /// obstacle moves.) The second point is the _current_ point on the
    /// agent's detector (a sphere) that will be impacted _in the future_.
    /// (This will change if the agent moves.)
    pub impact_points: Option<(Vec3, Vec3)>,
    /// Points in local space. The first point is the future impact normal on
    /// the obstacle. The second point is the future impact normal on the
    /// agent.
    pub impact_normals: Option<(Vec3, Vec3)>,
}

/// A struct that stores the nearby obstacles of an agent. Automatically
/// updated when [TrackNearbyObstacles] is added to a component.
#[derive(Component, Debug, Clone, Reflect, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct NearbyObstacles {
    pub obstacles: HashMap<Entity, ComputedObstacle>,
}

/// Stores references to child obstacle detector entities.
#[derive(Component, Debug, Clone, Default, Reflect, Deref, DerefMut)]
#[reflect(Component)]
#[relationship_target(relationship = ObstacleDetectorOf)]
pub(crate) struct ObstacleDetectors(Vec<Entity>);

/// Marker component for obstacle detection shape casters.
#[derive(Component, Debug, Clone, Reflect)]
#[reflect(Component)]
#[relationship(relationship_target = ObstacleDetectors)]
pub(crate) struct ObstacleDetectorOf(Entity);

pub(crate) fn setup_obstacle_tracking(
    mut commands: Commands,
    query: Query<(Entity, &TrackNearbyObstacles, &RigidBodyColliders), With<SetupObstacleTracking>>,
) {
    for (entity, config, colliders) in query.iter() {
        let distance = config.distance;
        let mask = config.avoid_mask;
        let exclude = [config.ignore_entites.clone(), colliders.iter().collect()].concat();
        let shape = Collider::sphere(config.radius);
        let filter = SpatialQueryFilter::default()
            .with_mask(mask)
            .with_excluded_entities(exclude);
        let detector_id = commands
            .spawn((
                ObstacleDetectorOf(entity),
                ShapeCaster::new(shape.clone(), Vec3::ZERO, Quat::default(), Dir3::NEG_Z)
                    .with_max_distance(distance)
                    .with_query_filter(filter.clone()),
            ))
            .id();
        commands.entity(entity).add_child(detector_id);
        commands.entity(entity).remove::<SetupObstacleTracking>();
    }
}

pub(crate) fn update_obstacle_tracking(
    parent_query: Query<(&GlobalTransform, &ForwardDir, &ObstacleDetectors)>,
    mut detector_query: Query<&mut ShapeCaster, With<ObstacleDetectorOf>>,
) {
    for (transform, forward_dir, child_detectors) in parent_query.iter() {
        // Get the world-space forward direction
        let world_forward = forward_dir.as_vec3();
        for detector_entity in child_detectors.iter() {
            let Ok(mut shape_caster) = detector_query.get_mut(detector_entity) else {
                continue;
            };

            // Convert world-space forward direction to detector's local space
            let local_dir = transform
                .affine()
                .inverse()
                .transform_vector3(world_forward);

            if let Ok(dir) = Dir3::new(local_dir) {
                shape_caster.direction = dir;
            }
        }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub(crate) struct UpdateNearbyObstaclesQuery {
    entity: Entity,
    transform: &'static GlobalTransform,
    nearby_obstacles: &'static mut NearbyObstacles,
    track_obstacles: &'static TrackNearbyObstacles,
    child_detectors: &'static ObstacleDetectors,
}

#[derive(QueryData)]
pub(crate) struct DetectorQuery {
    shape_hits: &'static ShapeHits,
}

#[derive(QueryData)]
pub(crate) struct ColliderQuery {
    collider_of: &'static ColliderOf,
    transform: &'static GlobalTransform,
}

#[derive(QueryData)]
pub(crate) struct RigidBodyQuery {
    entity: Entity,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

pub(crate) fn update_nearby_obstacles(
    mut agent_query: Query<UpdateNearbyObstaclesQuery>,
    detector_query: Query<DetectorQuery, With<ObstacleDetectorOf>>,
    collider_query: Query<ColliderQuery>,
    rigid_body_query: Query<RigidBodyQuery, With<RigidBody>>,
) {
    for mut agent in agent_query.iter_mut() {
        agent.nearby_obstacles.clear();

        // Iterate through all child obstacle detectors
        for detector_entity in agent.child_detectors.iter() {
            let Ok(detector) = detector_query.get(detector_entity) else {
                continue;
            };

            // Process each shape hit from this detector
            for hit in detector.shape_hits.iter() {
                // Get the collider info to find its parent rigid body
                let Ok(collider_info) = collider_query.get(hit.entity) else {
                    continue;
                };

                // Get the rigid body that owns this collider
                let Ok(rigid_body) = rigid_body_query.get(collider_info.collider_of.body) else {
                    continue;
                };

                let point2 = agent.transform.transform_point(hit.point2);
                // (obstacle, agent) tuples
                let impact_points = Some((hit.point1, point2));
                let impact_normals = Some((hit.normal1, hit.normal2));
                let computed_obstacle = ComputedObstacle {
                    entity: rigid_body.entity,
                    transform: *rigid_body.transform,
                    velocity: rigid_body.velocity.0,
                    distance: hit.distance,
                    impact_points,
                    impact_normals,
                };

                agent
                    .nearby_obstacles
                    .insert(rigid_body.entity, computed_obstacle);
            }
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
    const LINE_COLOR_2: Color = Color::srgb(0.0, 0.933, 0.0);

    for result in agent_query.iter() {
        let agent_position = result.transform.translation();
        gizmos.sphere(agent_position, result.track.distance, SPHERE_COLOR);
        for obstacle in result.obstacles.values() {
            if let Some((point_a, point_b)) = obstacle.impact_points {
                gizmos.line_gradient(point_a, point_b, LINE_COLOR, LINE_COLOR_2);

                if let Some((normal_a, normal_b)) = obstacle.impact_normals {
                    gizmos.line(point_a, normal_a, LINE_COLOR);
                    gizmos.line(point_a, point_a + normal_b, LINE_COLOR_2);
                }
            }
        }
    }
}

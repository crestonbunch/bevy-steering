use avian3d::{
    collision::collider::contact_query::{ClosestPoints, closest_points, contact, distance},
    prelude::*,
};
use bevy::{ecs::query::QueryData, platform::collections::HashMap, prelude::*};

use crate::agent::SteeringAgent;

/// A component that marks an entity as a neighbor. For steering behaviors
/// that require neighbors, you can add this component to the entity you want
/// to consider as a neighbor. For example, the player character in a game might
/// have a neighbor component to make agents avoid the player. All Agents are
/// automatically neighbors of each other.
#[derive(Component, Debug, Default, Reflect)]
#[reflect(Component)]
pub struct Neighbor;

/// A struct that stores the entities, position, and velocity of a neighbor.
/// This makes it easier to get common properties for neighbors without
/// performing extra queries.
#[derive(Component, Debug, Reflect)]
#[reflect(Component)]
pub(crate) struct ComputedNeighbor {
    pub(crate) entity: Entity,
    pub(crate) transform: GlobalTransform,
    pub(crate) velocity: Vec3,
    pub(crate) distance: f32,
    pub(crate) closest_points: Option<(Vec3, Vec3)>,
}

/// A component that stores the neighbors of an agent. Neighbors are
/// used to calculate steering behaviors based on the presence of other
/// agents.
#[derive(Component, Default, Reflect)]
#[reflect(Component)]
pub(crate) struct Neighborhood {
    pub(crate) neighbors: HashMap<Entity, ComputedNeighbor>,
}

#[derive(QueryData)]
#[query_data(mutable)]
pub(crate) struct UpdateNeighborhoodAgentQuery {
    entity: Entity,
    agent: &'static SteeringAgent,
    transform: &'static GlobalTransform,
    neighborhood: &'static mut Neighborhood,
    colliders: &'static RigidBodyColliders,
}

#[derive(QueryData)]
pub(crate) struct UpdateNeighborhoodColliderQuery {
    entity: Entity,
    collider: &'static Collider,
    collider_of: &'static ColliderOf,
    transform: &'static GlobalTransform,
}

#[derive(QueryData)]
pub(crate) struct UpdateNeighborhoodRigidBodyQuery {
    entity: Entity,
    rigid_body: &'static RigidBody,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
    _neighbor: &'static Neighbor, // filter only neighbors
}

pub(crate) fn update_neighborhoods(
    mut agent_query: Query<UpdateNeighborhoodAgentQuery>,
    collider_query: Query<UpdateNeighborhoodColliderQuery>,
    rigid_body_query: Query<UpdateNeighborhoodRigidBodyQuery>,
    spatial_query: SpatialQuery,
) {
    for agent_query_item in agent_query.iter_mut() {
        let mut neighborhood = agent_query_item.neighborhood;
        let agent = agent_query_item.agent;
        let shape = Collider::sphere(agent.neighborhood_radius);
        let origin = agent_query_item.transform.translation();
        let rot = Quat::IDENTITY;
        let filter = SpatialQueryFilter::default().with_mask(agent.neighborhood_filter);
        let spatial_query_result = spatial_query.shape_intersections(&shape, origin, rot, &filter);
        neighborhood.neighbors.clear();
        // For each candidate neighbor collider, find the closest distance between all agent
        // colliders and the candidate collider.
        for candidate in spatial_query_result {
            let Ok(collider_item) = collider_query.get(candidate) else {
                continue;
            };
            let Ok(rigid_body) = rigid_body_query.get(collider_item.collider_of.body) else {
                continue;
            };
            if rigid_body.entity == agent_query_item.entity {
                continue;
            }
            let mut closest_distance = f32::INFINITY;
            let mut closest_points_result = None;
            let agent_position = agent_query_item.transform.translation();
            let agent_rotation = agent_query_item.transform.rotation();
            let agent_colliders = agent_query_item
                .colliders
                .iter()
                .flat_map(|e| collider_query.get(e));
            // Find the closest distance between all agent colliders and the candidate collider.
            for agent_collider in agent_colliders {
                let points = closest_points(
                    agent_collider.collider,
                    agent_position,
                    agent_rotation,
                    collider_item.collider,
                    collider_item.transform.translation(),
                    collider_item.transform.rotation(),
                    agent.neighborhood_radius,
                );
                let d = distance(
                    agent_collider.collider,
                    agent_position,
                    agent_rotation,
                    collider_item.collider,
                    collider_item.transform.translation(),
                    collider_item.transform.rotation(),
                );
                let contact = contact(
                    agent_collider.collider,
                    agent_position,
                    agent_rotation,
                    collider_item.collider,
                    collider_item.transform.translation(),
                    collider_item.transform.rotation(),
                    agent.neighborhood_radius,
                );
                let Ok(d) = d else {
                    warn!("Unsupported distance calculation in neighborhood");
                    continue;
                };
                if d < closest_distance {
                    closest_distance = d;
                    closest_points_result = match (points, contact) {
                        (Ok(ClosestPoints::WithinMargin(a, b)), _) => Some((a, b)),
                        (Ok(ClosestPoints::Intersecting), Ok(Some(contact))) => {
                            let pos = collider_item.transform.translation();
                            let world_point1 = pos + contact.local_point1;
                            let world_point2 = pos + contact.local_point2;
                            Some((world_point1, world_point2))
                        }
                        _ => None,
                    };
                }
            }
            if let Some(existing_neighbor) = neighborhood.neighbors.get(&rigid_body.entity) {
                // Only update the neighbor if the new collider distance is closer.
                if existing_neighbor.distance < closest_distance {
                    continue;
                }
            }
            let computed_neighbor = ComputedNeighbor {
                entity: rigid_body.entity,
                transform: *rigid_body.transform,
                velocity: rigid_body.velocity.xyz(),
                closest_points: closest_points_result,
                distance: closest_distance,
            };
            neighborhood
                .neighbors
                .insert(rigid_body.entity, computed_neighbor);
        }
    }
}

/// Draw gizmos in the scene to visualize the neighborhoods.
pub(crate) fn debug_neighborhoods(
    mut gizmos: Gizmos,
    agent_query: Query<(&SteeringAgent, &GlobalTransform, &Neighborhood)>,
) {
    const SPHERE_COLOR: Color = Color::srgba(0.0, 1.0, 1.0, 0.3);
    const LINE_COLOR: Color = Color::srgb(1.0, 0.0, 0.0);

    for (agent, transform, neighborhood) in agent_query.iter() {
        let agent_position = transform.translation();
        gizmos.sphere(agent_position, agent.neighborhood_radius, SPHERE_COLOR);
        for neighbor in neighborhood.neighbors.values() {
            if let Some((point_a, point_b)) = neighbor.closest_points {
                gizmos.line(point_a, point_b, LINE_COLOR);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use avian3d::prelude::{Collider, PhysicsPlugins, RigidBody};
    use bevy::{
        MinimalPlugins,
        asset::{AssetEvent, AssetPlugin, Assets},
        ecs::system::RunSystemOnce,
        prelude::{App, Mesh, Transform, Vec3},
        scene::ScenePlugin,
        transform::TransformPlugin,
    };

    fn basic_agent(position: Vec3) -> impl Bundle {
        (
            SteeringAgent::default().with_neighborhood_radius(5.0),
            Transform::from_translation(position),
            RigidBody::Static,
            Collider::sphere(0.5),
        )
    }

    fn complex_agent(world: &mut World, position: Vec3) -> Entity {
        let mut entity = world.spawn((
            SteeringAgent::default().with_neighborhood_radius(5.0),
            Transform::from_translation(position),
            RigidBody::Static,
        ));
        entity.with_child((
            Transform::from_translation(Vec3::new(0.0, -0.5, 0.0)),
            Collider::sphere(0.5),
        ));
        entity.with_child((
            Transform::from_translation(Vec3::new(0.0, 0.5, 0.0)),
            Collider::sphere(0.5),
        ));
        entity.id()
    }

    fn run_app_test<T>(setup: impl FnOnce(&mut App) -> T) -> (App, T) {
        let mut app = App::new();

        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            ScenePlugin,
            TransformPlugin,
            // Force the physics onto the Update schedule during tests
            PhysicsPlugins::new(Update),
        ));

        // Initialize required events and resources
        app.init_resource::<Assets<Mesh>>();
        app.add_message::<AssetEvent<Mesh>>();

        let result = setup(&mut app);

        for _ in 0..10 {
            app.finish();
            app.cleanup();
            app.update();
        }

        (app, result)
    }

    #[test]
    fn test_update_neighborhoods() {
        // Create a test app with necessary plugins
        let (mut app, (agent1, agent2)) = run_app_test(|app| {
            let agent1 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(0.0, 0.0, 0.0)))
                .id();
            let agent2 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(2.0, 0.0, 0.0)))
                .id();
            (agent1, agent2)
        });

        // Run the update_neighborhoods system
        app.world_mut()
            .run_system_once(update_neighborhoods)
            .expect("Failed to run update_neighborhoods system");

        // Basic verification: check that the neighborhoods exist and were updated
        let neighborhood1 = app.world().get::<Neighborhood>(agent1).unwrap();
        let neighborhood2 = app.world().get::<Neighborhood>(agent2).unwrap();

        assert_eq!(neighborhood1.neighbors.len(), 1);
        assert_eq!(neighborhood2.neighbors.len(), 1);
        let neighbor1 = neighborhood1.neighbors.values().next().unwrap();
        let neighbor2 = neighborhood2.neighbors.values().next().unwrap();
        assert_eq!(neighbor1.entity, agent2);
        assert_eq!(neighbor2.entity, agent1);
        assert_eq!(neighbor1.distance, 1.0);
        assert_eq!(neighbor2.distance, 1.0);

        let a = Vec3::new(0.5, 0.0, 0.0);
        let b = Vec3::new(1.5, 0.0, 0.0);
        assert_eq!(neighbor1.closest_points, Some((a, b)));
        assert_eq!(neighbor2.closest_points, Some((b, a)));
    }

    #[test]
    fn test_update_neighborhoods_with_complex_agent() {
        let (mut app, (agent1, agent2, agent3)) = run_app_test(|app| {
            let agent1 = complex_agent(app.world_mut(), Vec3::new(4.0, 0.0, 0.0));
            let agent2 = complex_agent(app.world_mut(), Vec3::new(0.0, 4.0, 0.0));
            let agent3 = complex_agent(app.world_mut(), Vec3::new(0.0, 0.0, 4.0));
            (agent1, agent2, agent3)
        });

        app.world_mut()
            .run_system_once(update_neighborhoods)
            .expect("Failed to run update_neighborhoods system");

        let neighborhood1 = app.world().get::<Neighborhood>(agent1).unwrap();
        let neighborhood2 = app.world().get::<Neighborhood>(agent2).unwrap();
        let neighborhood3 = app.world().get::<Neighborhood>(agent3).unwrap();

        assert!(!neighborhood1.neighbors.contains_key(&agent1));
        assert!(!neighborhood2.neighbors.contains_key(&agent2));
        assert!(!neighborhood3.neighbors.contains_key(&agent3));

        assert_eq!(neighborhood1.neighbors.len(), 1);
        assert_eq!(neighborhood2.neighbors.len(), 2);
        assert_eq!(neighborhood3.neighbors.len(), 1);

        let neighbor1 = neighborhood1.neighbors.values().next().unwrap();
        let neighbor2_1 = neighborhood2.neighbors.values().next().unwrap();
        let neighbor2_2 = neighborhood2.neighbors.values().nth(1).unwrap();
        let neighbor3 = neighborhood3.neighbors.values().next().unwrap();

        assert_eq!(neighbor1.entity, agent2);
        assert_eq!(neighbor2_1.entity, agent3);
        assert_eq!(neighbor2_2.entity, agent1);
        assert_eq!(neighbor3.entity, agent2);

        // NB: not exactly sure how 4.315073 is calculated,
        // I would expect it to be sqrt(4^2 + 4^2) - 1.0 = 4.65
        assert_eq!(neighbor1.distance, 4.315073);
        assert_eq!(neighbor2_1.distance, 4.315073);
        assert_eq!(neighbor2_2.distance, 4.315073);
        assert_eq!(neighbor3.distance, 4.315073);
    }
}

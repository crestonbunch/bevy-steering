use avian3d::prelude::LinearVelocity;
use bevy::{ecs::query::QueryData, prelude::*};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs, SteeringTarget},
    neighbors::Neighborhood,
};

/// Align behavior attempts to match the velocity and direction of nearby neighbors.
#[derive(Component, Default, Debug, Copy, Clone, Reflect)]
pub struct Align;

#[derive(QueryData)]
#[query_data(mutable)]
pub struct AlignmentBehaviorAgentQuery {
    align: &'static Align,
    agent: &'static SteeringAgent,
    neighborhood: &'static Neighborhood,
    target: &'static mut SteeringOutputs,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

/// Compute the alignment behavior for an agent. This behavior calculates the average
/// velocity of all neighbors and adjusts the agent's target to match that direction and speed.
pub(crate) fn run(mut query: Query<AlignmentBehaviorAgentQuery>) {
    for mut agent in query.iter_mut() {
        let neighbors = &agent.neighborhood.neighbors;

        if neighbors.is_empty() {
            agent.target.clear(BehaviorType::Alignment);
            continue;
        }

        // Calculate average velocity of all neighbors
        let mut avg_velocity = Vec3::ZERO;
        for neighbor in neighbors.values() {
            avg_velocity += neighbor.velocity;
        }
        avg_velocity /= neighbors.len() as f32;

        let speed = avg_velocity.length();

        // Skip if neighbors are stationary (avoid division by zero)
        if speed < 0.001 {
            agent.target.clear(BehaviorType::Alignment);
            continue;
        }

        let direction = avg_velocity / speed;

        // Create steering target aligned with neighbors' average velocity
        let mut target = SteeringTarget::default();
        target.set_interest(direction);
        agent.target.set(BehaviorType::Alignment, target);
    }
}

#[cfg(test)]
mod tests {
    use crate::control::BehaviorType;
    use crate::neighbors::update_neighborhoods;

    use super::*;
    use avian3d::prelude::*;
    use bevy::{
        MinimalPlugins,
        asset::{AssetEvent, AssetPlugin, Assets},
        ecs::system::RunSystemOnce,
        prelude::{App, Mesh, Transform, Vec3},
        scene::ScenePlugin,
        transform::TransformPlugin,
    };

    fn basic_agent(position: Vec3, velocity: Vec3) -> impl Bundle {
        let agent = SteeringAgent::default().with_neighborhood_radius(10.0);
        (
            Transform::from_translation(position),
            RigidBody::Kinematic,
            Collider::sphere(1.0),
            LinearVelocity(velocity),
            Align::default(),
            agent,
        )
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

        for _ in 0..2 {
            app.finish();
            app.cleanup();
            app.update();
        }

        (app, result)
    }

    #[test]
    fn test_simple_alignment() {
        // Agent1 at origin moving +Z, Agent2 nearby moving +X
        // Agent1 should want to align toward average velocity (+X+Z direction)
        let (mut app, (agent1, agent2)) = run_app_test(|app| {
            let agent1 = app
                .world_mut()
                .spawn(basic_agent(
                    Vec3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 5.0),
                ))
                .id();
            let agent2 = app
                .world_mut()
                .spawn(basic_agent(
                    Vec3::new(3.0, 0.0, 0.0),
                    Vec3::new(5.0, 0.0, 0.0),
                ))
                .id();
            (agent1, agent2)
        });

        // Run neighborhood detection and alignment behavior
        let _ = app.world_mut().run_system_once(update_neighborhoods);
        let _ = app.world_mut().run_system_once(super::run);

        // Check agent1's steering output - should want to align toward +X (neighbor's velocity)
        let outputs1 = app.world().get::<SteeringOutputs>(agent1).unwrap();
        let target1 = outputs1.get(BehaviorType::Alignment);
        assert!(
            target1.is_some(),
            "Agent1 should have alignment steering target"
        );
        let target1 = target1.unwrap();
        // Slot 0 is +X direction - should have high interest since neighbor is moving +X
        assert!(
            target1.interest_map[0] > 0.5,
            "Agent1 interest should be high in +X direction (slot 0), got {}",
            target1.interest_map[0]
        );

        // Check agent2's steering output - should want to align toward +Z (neighbor's velocity)
        let outputs2 = app.world().get::<SteeringOutputs>(agent2).unwrap();
        let target2 = outputs2.get(BehaviorType::Alignment);
        assert!(
            target2.is_some(),
            "Agent2 should have alignment steering target"
        );
        let target2 = target2.unwrap();
        // Slot 4 is +Z direction - should have high interest since neighbor is moving +Z
        assert!(
            target2.interest_map[4] > 0.5,
            "Agent2 interest should be high in +Z direction (slot 4), got {}",
            target2.interest_map[4]
        );
    }
}

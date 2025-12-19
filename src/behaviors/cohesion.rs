use avian3d::prelude::LinearVelocity;
use bevy::{ecs::query::QueryData, prelude::*};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs, SteeringTarget},
    neighbors::Neighborhood,
};

#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[derivative(Default)]
pub struct Cohere {
    /// Below this distance from center of mass, no cohesion is applied
    #[derivative(Default(value = "0.0"))]
    radius: f32,
}

impl Cohere {
    /// Set minimum radius - below this distance, no cohesion is applied
    pub fn with_radius(mut self, radius: f32) -> Self {
        self.radius = radius.max(0.0);
        self
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct CohesionBehaviorAgentQuery {
    cohesion: &'static Cohere,
    agent: &'static SteeringAgent,
    neighborhood: &'static Neighborhood,
    target: &'static mut SteeringOutputs,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

/// Compute the cohesion behavior for an agent. This behavior calculates the average
/// position (center of gravity) of all neighbors and sets that as the steering target.
pub(crate) fn run(mut query: Query<CohesionBehaviorAgentQuery>) {
    for mut agent in query.iter_mut() {
        let neighbors = &agent.neighborhood.neighbors;

        if neighbors.is_empty() {
            agent.target.clear(BehaviorType::Cohere);
            continue;
        }

        // Calculate center of mass of all neighbors
        let mut center_of_mass = Vec3::ZERO;
        for neighbor in neighbors.values() {
            center_of_mass += neighbor.transform.translation();
        }
        center_of_mass /= neighbors.len() as f32;

        // Compute direction from agent to center of mass
        let agent_position = agent.transform.translation();
        let to_center = center_of_mass - agent_position;
        let distance = to_center.length();

        // Skip if already at center (avoid division by zero)
        if distance < 0.001 {
            agent.target.clear(BehaviorType::Cohere);
            continue;
        }

        // Below min_radius: no cohesion
        if distance <= agent.cohesion.radius {
            agent.target.clear(BehaviorType::Cohere);
            continue;
        }

        let direction = to_center / distance;

        // Create steering target toward center of mass
        let mut target = SteeringTarget::default();
        target.set_interest(direction);
        agent.target.set(BehaviorType::Cohere, target);
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

    fn basic_agent(position: Vec3) -> impl Bundle {
        let agent = SteeringAgent::default().with_neighborhood_radius(10.0);
        (
            Transform::from_translation(position),
            RigidBody::Kinematic,
            Collider::sphere(1.0),
            Cohere::default(),
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
    fn test_simple_cohesion() {
        // Agent1 at origin, Agent2 to the right at x=5
        // Agent1 should want to move towards Agent2 (center of mass is at x=5)
        let (mut app, (agent1, agent2)) = run_app_test(|app| {
            let agent1 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(0.0, 0.0, 0.0)))
                .id();
            let agent2 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(5.0, 0.0, 0.0)))
                .id();
            (agent1, agent2)
        });

        // Run neighborhood detection and cohesion behavior
        let _ = app.world_mut().run_system_once(update_neighborhoods);
        let _ = app.world_mut().run_system_once(super::run);

        // Check agent1's steering output - should want to move right (toward agent2)
        let outputs1 = app.world().get::<SteeringOutputs>(agent1).unwrap();
        let target1 = outputs1.get(BehaviorType::Cohere);
        assert!(
            target1.is_some(),
            "Agent1 should have cohesion steering target"
        );
        let target1 = target1.unwrap();
        // Slot 0 is the +X direction (toward agent2)
        assert!(
            target1.interest_map[0] > 0.5,
            "Agent1 interest should be high in +X direction (slot 0), got {}",
            target1.interest_map[0]
        );

        // Check agent2's steering output - should want to move left (toward agent1)
        let outputs2 = app.world().get::<SteeringOutputs>(agent2).unwrap();
        let target2 = outputs2.get(BehaviorType::Cohere);
        assert!(
            target2.is_some(),
            "Agent2 should have cohesion steering target"
        );
        let target2 = target2.unwrap();
        // Slot 8 is the -X direction (toward agent1)
        assert!(
            target2.interest_map[8] > 0.5,
            "Agent2 interest should be high in -X direction (slot 8), got {}",
            target2.interest_map[8]
        );
    }
}

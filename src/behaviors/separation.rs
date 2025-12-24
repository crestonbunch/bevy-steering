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

/// Separation behavior attempts to maintain distance from nearby neighbors.
#[derive(Component, Debug, Copy, Clone, Reflect, Derivative)]
#[derivative(Default)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
pub struct Separate {
    /// The desired separation radius. Agents will try to maintain this
    /// distance from neighbors.
    #[derivative(Default(value = "10.0"))]
    pub desired_radius: f32,
    /// The panic radius. This is a much harder constraint, making the agent
    /// extremely unlikely to do anything except separate.
    #[derivative(Default(value = "1.0"))]
    pub panic_radius: f32,
}

impl Separate {
    /// Set the desired separation radius. Agents will try to
    /// maintain this distance from neighbors.
    pub fn with_desired_radius(mut self, radius: f32) -> Self {
        self.desired_radius = radius;
        self
    }

    /// Set the panic radius. This is a much harder constraint,
    /// making the agent extremely unlikely to do anything
    /// except separate.
    pub fn with_panic_radius(mut self, radius: f32) -> Self {
        self.panic_radius = radius;
        self
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct SeparationBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    neighborhood: &'static Neighborhood,
    separate: &'static Separate,
    target: &'static mut SteeringOutputs,
    transform: &'static GlobalTransform,
    velocity: &'static LinearVelocity,
}

/// Apply force to keep agents separated from their neighbors.
pub(crate) fn run(mut query: Query<SeparationBehaviorAgentQuery>) {
    for mut agent in query.iter_mut() {
        // Accumulate weighted "away" vectors from all neighbors
        let mut combined_away = Vec3::ZERO;
        let mut max_intensity = 0.0_f32;
        let mut max_urgency = 0.0_f32;

        for neighbor in agent.neighborhood.neighbors.values() {
            let Some(closest_points) = neighbor.closest_points else {
                continue;
            };
            let away = agent.transform.translation() - closest_points.1;
            let distance = away.length();

            // Skip if outside desired separation or too close (avoid division by zero)
            if distance >= agent.separate.desired_radius || distance < 0.001 {
                continue;
            }

            let direction = away / distance;
            let intensity = 1.0 - (distance / agent.separate.desired_radius);

            // Weight the away direction by intensity (closer = stronger)
            combined_away += direction * intensity;
            max_intensity = max_intensity.max(intensity);

            // If within panic radius, increase urgency
            if distance < agent.separate.panic_radius {
                let panic_level = 1.0 - (distance / agent.separate.panic_radius);
                max_urgency = max_urgency.max(panic_level);
            }
        }

        // Only set target if we have neighbors to separate from
        if max_intensity > 0.0 {
            let away_dir = combined_away.normalize_or_zero();
            let mut target = SteeringTarget::default();
            // Interest in moving away from neighbors
            // Danger in the direction toward neighbors
            target.set_interest(away_dir);
            target.add_danger(-away_dir, max_intensity);
            target.set_urgency(max_urgency);
            agent.target.set(BehaviorType::Separation, target);
        } else {
            // Stop moving if we're outside the desired radius
            agent.target.clear(BehaviorType::Separation);
        }
    }
}

pub(crate) fn debug_separation(
    mut gizmos: Gizmos,
    agent_query: Query<(&GlobalTransform, &Separate)>,
) {
    for (transform, separate) in agent_query.iter() {
        let agent_position = transform.translation();
        gizmos.sphere(
            agent_position,
            separate.desired_radius,
            Color::srgb(1.0, 1.0, 0.0),
        );
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
            Separate::default().with_desired_radius(10.0),
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
            PhysicsPlugins::new(Update),
        ));

        app.init_resource::<Assets<Mesh>>();
        app.add_message::<AssetEvent<Mesh>>();

        let result = setup(&mut app);

        // Run initial updates to initialize physics
        for _ in 0..2 {
            app.finish();
            app.cleanup();
            app.update();
        }

        (app, result)
    }

    #[test]
    fn test_simple_agent_separation() {
        // Agent1 at origin, Agent2 to the right at x=3
        let (mut app, (agent1, agent2)) = run_app_test(|app| {
            let agent1 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(0.0, 0.0, 0.0)))
                .id();
            let agent2 = app
                .world_mut()
                .spawn(basic_agent(Vec3::new(3.0, 0.0, 0.0)))
                .id();
            (agent1, agent2)
        });

        // Run neighborhood detection and separation behavior
        let _ = app.world_mut().run_system_once(update_neighborhoods);
        let _ = app.world_mut().run_system_once(super::run);

        // Check agent1's steering output - should want to move left (away from agent2)
        let outputs1 = app.world().get::<SteeringOutputs>(agent1).unwrap();
        let target1 = outputs1.get(BehaviorType::Separation);
        assert!(
            target1.is_some(),
            "Agent1 should have separation steering target"
        );
        let target1 = target1.unwrap();
        // Slot 8 is the -X direction (away from agent2)
        assert!(
            target1.interest_map[8] > 0.5,
            "Agent1 interest should be high in -X direction (slot 8), got {}",
            target1.interest_map[8]
        );

        // Check agent2's steering output - should want to move right (away from agent1)
        let outputs2 = app.world().get::<SteeringOutputs>(agent2).unwrap();
        let target2 = outputs2.get(BehaviorType::Separation);
        assert!(
            target2.is_some(),
            "Agent2 should have separation steering target"
        );
        let target2 = target2.unwrap();
        // Slot 0 is the +X direction (away from agent1)
        assert!(
            target2.interest_map[0] > 0.5,
            "Agent2 interest should be high in +X direction (slot 0), got {}",
            target2.interest_map[0]
        );
    }
}

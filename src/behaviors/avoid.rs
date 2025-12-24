use avian3d::prelude::*;
use bevy::{
    ecs::{lifecycle::HookContext, query::QueryData, world::DeferredWorld},
    prelude::*,
};
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    agent::SteeringAgent,
    control::{BehaviorType, SteeringOutputs, SteeringTarget},
};

/// Avoid obstacles. This is not a replacement for navigation
/// (e.g. A* or similar.) An agent can get stuck even with this
/// behavior. Use this behavior so that the agent routes around
/// obstacles that are directly in front.
#[derive(Component, Debug, Clone, Reflect, Derivative)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[derivative(Default)]
#[component(on_add = on_avoid_added, on_remove = on_avoid_removed)]
pub struct Avoid {
    /// The radius of the sphere used for shape casting
    #[derivative(Default(value = "0.5"))]
    pub cast_radius: f32,
    /// How far ahead to look for obstacles
    #[derivative(Default(value = "5.0"))]
    pub cast_distance: f32,
    /// The layer mask used for obstacle detection
    #[derivative(Default(value = "LayerMask::DEFAULT"))]
    pub avoid_mask: LayerMask,
    /// Entities to ignore when avoiding obstacles
    pub ignore_entites: Vec<Entity>,
}

impl Avoid {
    /// Set the radius of the sphere used to detect obstacles.
    pub fn with_cast_radius(mut self, radius: f32) -> Self {
        self.cast_radius = radius.max(0.01);
        self
    }

    /// Set the maximum distance to cast for obstacles.
    pub fn with_cast_distance(mut self, distance: f32) -> Self {
        self.cast_distance = distance.max(0.0);
        self
    }

    /// Set the layer mask used for obstacle detection. Only avoid
    /// obstacles that match the mask.
    pub fn with_avoid_mask(mut self, mask: LayerMask) -> Self {
        self.avoid_mask = mask;
        self
    }

    /// Ignore entities in this set. Do not avoid them.
    pub fn with_ignored_entities(mut self, entities: Vec<Entity>) -> Self {
        self.ignore_entites = entities;
        self
    }
}

/// Hook called when Avoid component is added to an entity.
/// Adds a ShapeCaster component for obstacle detection.
fn on_avoid_added(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    let avoid = world.get::<Avoid>(entity);
    if let Some(avoid) = avoid {
        let filter = SpatialQueryFilter::from_mask(avoid.avoid_mask)
            .with_excluded_entities(avoid.ignore_entites.clone());
        let shape_caster = ShapeCaster::new(
            Collider::sphere(avoid.cast_radius),
            Vec3::ZERO,
            Quat::IDENTITY,
            Dir3::NEG_Z, // Cast forward (local -Z is typically forward in Bevy)
        )
        .with_query_filter(filter)
        .with_max_distance(avoid.cast_distance)
        .with_max_hits(1);
        world.commands().entity(entity).insert(shape_caster);
    }
}

/// Hook called when Avoid component is removed from an entity.
/// Removes the associated ShapeCaster component.
fn on_avoid_removed(mut world: DeferredWorld, HookContext { entity, .. }: HookContext) {
    world
        .commands()
        .entity(entity)
        .remove::<(ShapeCaster, ShapeHits)>();
}

#[derive(QueryData)]
#[query_data(mutable)]
pub struct AvoidBehaviorAgentQuery {
    agent: &'static SteeringAgent,
    avoid: &'static Avoid,
    global_transform: &'static GlobalTransform,
    shape_hits: Option<&'static ShapeHits>,
    outputs: &'static mut SteeringOutputs,
}

pub(crate) fn run(mut query: Query<AvoidBehaviorAgentQuery>) {
    for mut agent in query.iter_mut() {
        let Some(shape_hits) = agent.shape_hits else {
            agent.outputs.clear(BehaviorType::Avoid);
            continue;
        };

        let Some(hit) = shape_hits.iter().next() else {
            agent.outputs.clear(BehaviorType::Avoid);
            continue;
        };

        // Get direction toward the obstacle
        let agent_position = agent.global_transform.translation();
        let to_obstacle = hit.point1 - agent_position;

        // Calculate intensity based on distance (closer = stronger)
        let direction = to_obstacle.normalize_or_zero();
        let mut target = SteeringTarget::default();

        // Set danger to avoid obstacle.
        target.add_danger(direction, 1.0);
        agent.outputs.set(BehaviorType::Avoid, target);
    }
}

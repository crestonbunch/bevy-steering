use avian3d::prelude::LayerMask;
use bevy::prelude::*;
use derivative::Derivative;
#[cfg(feature = "serialize")]
use serde::{Deserialize, Serialize};

use crate::{
    control::SteeringOutputs,
    neighbors::{Neighbor, Neighborhood},
    speed::SpeedController,
};

/// Determine how the agent will move. This will determine how
/// forces and torques can be applied to the agent. For car-like
/// agents, you should use the Directional mode. For simple
/// point-like agents, you should use the OmniDirectional mode.
#[derive(Copy, Clone, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
pub enum SteeringLocomotionMode {
    /// The agent can move in any direction, and forces can
    /// be applied in any direction. Torque will not be applied.
    #[default]
    OmniDirectional,
    /// Forces can only be applied along its forward/backward axis.
    /// The agent must turn to change direction. Torque will be applied
    /// to change its rotation to steer the agent.
    Directional,
}

/// Represents an autonomous agent. Agents are controlled by
/// steering behaviors. Add a [SteeringController] component
/// to the agent to give it steering behavior.
#[derive(Component, Clone, Debug, Reflect, Derivative)]
#[cfg_attr(feature = "serialize", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serialize", serde(default))]
#[derivative(Default)]
#[require(Neighbor, Neighborhood, SteeringOutputs, SpeedController)]
#[reflect(Component)]
pub struct SteeringAgent {
    /// How the agent will move through the world.
    pub(crate) locomotion_mode: SteeringLocomotionMode,
    /// The maximum speed of the agent. The agent will normally
    /// try to reach this speed unless it needs to slow down.
    #[derivative(Default(value = "10.0"))]
    pub(crate) max_speed: f32,
    /// The maximum force that can be applied to the agent. This
    /// represents, e.g. the power of the engine. If you want
    /// to disable movement along an axis, set the force to 0.
    #[derivative(Default(value = "Vec3::splat(100.0)"))]
    pub(crate) max_force: Vec3,
    /// The maximum torque that can be applied to the agent. This
    /// represents, e.g. the maximum turning force of the wheels.
    /// If you want to disable turning, set the torque to 0.
    #[derivative(Default(value = "Vec3::splat(100.0)"))]
    pub(crate) max_torque: Vec3,
    /// The radius of the neighborhood around the agent. This is
    /// used to determine which agents are considered neighbors.
    #[derivative(Default(value = "1.0"))]
    pub(crate) neighborhood_radius: f32,
    /// The filter used to determine which entities can be neighbors.
    /// Note that they must also have a [Neighbor] component. A good
    /// idea is to put all neighboars in the same layer.
    #[derivative(Default(value = "LayerMask::DEFAULT"))]
    pub(crate) neighborhood_filter: LayerMask,
    /// How much the agent prefers to turn, vs. moving forward (when
    /// it needs to turn.) This is equivalent to the proportion of
    /// lateral force that contributes to braking. I.e., if this is 0.0
    /// (default) then no braking is applied. But at 1.0 all lateral
    /// force is used to brake.
    #[derivative(Default(value = "0.0"))]
    pub(crate) turn_priority: f32,
}

impl SteeringAgent {
    /// Set the agent to use a directional locomotion mode.
    pub fn directional(self) -> Self {
        Self {
            locomotion_mode: SteeringLocomotionMode::Directional,
            ..self
        }
    }

    /// Set the agent to use an omni-directional locomotion mode (the default).
    pub fn omni_directional(self) -> Self {
        Self {
            locomotion_mode: SteeringLocomotionMode::OmniDirectional,
            ..self
        }
    }

    /// Set the maximum speed of the agent. The agent will normally
    /// try to reach this speed unless it needs to slow down.
    pub fn with_max_speed(self, speed: f32) -> Self {
        Self {
            max_speed: speed,
            ..self
        }
    }

    /// Set the maximum force that can be applied to the agent.
    /// This represents, e.g. the power of the engine. If you want
    /// to disable movement along an axis, set the force to 0.
    pub fn with_max_force(self, force: Vec3) -> Self {
        Self {
            max_force: force,
            ..self
        }
    }

    /// Set the maximum torque that can be applied to the agent.
    /// This represents, e.g. the maximum turning force of the wheels.
    /// If you want to disable turning, set the torque to 0.
    pub fn with_max_torque(self, torque: Vec3) -> Self {
        Self {
            max_torque: torque,
            ..self
        }
    }

    /// Set the radius of the neighborhood around the agent. This is
    /// used to determine which agents are considered neighbors.
    pub fn with_neighborhood_radius(self, radius: f32) -> Self {
        Self {
            neighborhood_radius: radius,
            ..self
        }
    }

    /// Set the filter used to determine which entities can be neighbors.
    /// Note that they must also have a [Neighbor] component. A good
    /// idea is to put all neighboars in the same layer.
    pub fn with_neighborhood_filter(self, filter: LayerMask) -> Self {
        Self {
            neighborhood_filter: filter,
            ..self
        }
    }

    /// Set the turn priority of the agent. This controls how much the agent
    /// prefers to turn vs. moving forward (when it needs to turn). This is
    /// equivalent to the proportion of lateral forces that contributes to
    /// braking. I.e., if this is 0.0 (default) the non-braking forces are
    /// applied. But at 1.0 all lateral forces are used to brake.
    pub fn with_turn_priority(self, priority: f32) -> Self {
        Self {
            turn_priority: priority,
            ..self
        }
    }
}

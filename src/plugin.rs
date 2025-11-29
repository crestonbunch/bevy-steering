use bevy::prelude::*;

use crate::{
    behaviors::{
        alignment, approach, avoid, cohesion,
        evasion::{self as evasion_behavior, debug_evasion},
        flee, path_following,
        pursuit::{self, debug_pursuit},
        seek,
        separation::{self, debug_separation},
        wander,
    },
    control::{
        combine_steering_targets, debug_combined_steering, temporal_smoothing,
        update_previous_steering_outputs,
    },
    movement::{debug_movement, move_agent},
    neighbors::{debug_neighborhoods, update_neighborhoods},
};

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct AgentsSystem;

pub struct AgentsPlugin;

impl Plugin for AgentsPlugin {
    fn build(&self, app: &mut App) {
        let update_systems = (
            update_neighborhoods,
            alignment::run,
            approach::run,
            avoid::run,
            cohesion::run,
            evasion_behavior::run,
            flee::run,
            path_following::run,
            pursuit::run,
            seek::run,
            separation::run,
            wander::run,
            temporal_smoothing,
            update_previous_steering_outputs,
            combine_steering_targets,
            move_agent,
        )
            .chain()
            .in_set(AgentsSystem);
        app.add_systems(FixedUpdate, update_systems);
    }
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct DebugAgentsSystem;

pub struct DebugAgentsPlugin;

impl Plugin for DebugAgentsPlugin {
    fn build(&self, app: &mut App) {
        let debug_systems = (
            debug_neighborhoods,
            debug_separation,
            debug_evasion,
            debug_pursuit,
            debug_combined_steering,
            debug_movement,
        )
            .chain()
            .in_set(DebugAgentsSystem);
        app.add_systems(Update, debug_systems);
    }
}

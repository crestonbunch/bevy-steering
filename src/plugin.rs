use bevy::prelude::*;
use bitflags::bitflags;

use crate::{
    behaviors::{
        alignment,
        approach::{self, debug_approach},
        avoid, cohesion,
        evasion::{self as evasion_behavior, debug_evasion},
        flee,
        path_following::{self, debug_path_following},
        pursuit::{self, debug_pursuit},
        seek,
        separation::{self, debug_separation},
        wander,
    },
    control::{
        combine_steering_targets, debug_combined_steering, debug_forward_dir, temporal_smoothing,
        update_forward_dir, update_previous_steering_outputs,
    },
    movement::{debug_movement, move_agent},
    neighbors::{debug_neighborhoods, update_neighborhoods},
    obstacles::{
        debug_obstacles, setup_obstacle_tracking, update_nearby_obstacles, update_obstacle_tracking,
    },
    speed::{debug_speed, reset_speed_override, speed_control},
};

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct BehaviorSystemSet;

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct SteeringSystemSet;

pub struct SteeringPlugin;

impl Plugin for SteeringPlugin {
    fn build(&self, app: &mut App) {
        let behavior_systems = (
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
        )
            .in_set(BehaviorSystemSet);
        let update_systems = (
            update_forward_dir,
            setup_obstacle_tracking,
            update_obstacle_tracking,
            update_nearby_obstacles,
            update_neighborhoods,
            behavior_systems,
            speed_control,
            temporal_smoothing,
            update_previous_steering_outputs,
            combine_steering_targets,
            move_agent,
        )
            .chain()
            .in_set(SteeringSystemSet);
        app.add_systems(FixedPreUpdate, reset_speed_override);
        app.add_systems(FixedUpdate, update_systems);
    }
}

#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub struct DebugSteeringSystem;

bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DebugSteeringFlags: u32 {
        const APPROACH          = 1 << 0;
        const FORWARD_DIR       = 1 << 1;
        const OBSTACLES         = 1 << 2;
        const NEIGHBORHOODS     = 1 << 3;
        const SEPARATION        = 1 << 4;
        const EVASION           = 1 << 5;
        const PURSUIT           = 1 << 6;
        const PATH_FOLLOWING    = 1 << 7;
        const SPEED             = 1 << 8;
        const COMBINED_STEERING = 1 << 9;
        const MOVEMENT          = 1 << 10;
    }
}

#[derive(Resource, Debug, Clone)]
pub struct DebugSteeringConfig {
    pub flags: DebugSteeringFlags,
}

impl Default for DebugSteeringConfig {
    fn default() -> Self {
        Self {
            flags: DebugSteeringFlags::all(),
        }
    }
}

fn flag_enabled(flag: DebugSteeringFlags) -> impl Fn(Res<DebugSteeringConfig>) -> bool {
    move |config: Res<DebugSteeringConfig>| config.flags.contains(flag)
}

pub struct DebugSteeringPlugin;

impl Plugin for DebugSteeringPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugSteeringConfig>();

        app.add_systems(
            FixedUpdate,
            (
                debug_approach.run_if(flag_enabled(DebugSteeringFlags::APPROACH)),
                debug_forward_dir.run_if(flag_enabled(DebugSteeringFlags::FORWARD_DIR)),
                debug_obstacles.run_if(flag_enabled(DebugSteeringFlags::OBSTACLES)),
                debug_neighborhoods.run_if(flag_enabled(DebugSteeringFlags::NEIGHBORHOODS)),
                debug_separation.run_if(flag_enabled(DebugSteeringFlags::SEPARATION)),
                debug_evasion.run_if(flag_enabled(DebugSteeringFlags::EVASION)),
                debug_pursuit.run_if(flag_enabled(DebugSteeringFlags::PURSUIT)),
                debug_path_following.run_if(flag_enabled(DebugSteeringFlags::PATH_FOLLOWING)),
                debug_speed.run_if(flag_enabled(DebugSteeringFlags::SPEED)),
                debug_combined_steering.run_if(flag_enabled(DebugSteeringFlags::COMBINED_STEERING)),
                debug_movement.run_if(flag_enabled(DebugSteeringFlags::MOVEMENT)),
            )
                .in_set(DebugSteeringSystem),
        );
    }
}

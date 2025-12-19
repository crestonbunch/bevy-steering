use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

/// Marker component for the pursuing threat
#[derive(Component)]
struct Pursuer;

/// Marker component for the evading agent
#[derive(Component)]
struct Evader;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(DebugSteeringPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Spawn pursuers first to get their entity IDs

    // First pursuer (red cylinder) - positioned on the right side
    let pursuer_agent = SteeringAgent::default()
        .with_max_speed(4.5)
        .with_max_force(Vec3::splat(10.0));

    let pursuer1_entity = commands
        .spawn((
            Pursuer,
            pursuer_agent,
            Transform::from_translation(Vec3::new(15.0, 0.0, -5.0)),
            RigidBody::Dynamic,
            Collider::cylinder(0.5, 0.5),
            Mesh3d(meshes.add(Cylinder::new(0.5, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.9, 0.2, 0.2),
                emissive: LinearRgba::rgb(1.0, 0.0, 0.0),
                ..default()
            })),
        ))
        .id();

    // Second pursuer (orange cuboid) - positioned on the left side
    let pursuer_agent2 = SteeringAgent::default()
        .with_max_speed(4.8)
        .with_max_force(Vec3::splat(10.0))
        .with_max_torque(Vec3::splat(10.0))
        .directional();

    let pursuer2_entity = commands
        .spawn((
            Pursuer,
            pursuer_agent2,
            Transform::from_translation(Vec3::new(15.0, 0.0, -15.0))
                .looking_at(Vec3::ZERO, Vec3::Y),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 1.0),
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.5, 0.0),
                emissive: LinearRgba::rgb(0.8, 0.3, 0.0),
                ..default()
            })),
        ))
        .id();

    // Now spawn evading agents with evasion behaviors

    // First evader (blue sphere - omnidirectional) evades from pursuer1 - on right side
    let evader_agent = SteeringAgent::default()
        .with_max_force(Vec3::splat(12.0))
        .with_max_speed(5.0);

    let evader1_entity = commands
        .spawn((
            Evader,
            evader_agent,
            Evasion::new(pursuer1_entity)
                .with_radius(5.0)
                .with_max_prediction(1.0),
            Transform::from_translation(Vec3::new(3.0, 0.0, 5.0)),
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.2, 0.6, 0.9),
                ..default()
            })),
        ))
        .id();

    // Second evader (green cuboid - directional) evades from pursuer2 - on left side
    let evader_agent2 = SteeringAgent::default()
        .with_max_speed(5.5)
        .with_max_force(Vec3::splat(12.0))
        .with_max_torque(Vec3::splat(12.0))
        .directional();

    let evader2_entity = commands
        .spawn((
            Evader,
            evader_agent2,
            Evasion::new(pursuer2_entity)
                .with_radius(10.0)
                .with_max_prediction(1.0),
            Transform::from_translation(Vec3::new(-8.0, 0.0, 9.0))
                .looking_at(Vec3::new(-10.0, 0.0, 0.0), Vec3::Y),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 1.0, 1.0),
            Mesh3d(meshes.add(Cuboid::new(0.5, 1.0, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.2, 0.9, 0.4),
                ..default()
            })),
        ))
        .id();

    // Add pursuit behaviors to pursuers now that we have evader entities
    commands
        .entity(pursuer1_entity)
        .insert(Pursuit::new(evader1_entity).with_radius(0.5));
    commands
        .entity(pursuer2_entity)
        .insert(Pursuit::new(evader2_entity).with_radius(0.5));

    // Platform
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(25.0, 25.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.6, 0.0),
        RigidBody::Static,
        Collider::cuboid(50.0, 0.2, 50.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 35.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AgentsPlugin)
        .add_plugins(DebugAgentsPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane (rectangular mesh)
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(50.0, 50.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.1, 0.0),
        RigidBody::Static,
        Collider::cuboid(50.0, 0.2, 50.0),
    ));

    // Red cuboid
    let agent1 = SteeringAgent::default()
        .with_neighborhood_radius(5.0)
        .with_max_force(Vec3::splat(1.0))
        .directional();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            ..default()
        })),
        Transform::from_xyz(-5.0, 0.5, -10.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 1.0),
        Align::default(),
        Seek::new(Vec3::new(0.0, 0.5, 0.0), 0.5),
        agent1,
    ));

    // Blue cuboid
    let agent2 = SteeringAgent::default()
        .with_neighborhood_radius(10.0)
        .with_max_force(Vec3::splat(1.0))
        .directional();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.8),
            ..default()
        })),
        Transform::from_xyz(10.0, 0.5, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 1.0),
        Align::default(),
        Seek::new(Vec3::new(0.0, 0.5, 0.0), 0.5),
        agent2,
    ));

    // Green cuboid
    let agent3 = SteeringAgent::default()
        .with_neighborhood_radius(10.0)
        .with_max_force(Vec3::splat(1.0))
        .directional();
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
        Transform::from_xyz(-10.0, 0.5, 3.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 0.5, 1.0),
        Align::default(),
        Seek::new(Vec3::new(0.0, 0.5, 0.0), 0.5),
        agent3,
    ));

    // Top-down camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 15.0, 0.1).looking_at(Vec3::ZERO, Vec3::Z),
    ));

    // Directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(DebugSteeringPlugin)
        .add_plugins(PhysicsDebugPlugin)
        .add_systems(Startup, setup)
        .run();
}

#[derive(Default, PhysicsLayer)]
enum GameLayers {
    #[default]
    Default,
    Agent,
    Obstacle,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane
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

    // Obstacle 1 - Wall on the left
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 2.0, 8.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.6, 0.6),
            ..default()
        })),
        Transform::from_xyz(-5.0, 1.0, 0.0),
        RigidBody::Static,
        Collider::cuboid(1.0, 2.0, 8.0),
        CollisionLayers::new([GameLayers::Obstacle], GameLayers::Agent),
    ));

    // Obstacle 2 - Wall on the right
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 2.0, 8.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.6, 0.6),
            ..default()
        })),
        Transform::from_xyz(5.0, 1.0, 0.0),
        RigidBody::Static,
        Collider::cuboid(1.0, 2.0, 8.0),
        CollisionLayers::new([GameLayers::Obstacle], GameLayers::Agent),
    ));

    // Obstacle 3 - Box in the middle
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 2.0, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.4, 0.4),
            ..default()
        })),
        Transform::from_xyz(0.0, 1.0, -3.0),
        RigidBody::Static,
        Collider::cuboid(2.0, 2.0, 2.0),
        CollisionLayers::new([GameLayers::Obstacle], GameLayers::Agent),
    ));

    // Obstacle 4 - Another box
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(2.0, 2.0, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.7, 0.4, 0.4),
            ..default()
        })),
        Transform::from_xyz(2.0, 1.0, 3.0),
        RigidBody::Static,
        Collider::cuboid(2.0, 2.0, 2.0),
        CollisionLayers::new([GameLayers::Obstacle], GameLayers::Agent),
    ));

    // Agent with Avoid and Seek behaviors
    // The agent will try to reach the target while avoiding obstacles
    let agent = SteeringAgent::default()
        .with_max_force(Vec3::splat(5.0))
        .directional();

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.8, 0.8, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
        Transform::from_xyz(-8.0, 0.5, -8.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.8, 0.8, 1.5),
        Avoid::default()
            .with_cast_radius(0.5)
            .with_cast_distance(5.0)
            .with_avoid_mask(GameLayers::Obstacle.into()),
        Approach::new(Vec3::new(8.0, 0.5, 8.0), 1.0),
        CollisionLayers::new(
            [GameLayers::Agent],
            [GameLayers::Default, GameLayers::Obstacle],
        ),
        agent.clone(),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.8, 0.8, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
        Transform::from_xyz(-8.0, 0.5, 0.0),
        RigidBody::Dynamic,
        Collider::cuboid(0.8, 0.8, 1.5),
        Avoid::default()
            .with_cast_radius(0.5)
            .with_cast_distance(10.0)
            .with_avoid_mask(GameLayers::Obstacle.into()),
        Approach::new(Vec3::new(8.0, 0.5, 8.0), 1.0),
        CollisionLayers::new(
            [GameLayers::Agent],
            [GameLayers::Default, GameLayers::Obstacle],
        ),
        agent,
    ));

    // Target marker (where the agent is trying to go)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 1.0, 0.0),
            emissive: LinearRgba::new(1.0, 1.0, 0.0, 1.0),
            ..default()
        })),
        Transform::from_xyz(8.0, 0.5, 8.0),
    ));

    // Top-down camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 20.0, 0.1).looking_at(Vec3::ZERO, Vec3::Z),
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

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

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default,
    Obstacle,
}

/// Helper function to spawn an obstacle
fn spawn_obstacle(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
    size: Vec3,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.5),
            ..default()
        })),
        Transform::from_translation(position),
        RigidBody::Static,
        Collider::cuboid(size.x, size.y, size.z),
        CollisionLayers::new([GameLayer::Obstacle], [LayerMask::ALL]),
    ));
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create several agents with different wander parameters

    // Agent 1 - Smooth, wide wandering
    let agent1 = SteeringAgent::default()
        .with_max_force(Vec3::splat(10.0))
        .with_max_speed(5.0);

    commands.spawn((
        agent1,
        Wander::default().with_radius(3.0).with_distance(4.0),
        Avoid::default()
            .with_cast_radius(0.6)
            .with_cast_distance(6.0)
            .with_avoid_mask(GameLayer::Obstacle.into()),
        Transform::from_translation(Vec3::new(-8.0, 0.0, 0.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        CollisionLayers::new([LayerMask::ALL], [LayerMask::ALL]),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            ..default()
        })),
    ));

    // Agent 2 - Tight, jittery wandering
    let agent2 = SteeringAgent::default()
        .with_max_force(Vec3::splat(15.0))
        .with_max_speed(6.0);

    commands.spawn((
        agent2,
        Wander::default().with_radius(1.0).with_distance(1.0),
        Avoid::default()
            .with_cast_radius(0.6)
            .with_cast_distance(7.5)
            .with_avoid_mask(GameLayer::Obstacle.into()),
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        CollisionLayers::new([LayerMask::ALL], [LayerMask::ALL]),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
    ));

    // Agent 3 - Directional agent with moderate wandering
    let agent3 = SteeringAgent::default()
        .with_max_speed(4.0)
        .with_max_force(Vec3::splat(12.0))
        .with_max_torque(Vec3::splat(1.0))
        .directional();

    commands.spawn((
        agent3,
        Wander::default(), // Default parameters
        Avoid::default()
            .with_cast_radius(0.7)
            .with_cast_distance(6.0)
            .with_avoid_mask(GameLayer::Obstacle.into()),
        Transform::from_translation(Vec3::new(8.0, 0.0, 0.0))
            .looking_at(Vec3::new(10.0, 0.0, 0.0), Vec3::Y),
        RigidBody::Dynamic,
        Collider::cuboid(0.4, 0.8, 0.8),
        CollisionLayers::new([LayerMask::ALL], [LayerMask::ALL]),
        Mesh3d(meshes.add(Cuboid::new(0.4, 0.8, 0.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.8),
            ..default()
        })),
    ));

    // Agent 4 - Fast wandering with large distance
    let agent4 = SteeringAgent::default()
        .with_max_force(Vec3::splat(20.0))
        .with_max_speed(8.0);

    commands.spawn((
        agent4,
        Wander::default().with_radius(1.0).with_distance(5.0),
        Avoid::default()
            .with_cast_radius(0.6)
            .with_cast_distance(6.0)
            .with_avoid_mask(GameLayer::Obstacle.into()),
        Transform::from_translation(Vec3::new(-8.0, 0.0, -8.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        CollisionLayers::new([LayerMask::ALL], [LayerMask::ALL]),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.8, 0.2),
            ..default()
        })),
    ));

    // Add obstacles scattered around the area
    let obstacle_height = 2.0;
    let wall_thickness = 0.5;
    let arena_size = 30.0;
    let half_arena = arena_size / 2.0;

    // Boundary walls to contain the agents in a 30x30 box
    // North wall
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, obstacle_height / 2.0, -half_arena),
        Vec3::new(arena_size, obstacle_height, wall_thickness),
    );
    // South wall
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, obstacle_height / 2.0, half_arena),
        Vec3::new(arena_size, obstacle_height, wall_thickness),
    );
    // West wall
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-half_arena, obstacle_height / 2.0, 0.0),
        Vec3::new(wall_thickness, obstacle_height, arena_size),
    );
    // East wall
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(half_arena, obstacle_height / 2.0, 0.0),
        Vec3::new(wall_thickness, obstacle_height, arena_size),
    );

    // Central cluster of obstacles (moved to avoid spawning on Agent 2)
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(1.5, obstacle_height / 2.0, -6.0),
        Vec3::new(3.0, obstacle_height, 3.0),
    );

    // Corner obstacles
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-10.0, obstacle_height / 2.0, -10.0),
        Vec3::new(2.0, obstacle_height, 2.0),
    );
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(10.0, obstacle_height / 2.0, -10.0),
        Vec3::new(2.0, obstacle_height, 2.0),
    );
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-10.0, obstacle_height / 2.0, 10.0),
        Vec3::new(2.0, obstacle_height, 2.0),
    );
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(10.0, obstacle_height / 2.0, 10.0),
        Vec3::new(2.0, obstacle_height, 2.0),
    );

    // Wall-like obstacles
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-5.0, obstacle_height / 2.0, 5.0),
        Vec3::new(0.5, obstacle_height, 5.0),
    );
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(5.0, obstacle_height / 2.0, -5.0),
        Vec3::new(5.0, obstacle_height, 0.5),
    );

    // Scattered smaller obstacles
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-3.0, obstacle_height / 2.0, -3.0),
        Vec3::new(1.5, obstacle_height, 1.5),
    );
    spawn_obstacle(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(6.0, obstacle_height / 2.0, 3.0),
        Vec3::new(1.5, obstacle_height, 1.5),
    );

    // Platform
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(40.0, 40.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.6, 0.0),
        RigidBody::Static,
        Collider::cuboid(50.0, 0.2, 50.0),
    ));

    // Camera positioned to view from above
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 40.0, 0.0).looking_at(Vec3::ZERO, Vec3::NEG_Z),
    ));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

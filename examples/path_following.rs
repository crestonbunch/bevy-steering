use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(DebugSteeringPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, draw_path_gizmos)
        .run();
}

fn draw_path_gizmos(mut gizmos: Gizmos, query: Query<&PathFollowing>) {
    for path_following in query.iter() {
        let path = path_following.path();

        // Draw the path spine as connected line segments
        for i in 0..path.len().saturating_sub(1) {
            gizmos.line(path[i], path[i + 1], Color::srgb(0.0, 0.8, 1.0));
        }

        // Draw spheres at each path point
        for point in path {
            gizmos.sphere(*point, 0.3, Color::srgb(0.0, 0.6, 1.0));
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create a winding path
    let path = vec![
        Vec3::new(-10.0, 0.0, -10.0),
        Vec3::new(-5.0, 0.0, -5.0),
        Vec3::new(0.0, 0.0, -8.0),
        Vec3::new(5.0, 0.0, -5.0),
        Vec3::new(8.0, 0.0, 0.0),
        Vec3::new(5.0, 0.0, 5.0),
        Vec3::new(0.0, 0.0, 8.0),
        Vec3::new(-5.0, 0.0, 5.0),
        Vec3::new(-10.0, 0.0, 0.0),
        Vec3::new(-10.0, 0.0, -10.0), // Loop back to start
    ];

    let agent1 = SteeringAgent::default()
        .with_max_force(Vec3::splat(15.0))
        .with_max_speed(5.0);

    let agent2 = SteeringAgent::default()
        .with_max_speed(7.0)
        .with_max_force(Vec3::splat(20.0))
        .with_max_torque(Vec3::splat(15.0))
        .directional();

    // Agent 1 - Omnidirectional (sphere)
    commands.spawn((
        agent1,
        PathFollowing::new(path.clone()),
        Transform::from_translation(Vec3::new(-10.0, 0.0, -10.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            ..default()
        })),
    ));

    // Agent 2 - Directional (cuboid)
    commands.spawn((
        agent2,
        PathFollowing::new(path.clone()),
        Transform::from_translation(Vec3::new(-5.0, 0.0, -5.0))
            .looking_at(Vec3::new(0.0, 0.0, -8.0), Vec3::Y),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 1.0, 1.0),
        Mesh3d(meshes.add(Cuboid::new(0.5, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
    ));

    // Platform
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(30.0, 30.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.6, 0.0),
        RigidBody::Static,
        Collider::cuboid(50.0, 0.2, 50.0),
    ));

    // Camera positioned to view the path from above
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 35.0, 0.0).looking_at(Vec3::ZERO, Vec3::NEG_Z),
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

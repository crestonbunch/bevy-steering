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
        .add_systems(Startup, (setup, configure_gizmos))
        .insert_resource(DebugSteeringConfig {
            flags: DebugSteeringFlags::OBSTACLES | DebugSteeringFlags::SPEED,
        })
        .run();
}

#[derive(Default, PhysicsLayer)]
enum GameLayers {
    #[default]
    Default,
    Agent,
    Obstacle,
}

fn configure_gizmos(mut config_store: ResMut<GizmoConfigStore>) {
    let (config, _) = config_store.config_mut::<DefaultGizmoConfigGroup>();
    config.depth_bias = -1.0;
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

    // Large cylinder obstacle in the center
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(1.5, 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.3, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, 1.0, 0.0),
        RigidBody::Static,
        Collider::cylinder(1.5, 2.0),
        CollisionLayers::new([GameLayers::Obstacle], GameLayers::Agent),
    ));

    // Agent configuration
    let agent = SteeringAgent::default()
        .with_max_speed(2.0)
        .with_max_force(Vec3::splat(50.0))
        .directional();

    // Body mesh (main chassis)
    let body_mesh = meshes.add(Cuboid::new(1.0, 0.4, 1.6));
    let body_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.6, 0.8),
        ..default()
    });

    // Wheel mesh and material
    let wheel_mesh = meshes.add(Cylinder::new(0.25, 0.15));
    let wheel_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.15, 0.15, 0.15),
        ..default()
    });

    // Collision layers for agents
    let agent_collision_layers = CollisionLayers::new(
        [GameLayers::Agent],
        [GameLayers::Default, GameLayers::Obstacle, GameLayers::Agent],
    );

    // Target position (straight ahead from the agents, past the obstacle)
    let target = Vec3::new(0.0, 0.5, 10.0);

    // Spawn 5 identical agents in a line along their forward direction
    for i in 0..5 {
        let z = -8.0 - (i as f32 * 3.5); // Spread agents in a column behind each other
        commands
            .spawn((
                Mesh3d(body_mesh.clone()),
                MeshMaterial3d(body_material.clone()),
                Transform::from_xyz(0.0, 0.5, z),
                // .with_rotation(Quat::from_rotation_y(std::f32::consts::PI)),
                RigidBody::Dynamic,
                Collider::cuboid(1.0, 0.4, 1.6),
                Avoid::default().with_distance(2.0),
                TrackNearbyObstacles::default()
                    .with_distance(4.0)
                    .with_avoid_mask([GameLayers::Obstacle, GameLayers::Agent].into()),
                SpeedController::default().with_stopping_distance(4.0),
                Approach::new(target, 1.0),
                agent_collision_layers,
                agent.clone(),
            ))
            .with_children(|parent| {
                // Front-left wheel
                parent.spawn((
                    Mesh3d(wheel_mesh.clone()),
                    MeshMaterial3d(wheel_material.clone()),
                    Transform::from_xyz(-0.6, -0.15, 0.5)
                        .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    Collider::cylinder(0.25, 0.15),
                    agent_collision_layers,
                ));
                // Front-right wheel
                parent.spawn((
                    Mesh3d(wheel_mesh.clone()),
                    MeshMaterial3d(wheel_material.clone()),
                    Transform::from_xyz(0.6, -0.15, 0.5)
                        .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    Collider::cylinder(0.25, 0.15),
                    agent_collision_layers,
                ));
                // Back-left wheel
                parent.spawn((
                    Mesh3d(wheel_mesh.clone()),
                    MeshMaterial3d(wheel_material.clone()),
                    Transform::from_xyz(-0.6, -0.15, -0.5)
                        .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    Collider::cylinder(0.25, 0.15),
                    agent_collision_layers,
                ));
                // Back-right wheel
                parent.spawn((
                    Mesh3d(wheel_mesh.clone()),
                    MeshMaterial3d(wheel_material.clone()),
                    Transform::from_xyz(0.6, -0.15, -0.5)
                        .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2)),
                    Collider::cylinder(0.25, 0.15),
                    agent_collision_layers,
                ));
            });
    }

    // Target marker (where agents are trying to go)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 1.0, 0.0),
            emissive: LinearRgba::new(1.0, 1.0, 0.0, 1.0),
            ..default()
        })),
        Transform::from_translation(target),
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

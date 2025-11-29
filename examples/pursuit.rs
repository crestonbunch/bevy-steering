use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

/// Marker for Agent A (seeks random waypoints, pursued by B)
#[derive(Component)]
struct AgentA;

/// Marker for Agent B (pursues A, pursued by C)
#[derive(Component)]
struct AgentB;

/// Marker for Agent C (pursues B)
#[derive(Component)]
struct AgentC;

/// Timer for changing Agent A's waypoint
#[derive(Resource)]
struct WaypointChangeTimer(Timer);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AgentsPlugin)
        .add_plugins(DebugAgentsPlugin)
        .insert_resource(WaypointChangeTimer(Timer::from_seconds(
            3.0,
            TimerMode::Repeating,
        )))
        .add_systems(Startup, setup)
        .add_systems(Update, update_agent_a_waypoint)
        .run();
}

/// Updates Agent A's seek target to random positions periodically
fn update_agent_a_waypoint(
    time: Res<Time>,
    mut timer: ResMut<WaypointChangeTimer>,
    mut query: Query<&mut Seek, With<AgentA>>,
) {
    timer.0.tick(time.delta());

    if timer.0.just_finished() {
        for mut seek in query.iter_mut() {
            // Generate random position within bounds
            let x = (rand::random::<f32>() - 0.5) * 30.0; // -15 to 15
            let z = (rand::random::<f32>() - 0.5) * 30.0; // -15 to 15
            let new_waypoint = Vec3::new(x, 0.0, z);
            seek.set_target(new_waypoint);
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Agent A - seeks random waypoints (blue sphere)
    let agent_a = SteeringAgent::default()
        .with_max_force(Vec3::splat(8.0))
        .with_max_speed(4.0);

    let entity_a = commands
        .spawn((
            AgentA,
            agent_a,
            Seek::new(Vec3::new(10.0, 0.0, 10.0), 0.5),
            Transform::from_translation(Vec3::new(5.0, 0.0, 5.0)),
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.2, 0.2, 0.9),
                ..default()
            })),
        ))
        .id();

    // Agent B - pursues A (green cuboid)
    let agent_b = SteeringAgent::default()
        .with_max_speed(5.0)
        .with_max_force(Vec3::splat(10.0))
        .with_max_torque(Vec3::splat(10.0))
        .directional();

    let entity_b = commands
        .spawn((
            AgentB,
            agent_b,
            Pursuit::new(entity_a)
                .with_max_prediction(5.0)
                .with_offset(10.0)
                .with_radius(1.5),
            Transform::from_translation(Vec3::new(-5.0, 0.0, 5.0)).looking_at(Vec3::ZERO, Vec3::Y),
            RigidBody::Dynamic,
            Collider::cuboid(0.5, 0.5, 1.0),
            Mesh3d(meshes.add(Cuboid::new(0.5, 0.5, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.2, 0.9, 0.2),
                ..default()
            })),
        ))
        .id();

    // Agent C - pursues B (red cylinder)
    let agent_c = SteeringAgent::default()
        .with_max_speed(5.0)
        .with_max_force(Vec3::splat(15.0));

    commands.spawn((
        AgentC,
        agent_c,
        Pursuit::new(entity_b)
            .with_radius(2.0)
            .with_max_prediction(5.0),
        Transform::from_translation(Vec3::new(-10.0, 0.0, -5.0)),
        RigidBody::Dynamic,
        Collider::cylinder(0.5, 0.5),
        Mesh3d(meshes.add(Cylinder::new(0.5, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.2, 0.2),
            ..default()
        })),
    ));

    // Platform
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(20.0, 20.0)))),
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
        Transform::from_xyz(0.0, 30.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
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

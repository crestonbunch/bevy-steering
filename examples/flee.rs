use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

/// Radius within which the agent will flee from the threat
const FLEE_RADIUS: f32 = 9.0;

/// Marker component for the threat that the agent flees from
#[derive(Component)]
struct Threat;

/// Marker component for the fleeing agent
#[derive(Component)]
struct FleeingAgent;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(AgentsPlugin)
        .add_plugins(DebugAgentsPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (move_threat, update_flee_target, draw_flee_gizmos))
        .run();
}

/// Updates the flee behavior to flee from the threat's position
fn update_flee_target(
    mut commands: Commands,
    threat_query: Query<&GlobalTransform, With<Threat>>,
    agent_query: Query<Entity, With<FleeingAgent>>,
) {
    let Ok(threat_transform) = threat_query.single() else {
        return;
    };

    let threat_pos = threat_transform.translation();

    for entity in agent_query.iter() {
        commands
            .entity(entity)
            .insert(Flee::new(threat_pos).with_radius(FLEE_RADIUS));
    }
}

/// Moves the threat in a circular pattern
fn move_threat(time: Res<Time>, mut query: Query<&mut Transform, With<Threat>>) {
    for mut transform in query.iter_mut() {
        let t = time.elapsed_secs();
        let radius = 8.0;
        transform.translation.x = radius * (t * 0.5).cos();
        transform.translation.z = radius * (t * 0.5).sin();
    }
}

/// Draws visual indicators for the flee radius
fn draw_flee_gizmos(mut gizmos: Gizmos, threat_query: Query<&GlobalTransform, With<Threat>>) {
    for transform in threat_query.iter() {
        let pos = transform.translation();
        // Draw the flee radius as a red circle on the ground using line segments
        let segments = 32;
        for i in 0..segments {
            let angle1 = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let angle2 = ((i + 1) as f32 / segments as f32) * std::f32::consts::TAU;
            let p1 = pos + Vec3::new(angle1.cos() * FLEE_RADIUS, 0.0, angle1.sin() * FLEE_RADIUS);
            let p2 = pos + Vec3::new(angle2.cos() * FLEE_RADIUS, 0.0, angle2.sin() * FLEE_RADIUS);
            gizmos.line(p1, p2, Color::srgb(1.0, 0.3, 0.3));
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Fleeing agent with omnidirectional movement
    let agent1 = SteeringAgent::default()
        .with_max_force(Vec3::splat(15.0))
        .with_max_speed(6.0);

    // Fleeing agent with directional movement (rotates to face away)
    let agent2 = SteeringAgent::default()
        .with_max_speed(6.0)
        .with_max_force(Vec3::splat(15.0))
        .with_max_torque(Vec3::splat(15.0))
        .directional();

    // Spawn first fleeing agent (sphere - omnidirectional)
    commands.spawn((
        FleeingAgent,
        agent1,
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
    ));

    // Spawn second fleeing agent (cuboid - directional)
    commands.spawn((
        FleeingAgent,
        agent2,
        Transform::from_translation(Vec3::new(-3.0, 0.0, 3.0)).looking_at(Vec3::ZERO, Vec3::Y),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 1.0, 1.0),
        Mesh3d(meshes.add(Cuboid::new(0.5, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.8, 0.2),
            ..default()
        })),
    ));

    // Spawn the threat
    commands.spawn((
        Threat,
        Transform::from_translation(Vec3::new(8.0, 0.0, 0.0)),
        RigidBody::Kinematic,
        Collider::sphere(0.7),
        Mesh3d(meshes.add(Sphere::new(0.7))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.1, 0.1),
            emissive: LinearRgba::rgb(2.0, 0.0, 0.0),
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

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
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create several complex agents at different positions
    let agent_positions = [
        Vec3::new(4.0, 0.0, 0.0),
        Vec3::new(0.0, 4.0, 0.0),
        Vec3::new(0.0, 0.0, 4.0),
    ];

    let colors = [
        Color::srgb(0.8, 0.2, 0.2), // Red
        Color::srgb(0.2, 0.2, 0.8), // Blue
        Color::srgb(0.2, 0.8, 0.2), // Green
    ];

    for (position, color) in agent_positions.iter().zip(colors.iter()) {
        spawn_complex_agent(
            &mut commands,
            &mut meshes,
            &mut materials,
            *position,
            *color,
        );
    }

    // Camera - positioned to view the scene from an angle
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(10.0, 12.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
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

fn spawn_complex_agent(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
    color: Color,
) {
    let mut entity = commands.spawn((
        SteeringAgent::default().with_neighborhood_radius(5.0),
        Transform::from_translation(position),
        RigidBody::Static,
    ));

    // Add two child colliders with visual meshes
    entity.with_child((
        Transform::from_translation(Vec3::new(0.0, -0.5, 0.0)),
        Collider::sphere(0.5),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            ..default()
        })),
    ));

    entity.with_child((
        Transform::from_translation(Vec3::new(0.0, 0.5, 0.0)),
        Collider::sphere(0.5),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            ..default()
        })),
    ));
}

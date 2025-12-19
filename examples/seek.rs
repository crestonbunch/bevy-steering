use avian3d::prelude::*;
use bevy::prelude::*;
use bevy_steering::prelude::*;

/// How close the agent needs to be to the waypoint to consider it reached
const TOLERANCE: f32 = 0.5;

/// A component to track the current waypoint index
#[derive(Component, Deref, DerefMut)]
struct CurrentWaypoint(usize);

/// A component to store the waypoints
#[derive(Component)]
struct Waypoints(Vec<Vec3>);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(DebugSteeringPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (update_seek_target, draw_waypoint_gizmos))
        .run();
}

fn update_seek_target(
    mut commands: Commands,
    mut query: Query<(Entity, &GlobalTransform, &Waypoints, &mut CurrentWaypoint)>,
) {
    for query_item in query.iter_mut() {
        let (entity, transform, Waypoints(waypoints), mut w) = query_item;
        let Some(waypoint) = waypoints.get(**w) else {
            continue;
        };
        // Set the seek target to the current waypoint
        commands
            .entity(entity)
            .insert(Seek::new(*waypoint, TOLERANCE));

        // Move to the next waypoint when the agent is close to the current waypoint
        let distance = (transform.translation() - waypoint).length();
        if distance < TOLERANCE {
            **w += 1;
            **w %= waypoints.len();
        }
    }
}

fn draw_waypoint_gizmos(mut gizmos: Gizmos, query: Query<&Waypoints>) {
    for Waypoints(waypoints) in query.iter() {
        for waypoint in waypoints {
            gizmos.sphere(*waypoint, 0.5, Color::srgb(1.0, 0.0, 0.0));
        }
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let agent1 = SteeringAgent::default()
        .with_max_force(Vec3::splat(10.0))
        .with_max_speed(5.0);

    let agent2 = SteeringAgent::default()
        // If you have very high mass, you may need to increase
        // the maximum force and torque for them to move. The default
        // values are pretty modest.
        .with_max_speed(5.0)
        .with_max_force(Vec3::splat(10.0))
        .with_max_torque(Vec3::splat(10.0))
        // Set the agent to use a directional locomotion mode. This rotates
        // the agent to face the target direction using torque.
        .directional();

    // Our agents
    commands.spawn((
        agent1,
        CurrentWaypoint(0),
        Waypoints(vec![
            Vec3::new(-5.0, 0.0, 5.0),
            Vec3::new(5.0, 0.0, 5.0),
            Vec3::new(5.0, 0.0, -5.0),
            Vec3::new(-5.0, 0.0, -5.0),
        ]),
        Transform::from_translation(Vec3::new(-5.0, 0.0, 5.0)),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            ..default()
        })),
    ));

    let waypoints = vec![
        Vec3::new(-10.0, 0.0, 10.0),
        Vec3::new(10.0, 0.0, 10.0),
        Vec3::new(10.0, 0.0, -10.0),
        Vec3::new(-10.0, 0.0, -10.0),
    ];

    commands.spawn((
        agent2,
        CurrentWaypoint(0),
        Waypoints(waypoints),
        Transform::from_translation(Vec3::new(-10.0, 0.0, 10.0))
            .looking_at(Vec3::new(10.0, 0.0, -10.0), Vec3::Y),
        RigidBody::Dynamic,
        Collider::cuboid(0.5, 1.0, 1.0),
        Mesh3d(meshes.add(Cuboid::new(0.5, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
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

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 30.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

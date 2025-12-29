use avian3d::prelude::*;
use bevy::{color::palettes, prelude::*};
use bevy_steering::prelude::*;
use vleue_navigator::{display_navmesh, prelude::*};

#[derive(Component, Default)]
struct Obstacle;

#[derive(Component)]
struct MovingObstacle {
    start_pos: Vec3,
    end_pos: Vec3,
    speed: f32,
    direction: f32, // 1.0 or -1.0
}

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default,
    Obstacle,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(DebugSteeringPlugin)
        .add_plugins(PhysicsDebugPlugin)
        .add_plugins(VleueNavigatorPlugin)
        .add_plugins(NavmeshUpdaterPlugin::<Collider, Obstacle>::default())
        .add_systems(Startup, (setup, setup_navmesh))
        .add_systems(
            Update,
            (
                setup_path,
                draw_path_gizmos,
                move_obstacles,
                // display_navmesh,
            ),
        )
        .run();
}

fn draw_path_gizmos(mut gizmos: Gizmos, query: Query<&PathFollowing>) {
    for path_following in query.iter() {
        let path = path_following.path();

        // Draw the path spine as connected line segments
        for i in 0..path.len().saturating_sub(1) {
            gizmos.line(path[i], path[i + 1], Color::srgb(1.0, 1.0, 0.0));
        }

        // Draw spheres at each path point
        for point in path {
            gizmos.sphere(*point, 0.2, Color::srgb(1.0, 0.8, 0.0));
        }
    }
}

/// Helper function to spawn a wall
fn spawn_wall(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
    size: Vec3,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.4),
            ..default()
        })),
        Transform::from_translation(position),
        RigidBody::Static,
        Collider::cuboid(size.x, size.y, size.z),
        CollisionLayers::new([GameLayer::Obstacle], [GameLayer::Default]),
        Obstacle,
    ));
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create the maze walls
    let wall_height = 2.0;
    let wall_thickness = 0.5;

    // Outer walls
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, wall_height / 2.0, -14.0),
        Vec3::new(30.0, wall_height, wall_thickness),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(0.0, wall_height / 2.0, 14.0),
        Vec3::new(30.0, wall_height, wall_thickness),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-14.0, wall_height / 2.0, 0.0),
        Vec3::new(wall_thickness, wall_height, 30.0),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(14.0, wall_height / 2.0, 0.0),
        Vec3::new(wall_thickness, wall_height, 30.0),
    );

    // Interior maze walls - creating corridors and obstacles
    // Horizontal walls
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-9.0, wall_height / 2.0, -9.0),
        Vec3::new(6.0, wall_height, wall_thickness),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-3.0, wall_height / 2.0, -3.0),
        Vec3::new(6.0, wall_height, wall_thickness),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(3.0, wall_height / 2.0, 3.0),
        Vec3::new(6.0, wall_height, wall_thickness),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(9.0, wall_height / 2.0, 9.0),
        Vec3::new(6.0, wall_height, wall_thickness),
    );

    // Vertical walls
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-9.0, wall_height / 2.0, -3.0),
        Vec3::new(wall_thickness, wall_height, 6.0),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-3.0, wall_height / 2.0, 3.0),
        Vec3::new(wall_thickness, wall_height, 6.0),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(3.0, wall_height / 2.0, -3.0),
        Vec3::new(wall_thickness, wall_height, 6.0),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(9.0, wall_height / 2.0, 3.0),
        Vec3::new(wall_thickness, wall_height, 6.0),
    );

    // Additional obstacles to make it more challenging
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(-9.0, wall_height / 2.0, -12.0),
        Vec3::new(wall_thickness, wall_height, 4.0),
    );
    spawn_wall(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec3::new(9.0, wall_height / 2.0, 12.0),
        Vec3::new(wall_thickness, wall_height, 4.0),
    );

    // Agent with path following and obstacle avoidance
    let agent1 = SteeringAgent::default()
        .with_max_speed(4.0)
        .with_max_force(Vec3::splat(25.0))
        .with_max_torque(Vec3::splat(20.0))
        .directional();

    commands.spawn((
        agent1,
        PathFollowing::new(vec![], 5.0)
            .with_target_distance(1.0)
            .with_prediction_time(0.1),
        // Try commenting out the Avoid behavior to see what happens!
        Avoid::default().with_distance(2.0),
        TrackNearbyObstacles::default()
            .with_distance(4.0)
            .with_avoid_mask(GameLayer::Obstacle.into()),
        SpeedController::default().with_stopping_distance(1.0),
        Transform::from_translation(Vec3::new(-12.0, 0.4, -14.0))
            .looking_at(Vec3::new(-12.0, 0.0, -6.0), Vec3::Y),
        RigidBody::Dynamic,
        Collider::cuboid(0.4, 0.8, 0.8),
        CollisionLayers::new([GameLayer::Default], [LayerMask::ALL]),
        Mesh3d(meshes.add(Cuboid::new(0.4, 0.8, 0.8))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.8),
            ..default()
        })),
    ));

    // Floor
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(30.0, 30.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.3, 0.2),
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0),
        RigidBody::Static,
        Collider::cuboid(50.0, 0.2, 50.0),
    ));

    // Camera positioned to view the maze from above at an angle
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 35.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Lighting
    commands.spawn((
        DirectionalLight {
            illuminance: 15000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ambient light to see into the maze better
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 200.0,
        ..Default::default()
    });

    // Spawn moving obstacles that cross the path
    let obstacle_size = Vec3::new(1.5, wall_height, 3.5);

    // First moving obstacle
    let start_pos_1 = Vec3::new(-6.0, wall_height / 2.0, 3.0);
    let end_pos_1 = Vec3::new(6.0, wall_height / 2.0, 3.0);

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(obstacle_size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.9, 0.3, 0.1),
            emissive: LinearRgba::rgb(0.9, 0.3, 0.1),
            ..default()
        })),
        Transform::from_translation(start_pos_1),
        RigidBody::Kinematic,
        Collider::cuboid(obstacle_size.x, obstacle_size.y, obstacle_size.z),
        CollisionLayers::new([GameLayer::Obstacle], [GameLayer::Default]),
        MovingObstacle {
            start_pos: start_pos_1,
            end_pos: end_pos_1,
            speed: 1.5,
            direction: 1.0,
        },
    ));

    // Second moving obstacle
    let start_pos_2 = Vec3::new(-3.0, wall_height / 2.0, 2.0);
    let end_pos_2 = Vec3::new(-3.0, wall_height / 2.0, -8.0);
    let obstacle_size = Vec3::new(1.5, wall_height, 3.0);

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::from_size(obstacle_size))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.3, 0.9),
            emissive: LinearRgba::rgb(0.1, 0.3, 0.9),
            ..default()
        })),
        Transform::from_translation(start_pos_2),
        RigidBody::Kinematic,
        Collider::cuboid(obstacle_size.x, obstacle_size.y, obstacle_size.z),
        CollisionLayers::new([GameLayer::Obstacle], [GameLayer::Default]),
        MovingObstacle {
            start_pos: start_pos_2,
            end_pos: end_pos_2,
            speed: 3.0,
            direction: 1.0,
        },
    ));
}

fn setup_navmesh(mut commands: Commands) {
    // Set up NavMesh
    commands.spawn((
        ManagedNavMesh::single(),
        NavMeshSettings {
            fixed: Triangulation::from_outer_edges(&[
                Vec2::new(-25.0, -25.0),
                Vec2::new(25.0, -25.0),
                Vec2::new(25.0, 25.0),
                Vec2::new(-25.0, 25.0),
            ]),
            merge_steps: 3,
            build_timeout: Some(30.0),
            simplify: 0.0,
            agent_radius: 0.7,
            ..default()
        },
        NavMeshUpdateMode::Direct,
        NavMeshDebug(palettes::tailwind::RED_800.into()),
        Transform::from_translation(Vec3::new(0.0, 0.1, 0.0))
            .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
    ));
}

fn setup_path(
    mut query: Query<&mut PathFollowing>,
    navmesh_query: Query<&ManagedNavMesh>,
    navmesh_assets: Res<Assets<NavMesh>>,
) {
    let start = Vec2::new(-12.0, -12.0);
    let end = Vec2::new(12.0, 12.0);

    let Ok(navmesh) = navmesh_query.single() else {
        println!("No navmesh found!");
        return;
    };
    let Some(navmesh) = navmesh_assets.get(navmesh.id()) else {
        println!("Navmesh asset not loaded yet!");
        return;
    };
    let Some(path) = navmesh.path(start, end) else {
        println!("No path found!");
        return;
    };

    let mut agent_path = vec![Vec3::new(start.x, 0.0, start.y)];
    for point in path.path {
        agent_path.push(Vec3::new(point.x, 0.0, point.y));
    }
    for mut agent in query.iter_mut() {
        agent.set_path(agent_path.clone());
    }
}

/// System to move obstacles back and forth
fn move_obstacles(time: Res<Time>, mut query: Query<(&mut Transform, &mut MovingObstacle)>) {
    for (mut transform, mut obstacle) in query.iter_mut() {
        // Calculate movement direction
        let current_pos = transform.translation;
        let target_pos = if obstacle.direction > 0.0 {
            obstacle.end_pos
        } else {
            obstacle.start_pos
        };

        // Move towards target
        let to_target = target_pos - current_pos;
        let distance = to_target.length();

        if distance < 0.1 {
            // Reached target, reverse direction
            obstacle.direction *= -1.0;
        } else {
            // Move towards target
            let movement = to_target.normalize() * obstacle.speed * time.delta_secs();
            transform.translation += movement;
        }
    }
}

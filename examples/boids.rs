use avian3d::prelude::*;
use bevy::{ecs::query::QueryData, prelude::*};
use bevy_steering::prelude::*;

const NUM_BOIDS: usize = 10;
const ZONE_SIZE: f32 = 50.0;
const TARGET_SPEED: f32 = 20.0;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        // .add_plugins(DebugAgentsPlugin)
        .init_resource::<FloatingTarget>()
        .add_systems(Startup, setup)
        .add_systems(Update, (move_boids_system, debug_target_gizmo))
        .insert_resource(TemporalSmoothing::new(0.3))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane (rectangular mesh)
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::new(1000.0, 1000.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.5, 0.3),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.1, 0.0),
        RigidBody::Static,
        Collider::cuboid(1000.0, 0.2, 1000.0),
    ));

    for _ in 0..NUM_BOIDS {
        let rand_x = rand::random_range(-50.0..=50.0);
        let rand_z = rand::random_range(-50.0..=50.0);
        let pos = Vec3::new(rand_x, 0.5, rand_z);
        let shape = Cuboid::new(1.0, 1.0, 2.0);
        let collider = Collider::cuboid(1.0, 1.0, 2.0);
        let agent1 = SteeringAgent::default()
            .with_max_force(Vec3::splat(20.0))
            .with_max_torque(Vec3::splat(100.0))
            .with_neighborhood_radius(20.0)
            .directional();
        commands.spawn((
            Mesh3d(meshes.add(shape)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.8, 0.2, 0.2),
                ..default()
            })),
            Transform::from_translation(pos),
            RigidBody::Dynamic,
            collider,
            Boid,
            Align,
            Cohere::default().with_radius(10.0),
            Separate::default().with_desired_radius(5.0),
            Seek::default(),
            agent1,
        ));
    }

    // Top-down camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 150.0, 0.1).looking_at(Vec3::ZERO, Vec3::Z),
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

#[derive(Component)]
struct Boid;

/// Which edge of the box the target is currently on (clockwise from top-down view)
#[derive(Default, Clone, Copy)]
enum BoxEdge {
    #[default]
    Top, // +Z edge, moving +X
    Right,  // +X edge, moving -Z
    Bottom, // -Z edge, moving -X
    Left,   // -X edge, moving +Z
}

#[derive(Resource)]
struct FloatingTarget {
    position: Vec3,
    edge: BoxEdge,
}

impl Default for FloatingTarget {
    fn default() -> Self {
        Self {
            // Start at top-left corner
            position: Vec3::new(-ZONE_SIZE, 0.0, ZONE_SIZE),
            edge: BoxEdge::Top,
        }
    }
}

#[derive(QueryData)]
#[query_data(mutable)]
struct BoidsQuery {
    boid: &'static Boid,
    seek: &'static mut Seek,
}

fn move_boids_system(
    mut target: ResMut<FloatingTarget>,
    time: Res<Time>,
    mut query: Query<BoidsQuery>,
) {
    let delta = time.delta_secs();
    let move_dist = TARGET_SPEED * delta;

    // Move clockwise along the border based on current edge
    match target.edge {
        BoxEdge::Top => {
            // Moving +X along +Z edge
            target.position.x += move_dist;
            if target.position.x >= ZONE_SIZE {
                target.position.x = ZONE_SIZE;
                target.edge = BoxEdge::Right;
            }
        }
        BoxEdge::Right => {
            // Moving -Z along +X edge
            target.position.z -= move_dist;
            if target.position.z <= -ZONE_SIZE {
                target.position.z = -ZONE_SIZE;
                target.edge = BoxEdge::Bottom;
            }
        }
        BoxEdge::Bottom => {
            // Moving -X along -Z edge
            target.position.x -= move_dist;
            if target.position.x <= -ZONE_SIZE {
                target.position.x = -ZONE_SIZE;
                target.edge = BoxEdge::Left;
            }
        }
        BoxEdge::Left => {
            // Moving +Z along -X edge
            target.position.z += move_dist;
            if target.position.z >= ZONE_SIZE {
                target.position.z = ZONE_SIZE;
                target.edge = BoxEdge::Top;
            }
        }
    }

    // Update all boids to seek the same floating target
    for mut boid in query.iter_mut() {
        boid.seek.set_target(target.position);
    }
}

fn debug_target_gizmo(mut gizmos: Gizmos, target: Res<FloatingTarget>) {
    // Draw a sphere at the target position
    gizmos.sphere(target.position, 2.0, Color::srgb(1.0, 1.0, 0.0));

    // Draw a velocity arrow showing direction based on current edge
    let direction = match target.edge {
        BoxEdge::Top => Vec3::X,
        BoxEdge::Right => Vec3::NEG_Z,
        BoxEdge::Bottom => Vec3::NEG_X,
        BoxEdge::Left => Vec3::Z,
    };
    let arrow_end = target.position + direction * 5.0;
    gizmos.arrow(target.position, arrow_end, Color::srgb(0.0, 1.0, 1.0));

    // Draw the border box
    let corners = [
        Vec3::new(-ZONE_SIZE, 0.0, ZONE_SIZE),
        Vec3::new(ZONE_SIZE, 0.0, ZONE_SIZE),
        Vec3::new(ZONE_SIZE, 0.0, -ZONE_SIZE),
        Vec3::new(-ZONE_SIZE, 0.0, -ZONE_SIZE),
    ];
    for i in 0..4 {
        gizmos.line(corners[i], corners[(i + 1) % 4], Color::srgb(0.5, 0.5, 0.5));
    }
}

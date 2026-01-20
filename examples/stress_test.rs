use avian3d::prelude::*;
use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
use bevy_steering::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(SteeringPlugin)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(FpsHistory {
            values: std::collections::VecDeque::with_capacity(60),
        })
        .add_systems(Startup, (setup, configure_gizmos))
        .add_systems(Update, update_fps_text)
        .run();
}

#[derive(Default, PhysicsLayer)]
enum GameLayers {
    #[default]
    Default,
    Agent,
}

#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct AvgFpsText;

#[derive(Resource)]
struct FpsHistory {
    values: std::collections::VecDeque<f64>,
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

    let agent = SteeringAgent::default()
        .with_max_speed(2.0)
        .with_max_force(Vec3::splat(50.0))
        .omni_directional();

    let agent_mesh = meshes.add(Sphere::new(0.5));

    // Spawn 20 tightly packed agents in a small area
    // Each agent tracks all other agents as obstacles
    let positions = [
        Vec3::new(0.0, 0.5, 0.0),
        Vec3::new(1.2, 0.5, 0.0),
        Vec3::new(-1.2, 0.5, 0.0),
        Vec3::new(0.0, 0.5, 1.2),
        Vec3::new(0.0, 0.5, -1.2),
        Vec3::new(0.85, 0.5, 0.85),
        Vec3::new(-0.85, 0.5, 0.85),
        Vec3::new(0.85, 0.5, -0.85),
        Vec3::new(-0.85, 0.5, -0.85),
        Vec3::new(0.0, 0.5, 0.0),
        // 10 more agents
        Vec3::new(1.8, 0.5, 0.6),
        Vec3::new(-1.8, 0.5, 0.6),
        Vec3::new(1.8, 0.5, -0.6),
        Vec3::new(-1.8, 0.5, -0.6),
        Vec3::new(0.6, 0.5, 1.8),
        Vec3::new(-0.6, 0.5, 1.8),
        Vec3::new(0.6, 0.5, -1.8),
        Vec3::new(-0.6, 0.5, -1.8),
        Vec3::new(1.5, 0.5, 1.5),
        Vec3::new(-1.5, 0.5, -1.5),
    ];

    let targets = [
        Vec3::new(10.0, 0.5, 10.0),
        Vec3::new(-10.0, 0.5, 10.0),
        Vec3::new(10.0, 0.5, -10.0),
        Vec3::new(-10.0, 0.5, -10.0),
        Vec3::new(0.0, 0.5, 12.0),
        Vec3::new(0.0, 0.5, -12.0),
        Vec3::new(12.0, 0.5, 0.0),
        Vec3::new(-12.0, 0.5, 0.0),
        Vec3::new(8.0, 0.5, 8.0),
        Vec3::new(-8.0, 0.5, -8.0),
        // 10 more targets
        Vec3::new(15.0, 0.5, 5.0),
        Vec3::new(-15.0, 0.5, 5.0),
        Vec3::new(15.0, 0.5, -5.0),
        Vec3::new(-15.0, 0.5, -5.0),
        Vec3::new(5.0, 0.5, 15.0),
        Vec3::new(-5.0, 0.5, 15.0),
        Vec3::new(5.0, 0.5, -15.0),
        Vec3::new(-5.0, 0.5, -15.0),
        Vec3::new(12.0, 0.5, 12.0),
        Vec3::new(-12.0, 0.5, -12.0),
    ];

    for (i, (pos, target)) in positions.iter().zip(targets.iter()).enumerate() {
        let hue = i as f32 / 20.0;
        let color = Color::hsl(hue * 360.0, 0.8, 0.5);

        commands.spawn((
            Mesh3d(agent_mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(*pos),
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            // Agents detect other agents as obstacles
            // Avoid::default().with_distance(3.0),
            TrackNearbyObstacles::default()
                .with_distance(10.0)
                .with_avoid_mask(GameLayers::Agent.into()),
            // Approach::new(*target, 1.0),
            CollisionLayers::new(
                [GameLayers::Agent],
                [GameLayers::Default, GameLayers::Agent],
            ),
            agent.clone(),
        ));
    }

    // Top-down camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 25.0, 0.1).looking_at(Vec3::ZERO, Vec3::Z),
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

    // FPS counter UI
    commands.spawn((
        Text::new("FPS: --"),
        TextFont {
            font_size: 30.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        FpsText,
    ));

    // Average FPS counter UI
    commands.spawn((
        Text::new("Avg (60): --"),
        TextFont {
            font_size: 30.0,
            ..default()
        },
        TextColor(Color::srgb(0.8, 0.8, 0.2)),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(45.0),
            left: Val::Px(10.0),
            ..default()
        },
        AvgFpsText,
    ));
}

fn update_fps_text(
    diagnostics: Res<DiagnosticsStore>,
    mut history: ResMut<FpsHistory>,
    mut fps_query: Query<&mut Text, (With<FpsText>, Without<AvgFpsText>)>,
    mut avg_query: Query<&mut Text, (With<AvgFpsText>, Without<FpsText>)>,
) {
    if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
        if let Some(value) = fps.smoothed() {
            // Update current FPS
            for mut text in &mut fps_query {
                **text = format!("FPS: {value:.1}");
            }

            // Track history
            if history.values.len() >= 60 {
                history.values.pop_front();
            }
            history.values.push_back(value);

            // Update average FPS
            let avg = history.values.iter().sum::<f64>() / history.values.len() as f64;
            for mut text in &mut avg_query {
                **text = format!("Avg (60): {avg:.1}");
            }
        }
    }
}

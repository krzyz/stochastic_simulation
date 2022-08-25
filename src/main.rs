use std::{f32::consts::PI, time::Duration};

use bevy::prelude::*;
use bevy_egui::{
    egui::{self, Rgba},
    EguiContext, EguiPlugin,
};
use bevy_prototype_lyon::prelude::{FillMode, *};
use bevy_rapier2d::prelude::*;
use egui::plot::{Line, Plot, Value, Values};
use iyes_loopless::{
    prelude::{AppLooplessStateExt, ConditionSet},
    state::NextState,
};
use rand::{prelude::Distribution, Rng};
use rand_distr::Normal;
use rmpfit::{MPFitter, MPResult};

const TIMER_STEP: u64 = 20;
const FIXED_HEIGHT: f32 = 1500.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum SimulationState {
    Stopped,
    Init,
    Running,
}

pub struct Settings {
    pub small_radius: f32,
    pub big_radius: f32,
    pub temp: f32,
    pub dens: f32,
    pub big_part_no: usize,
    pub fit_sqrt: bool,
}

impl Default for Settings {
    fn default() -> Self {
        let small_radius = 5.0;
        let big_radius = 25.0;
        let temp = 100.0;
        let dens = 2.0;
        let big_part_no = 1;
        let fit_sqrt = false;

        Settings {
            small_radius,
            big_radius,
            temp,
            dens,
            big_part_no,
            fit_sqrt,
        }
    }
}

struct Measures {
    distances: Vec<Vec<f32>>,
    avg_distance: Vec<f32>,
}

impl Measures {
    fn new(big_particle_no: usize) -> Self {
        Measures {
            distances: vec![vec![]; big_particle_no],
            avg_distance: vec![],
        }
    }
}

#[derive(Component)]
struct AirParticle;

#[derive(Component)]
struct BigParticle {
    start_pos: Vec2,
    id: usize,
}

#[derive(Component)]
struct MeasureTime {
    timer: Timer,
}

fn main() {
    App::new()
        .insert_resource(Msaa { samples: 4 })
        .add_plugins(DefaultPlugins)
        .add_plugin(EguiPlugin)
        .add_plugin(ShapePlugin)
        .add_plugin(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(1.0))
        .add_startup_system(setup_system)
        .add_system(controls_ui)
        .add_loopless_state(SimulationState::Stopped)
        .add_enter_system(SimulationState::Init, init_running)
        .add_enter_system(SimulationState::Running, start_simulation)
        .add_exit_system(SimulationState::Running, cleanup_simulation)
        .add_system_set(
            ConditionSet::new()
                .run_in_state(SimulationState::Running)
                .with_system(measure)
                .with_system(distance_plot_ui)
                .into(),
        )
        .run();
}

fn setup_system(mut commands: Commands) {
    commands.insert_resource(Settings::default());

    commands.insert_resource(Measures::new(Settings::default().big_part_no));

    commands.spawn_bundle(Camera2dBundle {
        projection: OrthographicProjection {
            scaling_mode: bevy::render::camera::ScalingMode::FixedVertical(FIXED_HEIGHT),
            ..default()
        },
        ..default()
    });
}

fn start_simulation(
    mut commands: Commands,
    mut measures: ResMut<Measures>,
    settings: Res<Settings>,
) {
    *measures = Measures::new(settings.big_part_no);

    let mut rng = rand::thread_rng();

    let shape_small = shapes::Circle {
        radius: settings.small_radius,
        ..shapes::Circle::default()
    };

    let shape_big = shapes::Circle {
        radius: settings.big_radius,
        ..shapes::Circle::default()
    };

    let air_part_no = (FIXED_HEIGHT * FIXED_HEIGHT * settings.dens / 1000.0) as usize;


    let mut big_part_positions: Vec<Vec2> = Vec::new();
    'outer: for _ in 0..settings.big_part_no {
        let max_retries = 20;
        for i in 0..=max_retries {
            let margin = 0.173 * settings.big_part_no.pow(2) as f32 - 24.33 * settings.big_part_no as f32 + 700.;
            let margin = margin.clamp(100., 700.);

            let pos = Vec2::new(
                get_val_in_range(FIXED_HEIGHT, settings.big_radius + margin, &mut rng),
                get_val_in_range(FIXED_HEIGHT, settings.big_radius + margin, &mut rng),
            );

            let valid_placement = big_part_positions 
                .iter()
                .all(|&other_pos| other_pos.distance(pos) >= 4.0 * settings.big_radius);

            if valid_placement {
                big_part_positions.push(pos);
                break;
            }

            if i == max_retries {
                break 'outer;
            }
        }
    }

    let mut small_part_positions: Vec<Vec2> = Vec::new();
    'outer: for _ in 0..air_part_no {
        let max_retries = 20;
        for i in 0..=max_retries {
            let pos = Vec2::new(
                get_val_in_range(FIXED_HEIGHT, settings.small_radius, &mut rng),
                get_val_in_range(FIXED_HEIGHT, settings.small_radius, &mut rng),
            );

            let valid_placement = small_part_positions
                .iter()
                .all(|&other_pos| other_pos.distance(pos) >= settings.small_radius);

            let valid_placement = valid_placement && big_part_positions 
                .iter()
                .all(|&other_pos| other_pos.distance(pos) >= settings.big_radius);

            if valid_placement {
                small_part_positions.push(pos);
                break;
            }

            if i == max_retries {
                break 'outer;
            }
        }
    }

    let normal = Normal::new(0.0, settings.temp).unwrap();

    let initial_velocities = (0..small_part_positions.len())
        .map(|_| Vec2::new(normal.sample(&mut rng), normal.sample(&mut rng)))
        .collect::<Vec<_>>();

    for (i, pos) in big_part_positions.iter().enumerate() {
        commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(LockedAxes::ROTATION_LOCKED)
            .insert(Collider::ball(settings.big_radius))
            .insert(CollisionGroups::new(0b100, 0b111))
            .insert(Friction::coefficient(0.0))
            .insert(Restitution::coefficient(1.0))
            .insert(GravityScale(0.0))
            .insert(Velocity::linear(Vec2::new(
                normal.sample(&mut rng) * settings.small_radius.powi(2)
                    / settings.big_radius.powi(2),
                normal.sample(&mut rng) * settings.small_radius.powi(2)
                    / settings.big_radius.powi(2),
            )))
            .insert_bundle(GeometryBuilder::build_as(
                &shape_big,
                DrawMode::Outlined {
                    fill_mode: FillMode::color(Color::RED),
                    outline_mode: StrokeMode::new(Color::BLACK, 2.0),
                },
                Transform::from_xyz(pos.x, pos.y, 0.0),
            ))
            .insert(BigParticle {
                start_pos: *pos,
                id: i,
            });
    }

    for (pos, &vel) in small_part_positions.iter().zip(initial_velocities.iter()) {
        commands
            .spawn()
            .insert(RigidBody::Dynamic)
            .insert(LockedAxes::ROTATION_LOCKED)
            .insert(Collider::ball(settings.small_radius))
            .insert(CollisionGroups::new(0b010, 0b111))
            .insert(Friction::coefficient(0.0))
            .insert(Restitution::coefficient(0.998))
            .insert(GravityScale(0.0))
            .insert(Velocity::linear(vel))
            .insert(AirParticle)
            .insert_bundle(GeometryBuilder::build_as(
                &shape_small,
                DrawMode::Outlined {
                    fill_mode: FillMode::color(Color::CYAN),
                    outline_mode: StrokeMode::new(Color::BLACK, 2.0),
                },
                Transform::from_xyz(pos.x, pos.y, 0.0),
            ));
    }

    let collider_width = 100.0;
    let wall_collider = Collider::cuboid(FIXED_HEIGHT + 100., 100.);

    commands
        .spawn()
        .insert(RigidBody::Fixed)
        .insert(Friction::coefficient(0.0))
        .insert(Restitution::coefficient(1.0))
        .insert(Collider::compound(vec![
            (
                Vec2::new(0.0, -0.5 * FIXED_HEIGHT - collider_width),
                0.0,
                wall_collider.clone(),
            ),
            (
                Vec2::new(0.0, 0.5 * FIXED_HEIGHT + collider_width),
                0.0,
                wall_collider.clone(),
            ),
            (
                Vec2::new(-0.5 * FIXED_HEIGHT - collider_width, 0.0),
                0.5 * PI,
                wall_collider.clone(),
            ),
            (
                Vec2::new(0.5 * FIXED_HEIGHT + collider_width, 0.0),
                0.5 * PI,
                wall_collider.clone(),
            ),
        ]))
        .insert(CollisionGroups::new(0b001, 0b111));

    commands.insert_resource(MeasureTime {
        timer: Timer::new(Duration::from_millis(TIMER_STEP), true),
    })
}

fn cleanup_simulation(mut commands: Commands, query: Query<Entity, With<RigidBody>>) {
    for e in query.iter() {
        commands.entity(e).despawn_recursive();
    }
}

fn get_val_in_range(range_width: f32, radius: f32, rng: &mut impl Rng) -> f32 {
    rng.gen_range(-0.5 * range_width + radius..0.5 * range_width - radius)
}

fn measure(
    mut measures: ResMut<Measures>,
    query: Query<(&Transform, &BigParticle)>,
    time: Res<Time>,
    mut measure_time: ResMut<MeasureTime>,
) {
    measure_time.timer.tick(time.delta());
    if measure_time.timer.finished() {
        let mut sum_distances = 0.0;
        let mut part_no = 0;
        for (transform, particle) in query.iter() {
            let distance = Vec2::new(transform.translation.x, transform.translation.y)
                .distance(particle.start_pos);
            measures.distances[particle.id].push(distance);
            sum_distances += distance;
            part_no += 1;
        }
        measures.avg_distance.push(sum_distances / (part_no as f32));
    }
}

fn distance_plot_ui(mut egui_context: ResMut<EguiContext>, settings: Res<Settings>, measures: Res<Measures>) {
    let avg_line = make_line(&measures.avg_distance[..]).color(Rgba::from_rgb(1.0, 1.0, 1.0));
    let other_lines = measures.distances.iter().map(|distance| {
        make_line(&distance[..]).color(Rgba::from_rgba_unmultiplied(1.0, 0.0, 0.0, 0.4))
    });

    egui::Window::new("Distance").show(egui_context.ctx_mut(), |ui| {
        Plot::new("distances").view_aspect(2.0).show(ui, |plot_ui| {
            for line in other_lines {
                plot_ui.line(line);
            }

            if settings.fit_sqrt {
                let fitted_line = fit_sqrt_and_make_line(
                    &measures
                        .avg_distance
                        .iter()
                        .map(|&v| v as f64)
                        .collect::<Vec<_>>()[..],
                )
                .color(Rgba::from_rgba_unmultiplied(0.0, 1.0, 0.0, 1.0));


                plot_ui.line(fitted_line);
            }
            plot_ui.line(avg_line);
        });
    });
}

fn init_running(mut commands: Commands) {
    commands.insert_resource(NextState(SimulationState::Running));
}

fn controls_ui(
    mut commands: Commands,
    mut egui_context: ResMut<EguiContext>,
    mut settings: ResMut<Settings>,
) {
    egui::Window::new("Controls").show(egui_context.ctx_mut(), |ui| {
        ui.add(egui::Slider::new(&mut settings.small_radius, 1.0..=50.0).text("R Small"));
        ui.add(egui::Slider::new(&mut settings.big_radius, 1.0..=120.0).text("R Big"));
        ui.add(egui::Slider::new(&mut settings.temp, 10.0..=200.0).text("T Small"));
        ui.add(egui::Slider::new(&mut settings.dens, 0.1..=4.0).text("ρ Small"));
        ui.add(egui::Slider::new(&mut settings.big_part_no, 0..=120).text("# Big"));
        ui.add(egui::Checkbox::new(&mut settings.fit_sqrt, "Fit square root"));
        if ui.button("(Re)start").clicked() {
            commands.insert_resource(NextState(SimulationState::Init));
        }
    });
}

fn make_line(distances: &[f32]) -> Line {
    let distance = distances.iter().enumerate().map(|(i, distance)| {
        let x = i as f64 * 0.001 * (TIMER_STEP as f64);
        Value::new(x, *distance)
    });
    Line::new(Values::from_values_iter(distance))
}

fn square_root(params: &[f64], x: f64) -> f64 {
    params[0] + params[1] * (x + params[2]).sqrt()
}

struct SquareRoot {
    x: Vec<f64>,
    y: Vec<f64>,
}

impl MPFitter for SquareRoot {
    fn eval(&self, params: &[f64], deviates: &mut [f64]) -> MPResult<()> {
        for ((d, x), y) in deviates.iter_mut().zip(self.x.iter()).zip(self.y.iter()) {
            *d = *y - square_root(params, *x);
        }
        Ok(())
    }

    fn number_of_points(&self) -> usize {
        self.x.len()
    }
}

fn fit_sqrt_and_make_line(distances: &[f64]) -> Line {
    let x = (0..distances.len())
        .map(|i| i as f64 * 0.001 * (TIMER_STEP as f64))
        .collect::<Vec<_>>();
    let y = distances.iter().copied().collect::<Vec<_>>();

    let sr = SquareRoot { x: x.clone(), y };

    let mut params = [0., 1., 0.];
    let _res = sr.mpfit(&mut params, None, &Default::default()).ok();

    Line::new(Values::from_values_iter(
        x.iter()
            .map(|&x_val| Value::new(x_val, square_root(&params, x_val))),
    ))
}

use macroquad::prelude::*;

// Arena & Entity Constants
const WIN_WIDTH: f32 = 540.0;
const WIN_HEIGHT: f32 = 960.0;

const ARENA_X: f32 = 40.0;
const ARENA_Y: f32 = 140.0;
const ARENA_W: f32 = 460.0;
const ARENA_H: f32 = 640.0;

const PADDLE_MARGIN: f32 = 24.0;
const PADDLE_W: f32 = 12.0;
const PADDLE_H: f32 = 95.0;
const BALL_RADIUS: f32 = 6.5;

// Harmonic Period of Paddles (in seconds)
const PADDLE_PERIOD: f32 = 1.60;

// Maximum balls to prevent GPU vertex buffer saturation
const MAX_BALLS: usize = 4096;

/// Cache-friendly 32-byte aligned Ball representation.
/// Instead of incrementally integrating `y += vy * dt` (which accumulates float drift),
/// we store the exact departure state and evaluate the unfolded triangle-wave reflection analytically,
/// while also maintaining `x, y, vx, vy` for crisp visual rendering.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct Ball {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    t_start: f64,
    y_start: f32,
    color_idx: u8,
    heading_right: bool,
}

const PALETTE: &[Color] = &[
    Color::new(0.95, 0.26, 0.21, 1.0), // Red
    Color::new(1.00, 0.60, 0.00, 1.0), // Orange
    Color::new(1.00, 0.92, 0.23, 1.0), // Yellow
    Color::new(0.30, 0.69, 0.31, 1.0), // Green
    Color::new(0.00, 0.74, 0.83, 1.0), // Cyan
    Color::new(0.13, 0.59, 0.95, 1.0), // Blue
    Color::new(0.61, 0.15, 0.69, 1.0), // Purple
    Color::new(0.91, 0.12, 0.39, 1.0), // Pink
];

/// Triangle wave oscillating in [-1.0, 1.0] with period 1.0
#[inline(always)]
fn triangle_wave(phase: f64) -> f32 {
    let p = phase.rem_euclid(1.0) as f32;
    if p < 0.25 {
        p * 4.0
    } else if p < 0.75 {
        2.0 - p * 4.0
    } else {
        p * 4.0 - 4.0
    }
}

/// Folds an unbounded vertical coordinate `u` into the valid bounce range `[0, h_eff]`
#[inline(always)]
fn fold_vertical(u: f32, h_eff: f32) -> (f32, f32) {
    let period = 2.0 * h_eff;
    let m = u.rem_euclid(period);
    if m <= h_eff {
        (m, 1.0)
    } else {
        (period - m, -1.0)
    }
}

/// Computes the vertical center of the left and right paddles at global time `t` (in local `[0, h_eff]` space).
/// Left and right paddles run 180 degrees (0.5 period) out of phase for visual symmetry.
#[inline(always)]
fn paddle_y_local(t: f64, is_right: bool, h_eff: f32) -> f32 {
    let phase_offset = if is_right { 0.5 } else { 0.0 };
    let phase = (t / PADDLE_PERIOD as f64) + phase_offset;
    let center = h_eff * 0.5;
    // Keep paddle within arena bounds
    let max_amp = (h_eff - PADDLE_H) * 0.5;
    center + max_amp * triangle_wave(phase)
}

/// Solves for the exact `(vx, vy)` vector so a ball leaving `(x_start, y_local)` at `t_now`
/// lands squarely on the opposite paddle after `k` top/bottom wall bounces.
#[inline]
fn compute_synchronized_velocity(
    t_now: f64,
    y_local: f32,
    heading_right: bool,
    w_eff: f32,
    h_eff: f32,
    half_periods: u32, // Transit time in multiples of (PADDLE_PERIOD / 2)
    k_bounces: i32,    // Number of vertical wall bounces (-3..=3)
    paddle_offset: f32,
) -> (f32, f32) {
    let transit_time = (half_periods as f32) * (PADDLE_PERIOD * 0.5);
    let t_arrival = t_now + transit_time as f64;

    let target_y = (paddle_y_local(t_arrival, heading_right, h_eff) + paddle_offset)
        .clamp(0.0, h_eff);

    // Unfold the target Y coordinate across `k_bounces` reflections
    let unfolded_target = if k_bounces.rem_euclid(2) == 0 {
        (k_bounces as f32) * h_eff + target_y
    } else {
        ((k_bounces + 1) as f32) * h_eff - target_y
    };

    let vx = if heading_right {
        w_eff / transit_time
    } else {
        -w_eff / transit_time
    };
    let vy = (unfolded_target - y_local) / transit_time;

    (vx, vy)
}

fn window_conf() -> Conf {
    Conf {
        window_title: "The Math is Mathing - Rust Simulation".to_owned(),
        window_width: WIN_WIDTH as i32,
        window_height: WIN_HEIGHT as i32,
        window_resizable: false,
        sample_count: 4, // 4x MSAA for crisp geometry
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    // Effective inner bounds for the ball's center coordinate
    let left_x = ARENA_X + PADDLE_MARGIN + PADDLE_W + BALL_RADIUS;
    let right_x = ARENA_X + ARENA_W - PADDLE_MARGIN - PADDLE_W - BALL_RADIUS;
    let w_eff = right_x - left_x;

    let top_y = ARENA_Y + BALL_RADIUS;
    let bottom_y = ARENA_Y + ARENA_H - BALL_RADIUS;
    let h_eff = bottom_y - top_y;

    // Pre-allocate buffers to avoid heap allocations during the hot simulation loop
    let mut balls: Vec<Ball> = Vec::with_capacity(MAX_BALLS);
    let mut spawned_balls: Vec<Ball> = Vec::with_capacity(512);

    let mut sim_time: f64 = 0.0;
    let fixed_dt: f64 = 1.0 / 120.0; // Fixed 120Hz physics clock for deterministic sync

    // Spawn the initial seed ball on the left paddle face
    let init_y_local = paddle_y_local(sim_time, false, h_eff);
    let (init_vx, init_vy) = compute_synchronized_velocity(
        sim_time,
        init_y_local,
        true,
        w_eff,
        h_eff,
        2, // 1 full period transit
        1, // 1 wall bounce
        0.0,
    );

    balls.push(Ball {
        x: left_x,
        y: top_y + init_y_local,
        vx: init_vx,
        vy: init_vy,
        t_start: sim_time,
        y_start: init_y_local,
        color_idx: 0,
        heading_right: true,
    });

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        // Press R to reset the simulation back to 1 ball
        if is_key_pressed(KeyCode::R) {
            balls.clear();
            sim_time = 0.0;
            let y0 = paddle_y_local(0.0, false, h_eff);
            let (vx, vy) = compute_synchronized_velocity(0.0, y0, true, w_eff, h_eff, 2, 1, 0.0);
            balls.push(Ball {
                x: left_x,
                y: top_y + y0,
                vx,
                vy,
                t_start: 0.0,
                y_start: y0,
                color_idx: 0,
                heading_right: true,
            });
        }

        // --- 1. UPDATE STEP (Deterministic Analytic Evaluation) ---
        sim_time += fixed_dt;
        spawned_balls.clear();
        let current_count = balls.len();

        for ball in balls.iter_mut() {
            let elapsed = (sim_time - ball.t_start) as f32;
            let dx = ball.vx * elapsed;

            ball.x = if ball.heading_right {
                left_x + dx
            } else {
                right_x + dx
            };

            let unfolded_y = ball.y_start + ball.vy * elapsed;
            let (folded_y, dir_sign) = fold_vertical(unfolded_y, h_eff);
            ball.y = top_y + folded_y;

            // Check if the ball has reached the opposing paddle plane
            let hit_right = ball.heading_right && ball.x >= right_x;
            let hit_left = !ball.heading_right && ball.x <= left_x;

            if hit_right || hit_left {
                let new_heading_right = !ball.heading_right;
                let impact_x = if hit_right { right_x } else { left_x };

                // Pick a synchronized trajectory for the primary rebounding ball
                let half_periods = fastrand::u32(1..=2); // Fast or medium crossing speed
                let k_bounces = if fastrand::bool() { 1 } else { -1 } * fastrand::i32(1..=2);
                let offset = (fastrand::f32() - 0.5) * (PADDLE_H * 0.45);

                let (nvx, nvy) = compute_synchronized_velocity(
                    sim_time,
                    folded_y,
                    new_heading_right,
                    w_eff,
                    h_eff,
                    half_periods,
                    k_bounces,
                    offset,
                );

                ball.x = impact_x;
                ball.vx = nvx;
                ball.vy = nvy * dir_sign.abs();
                ball.t_start = sim_time;
                ball.y_start = folded_y;
                ball.heading_right = new_heading_right;
                ball.color_idx = (ball.color_idx + 1) % (PALETTE.len() as u8);

                // Spawn a split ball with a complementary bounce harmonic if under capacity
                if current_count + spawned_balls.len() < MAX_BALLS {
                    let alt_k = -k_bounces;
                    let alt_periods = if half_periods == 1 { 2 } else { 1 };
                    let alt_offset = -offset;

                    let (svx, svy) = compute_synchronized_velocity(
                        sim_time,
                        folded_y,
                        new_heading_right,
                        w_eff,
                        h_eff,
                        alt_periods,
                        alt_k,
                        alt_offset,
                    );

                    spawned_balls.push(Ball {
                        x: impact_x,
                        y: top_y + folded_y,
                        vx: svx,
                        vy: svy,
                        t_start: sim_time,
                        y_start: folded_y,
                        color_idx: fastrand::u8(0..PALETTE.len() as u8),
                        heading_right: new_heading_right,
                    });
                }
            }
        }

        balls.extend_from_slice(&spawned_balls);

        // --- 2. RENDER STEP ---
        clear_background(Color::from_rgba(242, 242, 244, 255));

        // Top Header Caption
        let title = "When a game dev decides to";
        let subtitle = "flex his math skills";
        draw_text(title, 78.0, 68.0, 32.0, BLACK);
        draw_text(subtitle, 130.0, 102.0, 32.0, BLACK);

        // Arena Box Background & Border
        draw_rectangle(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, WHITE);
        draw_rectangle_lines(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, 5.0, BLACK);

        // Render Balls
        for ball in &balls {
            let fill = PALETTE[ball.color_idx as usize];
            draw_circle(ball.x, ball.y, BALL_RADIUS, BLACK);
            draw_circle(ball.x, ball.y, BALL_RADIUS - 1.5, fill);
        }

        // Render Synchronized Paddles
        let left_paddle_center = top_y + paddle_y_local(sim_time, false, h_eff);
        let right_paddle_center = top_y + paddle_y_local(sim_time, true, h_eff);

        draw_rectangle(
            ARENA_X + PADDLE_MARGIN,
            left_paddle_center - PADDLE_H * 0.5,
            PADDLE_W,
            PADDLE_H,
            BLACK,
        );
        draw_rectangle(
            ARENA_X + ARENA_W - PADDLE_MARGIN - PADDLE_W,
            right_paddle_center - PADDLE_H * 0.5,
            PADDLE_W,
            PADDLE_H,
            BLACK,
        );

        // Bottom Live Counter
        let count_text = format!("Balls: {}", balls.len());
        let text_dims = measure_text(&count_text, None, 38, 1.0);
        draw_text(
            &count_text,
            (WIN_WIDTH - text_dims.width) * 0.5,
            ARENA_Y + ARENA_H + 65.0,
            38.0,
            BLACK,
        );

        next_frame().await;
    }
}
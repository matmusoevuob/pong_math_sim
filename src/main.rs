use macroquad::prelude::*;

// Window & Arena Constants
const WIN_WIDTH: f32 = 540.0;
const WIN_HEIGHT: f32 = 960.0;

const ARENA_X: f32 = 40.0;
const ARENA_Y: f32 = 140.0;
const ARENA_W: f32 = 460.0;
const ARENA_H: f32 = 640.0;

const PADDLE_MARGIN: f32 = 22.0;
const PADDLE_W: f32 = 12.0;
const PADDLE_H: f32 = 100.0;
const BALL_RADIUS: f32 = 6.5;

// Single unified period for BOTH paddles so they remain locked in opposite directions
const PADDLE_PERIOD: f64 = 2.60;

// Slow, calm transit time range (in seconds) that spans a full paddle cycle
// so arriving balls can intercept the opposite paddle at ANY vertical height
const MIN_TRANSIT_TIME: f32 = 1.65;
const MAX_TRANSIT_TIME: f32 = 3.45;

const MAX_BALLS: usize = 3000;

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

/// Linear triangle wave oscillating in [-1.0, 1.0] with period 1.0
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

/// Folds an unbounded vertical coordinate `u` into `[0, h_eff]`
#[inline(always)]
fn fold_vertical(u: f32, h_eff: f32) -> f32 {
    let period = 2.0 * h_eff;
    let m = u.rem_euclid(period);
    if m <= h_eff {
        m
    } else {
        period - m
    }
}

/// Computes the vertical center of the left or right paddle at global time `t`.
/// Left and right paddles share `PADDLE_PERIOD` and are locked 0.5 (180 degrees) out of phase:
/// when the right paddle hits the top corner, the left paddle hits the bottom corner.
#[inline(always)]
fn paddle_y_local(t: f64, is_right: bool, h_eff: f32) -> f32 {
    let phase_offset = if is_right { 0.5 } else { 0.0 };
    let phase = (t / PADDLE_PERIOD) + phase_offset;

    let center = h_eff * 0.5;
    // Sweep all the way from the top wall to the bottom wall
    let max_amp = (h_eff - PADDLE_H) * 0.5 + BALL_RADIUS;
    center + max_amp * triangle_wave(phase)
}

/// Solves for `(vx, vy)` by sweeping arrival times across a full paddle oscillation cycle.
/// Because `dt` spans a full period `[MIN_TRANSIT_TIME, MAX_TRANSIT_TIME]`, the target paddle
/// sweeps from top to bottom during that window, allowing us to pick the exact arrival time
/// where the wall bounce lands at `target_wall_alpha` in `[0.06, 0.94]`.
fn solve_wide_wall_trajectory(
    t_now: f64,
    y_start: f32,
    heading_right: bool,
    w_eff: f32,
    h_eff: f32,
    target_wall_alpha: f32, // Desired wall hit fraction across the width [0.06, 0.94]
    target_top_wall: bool,
) -> (f32, f32) {
    let mut best_vx = 0.0;
    let mut best_vy = 0.0;
    let mut best_score = f32::MAX;

    // Sample 48 candidate arrival times across our slow transit window
    let steps = 48;
    for i in 0..steps {
        let frac = (i as f32) / ((steps - 1) as f32);
        let dt = MIN_TRANSIT_TIME + frac * (MAX_TRANSIT_TIME - MIN_TRANSIT_TIME);
        let t_arrival = t_now + dt as f64;

        let paddle_center = paddle_y_local(t_arrival, heading_right, h_eff);

        // Allow landing anywhere along the paddle face to widen corner reach
        for &offset_ratio in &[-0.42_f32, -0.20, 0.0, 0.20, 0.42] {
            let y_target = (paddle_center + offset_ratio * PADDLE_H)
                .clamp(2.0, h_eff - 2.0);

            let (unfolded_y, wall_alpha) = if target_top_wall {
                // Single bounce off TOP wall (y = 0)
                let unfolded = -y_target;
                let alpha = y_start / (y_start + y_target).max(1.0);
                (unfolded, alpha)
            } else {
                // Single bounce off BOTTOM wall (y = h_eff)
                let unfolded = 2.0 * h_eff - y_target;
                let d0 = h_eff - y_start;
                let d1 = h_eff - y_target;
                let alpha = d0 / (d0 + d1).max(1.0);
                (unfolded, alpha)
            };

            let vy = (unfolded_y - y_start) / dt;
            let vx = if heading_right { w_eff / dt } else { -w_eff / dt };

            // Primary objective: hit the wall at `target_wall_alpha`
            let mut score = (wall_alpha - target_wall_alpha).abs();

            // Gentle penalty against extreme vertical speeds so motion stays uniform and calm
            if vy.abs() > 380.0 {
                score += (vy.abs() - 380.0) * 0.004;
            }

            if score < best_score {
                best_score = score;
                best_vx = vx;
                best_vy = vy;
            }
        }
    }

    (best_vx, best_vy)
}

fn window_conf() -> Conf {
    Conf {
        window_title: "The Math is Mathing - Anti-Phase Paddles".to_owned(),
        window_width: WIN_WIDTH as i32,
        window_height: WIN_HEIGHT as i32,
        window_resizable: false,
        sample_count: 4,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let left_x = ARENA_X + PADDLE_MARGIN + PADDLE_W + BALL_RADIUS;
    let right_x = ARENA_X + ARENA_W - PADDLE_MARGIN - PADDLE_W - BALL_RADIUS;
    let w_eff = right_x - left_x;

    let top_y = ARENA_Y + BALL_RADIUS;
    let bottom_y = ARENA_Y + ARENA_H - BALL_RADIUS;
    let h_eff = bottom_y - top_y;

    let mut balls: Vec<Ball> = Vec::with_capacity(MAX_BALLS);
    let mut spawned_balls: Vec<Ball> = Vec::with_capacity(512);

    let mut sim_time: f64 = 0.0;
    let fixed_dt: f64 = 1.0 / 120.0;

    // Start at t = 0.25 * PADDLE_PERIOD so the left paddle begins near the bottom corner
    // and the right paddle begins near the top corner, immediately enabling wide wall bounces.
    let start_t = 0.22 * PADDLE_PERIOD;
    sim_time = start_t;

    let spawn_initial_ball = |balls: &mut Vec<Ball>, t: f64| {
        balls.clear();
        let y0 = paddle_y_local(t, false, h_eff);
        let (vx, vy) = solve_wide_wall_trajectory(
            t,
            y0,
            true,
            w_eff,
            h_eff,
            0.22, // Initial ball hits top wall 22% from the left
            true,
        );
        balls.push(Ball {
            x: left_x,
            y: top_y + y0,
            vx,
            vy,
            t_start: t,
            y_start: y0,
            color_idx: 0,
            heading_right: true,
        });
    };

    spawn_initial_ball(&mut balls, sim_time);

    // Golden-ratio quasirandom sequence state so wall hit targets evenly blanket [0.06, 0.94]
    // without clumping in the middle
    let mut quasi_seq: f32 = 0.15;

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        if is_key_pressed(KeyCode::R) {
            sim_time = start_t;
            spawn_initial_ball(&mut balls, sim_time);
        }

        // --- 1. UPDATE STEP ---
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
            let folded_y = fold_vertical(unfolded_y, h_eff);
            ball.y = top_y + folded_y;

            let hit_right = ball.heading_right && ball.x >= right_x;
            let hit_left = !ball.heading_right && ball.x <= left_x;

            if hit_right || hit_left {
                let new_heading_right = !ball.heading_right;
                let impact_x = if hit_right { right_x } else { left_x };

                // Advance golden-ratio low-discrepancy sequence to guarantee uniform coverage
                // across the entire wall from left corner (0.06) to right corner (0.94)
                quasi_seq = (quasi_seq + 0.61803398875).fract();
                let alpha_primary = 0.06 + quasi_seq * 0.88;
                let bounce_top = fastrand::bool();

                let (nvx, nvy) = solve_wide_wall_trajectory(
                    sim_time,
                    folded_y,
                    new_heading_right,
                    w_eff,
                    h_eff,
                    alpha_primary,
                    bounce_top,
                );

                ball.x = impact_x;
                ball.vx = nvx;
                ball.vy = nvy;
                ball.t_start = sim_time;
                ball.y_start = folded_y;
                ball.heading_right = new_heading_right;
                ball.color_idx = (ball.color_idx + 1) % (PALETTE.len() as u8);

                // Spawn split ball targeting the opposite wall and complementary horizontal zone
                if current_count + spawned_balls.len() < MAX_BALLS {
                    let alpha_split = (1.0 - alpha_primary).clamp(0.06, 0.94);
                    let (svx, svy) = solve_wide_wall_trajectory(
                        sim_time,
                        folded_y,
                        new_heading_right,
                        w_eff,
                        h_eff,
                        alpha_split,
                        !bounce_top,
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

        let title = "When a game dev decides to";
        let subtitle = "flex his math skills";
        draw_text(title, 78.0, 68.0, 32.0, BLACK);
        draw_text(subtitle, 130.0, 102.0, 32.0, BLACK);

        draw_rectangle(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, WHITE);
        draw_rectangle_lines(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, 5.0, BLACK);

        for ball in &balls {
            let fill = PALETTE[ball.color_idx as usize];
            draw_circle(ball.x, ball.y, BALL_RADIUS, BLACK);
            draw_circle(ball.x, ball.y, BALL_RADIUS - 1.5, fill);
        }

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
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

// Slower, smooth paddle periods with an irrational/coprime ratio (Phi-like)
// so left and right paddles constantly explore new (y_0, y_1) height combinations!
const LEFT_PADDLE_PERIOD: f64 = 2.80;
const RIGHT_PADDLE_PERIOD: f64 = 2.15;

// Slow, satisfying transit time range (in seconds) to cross the screen
const MIN_TRANSIT_TIME: f32 = 1.85;
const MAX_TRANSIT_TIME: f32 = 3.20;

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

/// Smoothly rounded triangle wave in [-1.0, 1.0] with period 1.0
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
/// Using two distinct periods ensures `(y_depart, y_arrive)` pairs cover all ratios,
/// which is the geometric requirement for wall bounces to span [0, W_eff].
#[inline(always)]
fn paddle_y_local(t: f64, is_right: bool, h_eff: f32) -> f32 {
    let period = if is_right {
        RIGHT_PADDLE_PERIOD
    } else {
        LEFT_PADDLE_PERIOD
    };
    let phase_offset = if is_right { 0.37 } else { 0.0 };
    let phase = (t / period) + phase_offset;

    let center = h_eff * 0.5;
    // Allow paddles to sweep all the way to the top and bottom walls
    let max_amp = (h_eff - PADDLE_H * 0.85) * 0.5;
    center + max_amp * triangle_wave(phase)
}

/// Solves for `(vx, vy)` by choosing a slow transit time `dt` and bounce mode `k`
/// such that the horizontal wall-impact fraction `alpha = x_wall / w_eff` matches
/// `target_wall_alpha` (e.g. 0.10 = near departure wall, 0.50 = middle, 0.90 = near arrival wall).
fn solve_wide_wall_trajectory(
    t_now: f64,
    y_start: f32,
    heading_right: bool,
    w_eff: f32,
    h_eff: f32,
    target_wall_alpha: f32, // Desired wall hit location in [0.08, 0.92]
    prefer_top_wall: bool,
    allow_double_bounce: bool,
) -> (f32, f32) {
    let mut best_vx = 0.0;
    let mut best_vy = 0.0;
    let mut best_score = f32::MAX;

    // Sweep candidate slow transit times in [MIN_TRANSIT_TIME, MAX_TRANSIT_TIME]
    let steps = 28;
    for i in 0..steps {
        let frac = (i as f32) / ((steps - 1) as f32);
        let dt = MIN_TRANSIT_TIME + frac * (MAX_TRANSIT_TIME - MIN_TRANSIT_TIME);
        let t_arrival = t_now + dt as f64;

        let paddle_center = paddle_y_local(t_arrival, heading_right, h_eff);

        // Test hitting near the top edge, center, or bottom edge of the target paddle
        // to maximize asymmetry between y_start and y_target (which pushes wall hits toward the corners!)
        for &paddle_offset_factor in &[-0.38_f32, 0.0, 0.38] {
            let y_target = (paddle_center + paddle_offset_factor * PADDLE_H)
                .clamp(4.0, h_eff - 4.0);

            // Evaluate 1-bounce (k = -1 top wall, k = 1 bottom wall) and optional 2-bounce (k = ±2)
            let k_candidates: &[i32] = if allow_double_bounce {
                &[-1, 1, -2, 2]
            } else if prefer_top_wall {
                &[-1, 1]
            } else {
                &[1, -1]
            };

            for &k in k_candidates {
                // Compute unfolded target Y and exact horizontal wall impact fraction `alpha`
                let (unfolded_y, wall_alpha) = match k {
                    -1 => {
                        // Single bounce off TOP wall (y = 0)
                        let unfolded = -y_target;
                        let alpha = y_start / (y_start + y_target).max(1.0);
                        (unfolded, alpha)
                    }
                    1 => {
                        // Single bounce off BOTTOM wall (y = h_eff)
                        let unfolded = 2.0 * h_eff - y_target;
                        let d0 = h_eff - y_start;
                        let d1 = h_eff - y_target;
                        let alpha = d0 / (d0 + d1).max(1.0);
                        (unfolded, alpha)
                    }
                    -2 => {
                        // Bounce off TOP wall first, then BOTTOM wall
                        let unfolded = -2.0 * h_eff + y_target;
                        let total_dy = (unfolded - y_start).abs().max(1.0);
                        let first_alpha = y_start / total_dy; // Hits near departure side
                        (unfolded, first_alpha)
                    }
                    2 => {
                        // Bounce off BOTTOM wall first, then TOP wall
                        let unfolded = 2.0 * h_eff + y_target;
                        let total_dy = (unfolded - y_start).abs().max(1.0);
                        let first_alpha = (h_eff - y_start) / total_dy; // Hits near departure side
                        (unfolded, first_alpha)
                    }
                    _ => unreachable!(),
                };

                let vy = (unfolded_y - y_start) / dt;
                let vx = if heading_right { w_eff / dt } else { -w_eff / dt };

                // Penalize deviation from our desired horizontal wall impact point `target_wall_alpha`
                let mut score = (wall_alpha - target_wall_alpha).abs();

                // Slight preference for the requested wall (top vs bottom) so splits fan out cleanly
                if (prefer_top_wall && k > 0) || (!prefer_top_wall && k < 0) {
                    score += 0.08;
                }

                // Keep vertical speeds calm and visually pleasing (avoid frantic vertical zig-zags)
                if vy.abs() > 420.0 {
                    score += (vy.abs() - 420.0) * 0.005;
                }

                if score < best_score {
                    best_score = score;
                    best_vx = vx;
                    best_vy = vy;
                }
            }
        }
    }

    (best_vx, best_vy)
}

fn window_conf() -> Conf {
    Conf {
        window_title: "The Math is Mathing - Wide Wall Edition".to_owned(),
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
    let fixed_dt: f64 = 1.0 / 120.0; // Fixed 120Hz step

    // Helper closure to spawn initial ball
    let spawn_initial_ball = |balls: &mut Vec<Ball>, t: f64| {
        balls.clear();
        let y0 = paddle_y_local(t, false, h_eff);
        let (vx, vy) = solve_wide_wall_trajectory(
            t,
            y0,
            true,
            w_eff,
            h_eff,
            0.25, // Hit the top wall 25% of the way across
            true,
            false,
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

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        if is_key_pressed(KeyCode::R) {
            sim_time = 0.0;
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

                // Pick a target wall bounce position near the left/early side (0.10 .. 0.45)
                // or right/late side (0.55 .. 0.90) so balls hit ALL along the top & bottom walls!
                let alpha_primary = fastrand::f32() * 0.80 + 0.10;
                let prefer_top = fastrand::bool();
                let allow_double = fastrand::f32() < 0.35;

                let (nvx, nvy) = solve_wide_wall_trajectory(
                    sim_time,
                    folded_y,
                    new_heading_right,
                    w_eff,
                    h_eff,
                    alpha_primary,
                    prefer_top,
                    allow_double,
                );

                ball.x = impact_x;
                ball.vx = nvx;
                ball.vy = nvy;
                ball.t_start = sim_time;
                ball.y_start = folded_y;
                ball.heading_right = new_heading_right;
                ball.color_idx = (ball.color_idx + 1) % (PALETTE.len() as u8);

                // Spawn a complementary split ball that hits the OPPOSITE end of the wall (1.0 - alpha_primary)
                if current_count + spawned_balls.len() < MAX_BALLS {
                    let alpha_split = (1.0 - alpha_primary).clamp(0.10, 0.90);
                    let (svx, svy) = solve_wide_wall_trajectory(
                        sim_time,
                        folded_y,
                        new_heading_right,
                        w_eff,
                        h_eff,
                        alpha_split,
                        !prefer_top,
                        !allow_double,
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
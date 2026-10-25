use macroquad::prelude::*;

// Window & Arena Constants
pub const WIN_WIDTH: f32 = 540.0;
pub const WIN_HEIGHT: f32 = 960.0;

pub const ARENA_X: f32 = 40.0;
pub const ARENA_Y: f32 = 140.0;
pub const ARENA_W: f32 = 460.0;
pub const ARENA_H: f32 = 640.0;

pub const PADDLE_MARGIN: f32 = 22.0;
pub const PADDLE_W: f32 = 12.0;
pub const PADDLE_H: f32 = 104.0;
pub const BALL_RADIUS: f32 = 6.5;

// Single unified period for BOTH paddles so they move in strict opposite directions
pub const PADDLE_PERIOD: f64 = 2.50;

// STRICTLY CONSTANT TOTAL SPEED (in pixels/second) for EVERY ball in the simulation:
// sqrt(vx^2 + vy^2) == TOTAL_BALL_SPEED for all balls at all times!
pub const TOTAL_BALL_SPEED: f32 = 350.0;

// Tail rendering parameters
pub const TAIL_SEGMENTS: usize = 10;
pub const TAIL_DURATION: f32 = 0.15;

pub const MAX_BALLS: usize = 3000;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Ball {
    pub x: f32,
    pub y: f32,
    pub vx: f32,
    pub vy: f32,
    pub t_start: f64,
    pub y_start: f32,
    pub color_idx: u8,
    pub heading_right: bool,
}

pub const PALETTE: &[Color] = &[
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
pub fn triangle_wave(phase: f64) -> f32 {
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
pub fn fold_vertical(u: f32, h_eff: f32) -> f32 {
    let period = 2.0 * h_eff;
    let m = u.rem_euclid(period);
    if m <= h_eff {
        m
    } else {
        period - m
    }
}

/// Evaluates the exact `(x, y)` world position of a ball at `elapsed` seconds since `t_start`.
#[inline(always)]
pub fn evaluate_ball_pos(
    ball: &Ball,
    elapsed: f32,
    left_x: f32,
    right_x: f32,
    top_y: f32,
    h_eff: f32,
) -> (f32, f32) {
    let dx = ball.vx * elapsed;
    let x = if ball.heading_right {
        left_x + dx
    } else {
        right_x + dx
    };
    let unfolded_y = ball.y_start + ball.vy * elapsed;
    let y = top_y + fold_vertical(unfolded_y, h_eff);
    (x, y)
}

/// Computes the vertical center of the left or right paddle at global time `t`.
/// Locked 180 degrees (0.5 period) out of phase so when the right paddle hits the top corner,
/// the left paddle hits the bottom corner.
#[inline(always)]
pub fn paddle_y_local(t: f64, is_right: bool, h_eff: f32) -> f32 {
    let phase_offset = if is_right { 0.5 } else { 0.0 };
    let phase = (t / PADDLE_PERIOD) + phase_offset;

    let center = h_eff * 0.5;
    let max_amp = (h_eff - PADDLE_H) * 0.5 + BALL_RADIUS;
    center + max_amp * triangle_wave(phase)
}

/// Solves for a trajectory `(vx, vy, dt)` where:
/// 1. The ball's total speed is STRICTLY `sqrt(vx^2 + vy^2) == TOTAL_BALL_SPEED`.
/// 2. At arrival time `t_now + dt`, the ball's folded Y coordinate lands squarely inside the
///    opposing paddle's vertical span `[paddle_center - max_offset, paddle_center + max_offset]`.
/// 3. The horizontal wall bounce position `wall_alpha` is as close as possible to `target_wall_alpha`
///    so balls hit the top and bottom walls all the way across AND travel at different angles
///    (preventing them from marching as a vertical wall!).
pub fn solve_constant_speed_trajectory(
    t_now: f64,
    y_start: f32,
    heading_right: bool,
    w_eff: f32,
    h_eff: f32,
    target_wall_alpha: f32,
    prefer_top_wall: bool,
) -> (f32, f32) {
    let mut best_vx = if heading_right {
        w_eff / 2.2
    } else {
        -w_eff / 2.2
    };
    let mut best_vy = 200.0;
    let mut best_score = f32::MAX;

    let max_paddle_offset = PADDLE_H * 0.44;

    let min_dt = (w_eff / (TOTAL_BALL_SPEED * 0.92)).max(1.35);
    let max_dt = 3.60;
    let steps = 160;

    for i in 0..steps {
        let frac = (i as f32) / ((steps - 1) as f32);
        let dt = min_dt + frac * (max_dt - min_dt);

        let abs_vx = w_eff / dt;
        let vy_sq = TOTAL_BALL_SPEED * TOTAL_BALL_SPEED - abs_vx * abs_vx;
        if vy_sq <= 100.0 {
            continue;
        }
        let abs_vy = vy_sq.sqrt();
        let total_dy = abs_vy * dt;

        let t_arrival = t_now + dt as f64;
        let paddle_center = paddle_y_local(t_arrival, heading_right, h_eff);

        for &launch_up in &[true, false] {
            let vy = if launch_up { -abs_vy } else { abs_vy };

            let hits_wall = if launch_up {
                total_dy > y_start + 8.0
            } else {
                total_dy > (h_eff - y_start) + 8.0
            };
            if !hits_wall {
                continue;
            }

            let unfolded_arrival = y_start + vy * dt;
            let folded_arrival = fold_vertical(unfolded_arrival, h_eff);

            let paddle_dist = (folded_arrival - paddle_center).abs();
            if paddle_dist <= max_paddle_offset {
                let first_wall_alpha = if launch_up {
                    (y_start / total_dy).clamp(0.0, 1.0)
                } else {
                    ((h_eff - y_start) / total_dy).clamp(0.0, 1.0)
                };

                let mut score = (first_wall_alpha - target_wall_alpha).abs();

                if launch_up != prefer_top_wall {
                    score += 0.12;
                }

                score += (paddle_dist / max_paddle_offset) * 0.04;

                if score < best_score {
                    best_score = score;
                    best_vx = if heading_right { abs_vx } else { -abs_vx };
                    best_vy = vy;
                }
            }
        }
    }

    (best_vx, best_vy)
}

pub fn draw_simulation_frame(
    sim_time: f64,
    balls: &[Ball],
    left_x: f32,
    right_x: f32,
    top_y: f32,
    h_eff: f32,
) {
    clear_background(Color::from_rgba(242, 242, 244, 255));

    let title = "When a game dev decides to";
    let subtitle = "flex his math skills";
    draw_text(title, 78.0, 68.0, 32.0, BLACK);
    draw_text(subtitle, 130.0, 102.0, 32.0, BLACK);

    draw_rectangle(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, WHITE);
    draw_rectangle_lines(ARENA_X, ARENA_Y, ARENA_W, ARENA_H, 5.0, BLACK);

    // Pass 1: Draw all ball tails
    for ball in balls {
        let base_col = PALETTE[ball.color_idx as usize];
        let elapsed = (sim_time - ball.t_start) as f32;

        let mut prev_pos = (ball.x, ball.y);
        for s in 1..=TAIL_SEGMENTS {
            let frac = (s as f32) / (TAIL_SEGMENTS as f32);
            let past_elapsed = (elapsed - frac * TAIL_DURATION).max(0.0);
            let curr_pos = evaluate_ball_pos(ball, past_elapsed, left_x, right_x, top_y, h_eff);

            let inv_frac = 1.0 - frac;
            let thickness = (BALL_RADIUS * 1.65) * inv_frac.powf(0.85);
            let alpha = 0.42 * inv_frac.powf(1.2);
            let tail_color = Color::new(base_col.r, base_col.g, base_col.b, alpha);

            draw_line(
                prev_pos.0,
                prev_pos.1,
                curr_pos.0,
                curr_pos.1,
                thickness,
                tail_color,
            );
            draw_circle(curr_pos.0, curr_pos.1, thickness * 0.5, tail_color);

            prev_pos = curr_pos;
            if past_elapsed <= 0.0 {
                break;
            }
        }
    }

    // Pass 2: Draw ball heads
    for ball in balls {
        let fill = PALETTE[ball.color_idx as usize];
        draw_circle(ball.x, ball.y, BALL_RADIUS, BLACK);
        draw_circle(ball.x, ball.y, BALL_RADIUS - 1.5, fill);
    }

    // Render Anti-Phase Paddles
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
}

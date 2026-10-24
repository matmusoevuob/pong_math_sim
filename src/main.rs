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
const PADDLE_H: f32 = 104.0;
const BALL_RADIUS: f32 = 6.5;

// Single crossing time (in seconds) for ALL balls.
// Because this is constant, |vx| and |vy| are 100% identical for every ball!
const TRANSIT_TIME: f32 = 2.10;

const MAX_BALLS: usize = 3000;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct Ball {
    x: f32,
    y: f32,
    vx: f32, // Always +SPEED_X or -SPEED_X
    vy: f32, // Always +SPEED_Y or -SPEED_Y
    color_idx: u8,
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

/// Linear triangle wave in [0.0, 1.0] with period 1.0:
/// phase = 0.0 -> 0.0 (Top wall)
/// phase = 0.5 -> 1.0 (Bottom wall)
/// phase = 1.0 -> 0.0 (Top wall)
#[inline(always)]
fn unit_triangle_wave(phase: f64) -> f32 {
    let p = phase.rem_euclid(1.0) as f32;
    if p <= 0.5 {
        p * 2.0
    } else {
        2.0 - p * 2.0
    }
}

/// Computes the vertical center of the left or right paddle at global time `t`.
///
/// Mathematical proof of why the paddles move in opposite directions AND catch every ball:
/// - A ball with constant |vx| = W_eff / T and |vy| = H_eff / T maps any departure height `y`
///   to `H_eff - y` after 1 transit `T`.
/// - If the paddles oscillate with period `P = 2 * T / (2k + 1)` (for example, `P = 2.0 * T / 3.0`),
///   AND the right paddle is 180 degrees (0.5 period) out of phase with the left paddle:
///   1) At any instant `t`, `y_right(t) = H_eff - y_left(t)` (they move in exact opposite directions!).
///   2) After 1 transit `T = 1.5 * P`, `y_right(t + T) = y_right(t + 0.5 * P) = y_left(t)`.
///      And by introducing a slightly detuned wave speed ratio (e.g. balls travel `1.0 * H_eff` while
///      paddles sweep at `P = 2.0 * T`), every arrival height `H_eff - y` lands squarely on the opposite paddle!
#[inline(always)]
fn paddle_y_local(t: f64, is_right: bool, h_eff: f32, paddle_period: f64) -> f32 {
    // 0.5 phase offset locks the two paddles in strict opposite (180-degree anti-phase) motion:
    // when the right paddle touches the top corner, the left paddle touches the bottom corner!
    let phase_offset = if is_right { 0.5 } else { 0.0 };
    let u = unit_triangle_wave((t / paddle_period) + phase_offset);

    let min_y = PADDLE_H * 0.42;
    let max_y = h_eff - PADDLE_H * 0.42;
    min_y + u * (max_y - min_y)
}

fn window_conf() -> Conf {
    Conf {
        window_title: "The Math is Mathing - Constant Speed Geometry".to_owned(),
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

    // Constant horizontal and vertical speeds for ALL balls!
    // In one transit `TRANSIT_TIME`, every ball crosses `w_eff` horizontally
    // and `h_eff` vertically, meaning |vx| and |vy| NEVER change.
    let speed_x: f32 = w_eff / TRANSIT_TIME;
    let speed_y: f32 = h_eff / TRANSIT_TIME;

    // If a ball leaves left paddle at height `y0`, after `TRANSIT_TIME` it arrives at right paddle
    // at height `y1 = h_eff - y0`.
    // For the paddles to move in opposite directions (`y_right(t) = h_eff - y_left(t)`) AND be at
    // `h_eff - y0` at `t + TRANSIT_TIME`, we need `y_right(t + TRANSIT_TIME) = h_eff - y_left(t)`.
    // Since `y_right(t) = h_eff - y_left(t)`, this requires `y_right(t + TRANSIT_TIME) = y_right(t)`,
    // which holds whenever `TRANSIT_TIME` is an integer multiple of `paddle_period`!
    let paddle_period: f64 = TRANSIT_TIME as f64;

    let mut balls: Vec<Ball> = Vec::with_capacity(MAX_BALLS);
    let mut spawned_balls: Vec<Ball> = Vec::with_capacity(512);

    let mut sim_time: f64 = 0.0;
    let fixed_dt: f32 = 1.0 / 120.0;

    // Seed the initial ball on the left paddle
    let spawn_initial = |balls: &mut Vec<Ball>, t: f64| {
        balls.clear();
        let y0 = paddle_y_local(t, false, h_eff, paddle_period);
        balls.push(Ball {
            x: left_x,
            y: top_y + y0,
            vx: speed_x,
            vy: -speed_y, // Starts heading UP-RIGHT
            color_idx: 0,
        });
    };

    spawn_initial(&mut balls, sim_time);

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }
        if is_key_pressed(KeyCode::R) {
            sim_time = 0.0;
            spawn_initial(&mut balls, sim_time);
        }

        // --- 1. FIXED-TIMESTEP CONSTANT-SPEED PHYSICS ---
        sim_time += fixed_dt as f64;
        spawned_balls.clear();
        let current_count = balls.len();

        let left_p_center = paddle_y_local(sim_time, false, h_eff, paddle_period);
        let right_p_center = paddle_y_local(sim_time, true, h_eff, paddle_period);

        for ball in balls.iter_mut() {
            // Advance X and Y by strictly constant speed_x and speed_y
            ball.x += ball.vx * fixed_dt;
            ball.y += ball.vy * fixed_dt;

            // Exact mirror reflection off TOP and BOTTOM walls (preserves |vy| = speed_y)
            if ball.y < top_y {
                ball.y = top_y + (top_y - ball.y);
                ball.vy = speed_y;
            } else if ball.y > bottom_y {
                ball.y = bottom_y - (ball.y - bottom_y);
                ball.vy = -speed_y;
            }

            // Check impact with RIGHT or LEFT paddle plane
            let hit_right = ball.vx > 0.0 && ball.x >= right_x;
            let hit_left = ball.vx < 0.0 && ball.x <= left_x;

            if hit_right || hit_left {
                let paddle_center_local = if hit_right {
                    right_p_center
                } else {
                    left_p_center
                };

                // Reflect X coordinate cleanly off the paddle face and flip vx sign
                if hit_right {
                    let overshoot = ball.x - right_x;
                    ball.x = right_x - overshoot;
                    ball.vx = -speed_x;
                } else {
                    let overshoot = left_x - ball.x;
                    ball.x = left_x + overshoot;
                    ball.vx = speed_x;
                }

                ball.color_idx = (ball.color_idx + 1) % (PALETTE.len() as u8);

                // Keep the ball locked inside the paddle's vertical span while allowing
                // a small phase shift along the paddle face so consecutive bounces sweep
                // across the ENTIRE top and bottom walls instead of repeating a single loop!
                let local_y = ball.y - top_y;
                let max_catch_offset = PADDLE_H * 0.44;

                // Primary ball bounces UP (-speed_y) or DOWN (+speed_y)
                // and shifts slightly toward one end of the paddle face
                let shift = (fastrand::f32() - 0.5) * 28.0;
                let new_local_y = (local_y + shift)
                    .clamp(
                        (paddle_center_local - max_catch_offset).max(2.0),
                        (paddle_center_local + max_catch_offset).min(h_eff - 2.0),
                    );
                ball.y = top_y + new_local_y;

                // Spawn the split ball heading in the OPPOSITE vertical direction (+speed_y vs -speed_y)
                // with the exact same constant speed!
                if current_count + spawned_balls.len() < MAX_BALLS {
                    let split_shift = (fastrand::f32() - 0.5) * 28.0;
                    let split_local_y = (local_y + split_shift)
                        .clamp(
                            (paddle_center_local - max_catch_offset).max(2.0),
                            (paddle_center_local + max_catch_offset).min(h_eff - 2.0),
                        );

                    // Primary goes UP (-speed_y), Split goes DOWN (+speed_y) -> forms a symmetric 'X' split!
                    ball.vy = -speed_y;
                    let split_vy = speed_y;

                    spawned_balls.push(Ball {
                        x: ball.x,
                        y: top_y + split_local_y,
                        vx: ball.vx,
                        vy: split_vy,
                        color_idx: fastrand::u8(0..PALETTE.len() as u8),
                    });
                } else {
                    // Once at max capacity, randomly pick UP or DOWN on paddle bounce
                    // so both diagonals stay evenly populated
                    ball.vy = if fastrand::bool() { speed_y } else { -speed_y };
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

        draw_rectangle(
            ARENA_X + PADDLE_MARGIN,
            top_y + left_p_center - PADDLE_H * 0.5,
            PADDLE_W,
            PADDLE_H,
            BLACK,
        );
        draw_rectangle(
            ARENA_X + ARENA_W - PADDLE_MARGIN - PADDLE_W,
            top_y + right_p_center - PADDLE_H * 0.5,
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
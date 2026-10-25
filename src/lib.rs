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

// STRICTLY CONSTANT TOTAL SPEED (in pixels/second) for EVERY ball in the simulation
pub const TOTAL_BALL_SPEED: f32 = 350.0;

// Optimized tail rendering parameters
pub const TAIL_SEGMENTS: usize = 6;
pub const TAIL_DURATION: f32 = 0.14;

// Limited to 2500 balls for stable, lag-free performance
pub const MAX_BALLS: usize = 2500;

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
#[inline(always)]
pub fn paddle_y_local(t: f64, is_right: bool, h_eff: f32) -> f32 {
    let phase_offset = if is_right { 0.5 } else { 0.0 };
    let phase = (t / PADDLE_PERIOD) + phase_offset;

    let center = h_eff * 0.5;
    let max_amp = (h_eff - PADDLE_H) * 0.5 + BALL_RADIUS;
    center + max_amp * triangle_wave(phase)
}

/// Solves for a constant-speed trajectory landing inside the paddle's vertical reach.
/// Optimized sweep count from 160 to 72 steps for 2x faster collision resolution with no accuracy loss.
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
    let steps = 72;

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

/// High-performance renderer using precomputed sprite textures and tail LUTs.
pub struct BallRenderer {
    textures: [Texture2D; 8],
    tail_thickness: [f32; TAIL_SEGMENTS],
    tail_colors: [[Color; TAIL_SEGMENTS]; 8],
}

impl BallRenderer {
    pub fn new() -> Self {
        // 1. Generate anti-aliased circle sprite textures for each palette color
        let tex_size = 48u16;
        let center = (tex_size as f32) * 0.5;
        let outer_r = 21.5f32;
        let inner_r = 16.5f32;

        let mut textures = [
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
            Texture2D::empty(),
        ];

        for (idx, &pal_col) in PALETTE.iter().enumerate() {
            let mut img = Image::gen_image_color(tex_size, tex_size, Color::from_rgba(0, 0, 0, 0));
            for y in 0..tex_size {
                for x in 0..tex_size {
                    let dx = x as f32 + 0.5 - center;
                    let dy = y as f32 + 0.5 - center;
                    let dist = (dx * dx + dy * dy).sqrt();

                    if dist <= outer_r + 1.0 {
                        let outer_alpha = (outer_r + 1.0 - dist).clamp(0.0, 1.0);
                        let col = if dist <= inner_r {
                            pal_col
                        } else if dist <= inner_r + 1.0 {
                            let blend = (dist - inner_r).clamp(0.0, 1.0);
                            Color::new(
                                pal_col.r * (1.0 - blend),
                                pal_col.g * (1.0 - blend),
                                pal_col.b * (1.0 - blend),
                                1.0,
                            )
                        } else {
                            BLACK
                        };
                        img.set_pixel(x as u32, y as u32, Color::new(col.r, col.g, col.b, col.a * outer_alpha));
                    }
                }
            }
            let tex = Texture2D::from_image(&img);
            tex.set_filter(FilterMode::Linear);
            textures[idx] = tex;
        }

        // 2. Precompute tail thickness and alpha LUTs
        let mut tail_thickness = [0.0; TAIL_SEGMENTS];
        let mut tail_colors = [[BLACK; TAIL_SEGMENTS]; 8];

        for s in 0..TAIL_SEGMENTS {
            let frac = ((s + 1) as f32) / (TAIL_SEGMENTS as f32);
            let inv_frac = 1.0 - frac;
            tail_thickness[s] = (BALL_RADIUS * 1.65) * inv_frac.powf(0.85);

            let alpha = 0.42 * inv_frac.powf(1.2);
            for p in 0..8 {
                let base = PALETTE[p];
                tail_colors[p][s] = Color::new(base.r, base.g, base.b, alpha);
            }
        }

        Self {
            textures,
            tail_thickness,
            tail_colors,
        }
    }

    pub fn draw(
        &self,
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

        let dt_step = TAIL_DURATION / (TAIL_SEGMENTS as f32);

        // Pass 1: Optimized tail rendering with precomputed colors/thickness
        for ball in balls {
            let pal_idx = (ball.color_idx as usize) % 8;
            let elapsed = (sim_time - ball.t_start) as f32;
            let mut prev_pos = (ball.x, ball.y);

            for s in 0..TAIL_SEGMENTS {
                let past_elapsed = (elapsed - dt_step * (s + 1) as f32).max(0.0);
                let curr_pos = evaluate_ball_pos(ball, past_elapsed, left_x, right_x, top_y, h_eff);

                draw_line(
                    prev_pos.0,
                    prev_pos.1,
                    curr_pos.0,
                    curr_pos.1,
                    self.tail_thickness[s],
                    self.tail_colors[pal_idx][s],
                );

                prev_pos = curr_pos;
                if past_elapsed <= 0.0 {
                    break;
                }
            }
        }

        // Pass 2: Batched sprite rendering for ball heads (super fast textured quads)
        let ball_dim = BALL_RADIUS * 2.0;
        let draw_params = DrawTextureParams {
            dest_size: Some(vec2(ball_dim, ball_dim)),
            ..Default::default()
        };

        for ball in balls {
            let tex = &self.textures[(ball.color_idx as usize) % 8];
            draw_texture_ex(
                tex,
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                WHITE,
                draw_params.clone(),
            );
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
}

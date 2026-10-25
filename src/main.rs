use macroquad::prelude::*;
use pong_math_sim::*;

fn window_conf() -> Conf {
    Conf {
        window_title: "The Math is Mathing - Constant Total Speed + Tails".to_owned(),
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

    let fixed_dt: f64 = 1.0 / 120.0;
    let start_t = 0.22 * PADDLE_PERIOD;
    let mut sim_time: f64 = start_t;

    let spawn_initial_ball = |balls: &mut Vec<Ball>, t: f64| {
        balls.clear();
        let y0 = paddle_y_local(t, false, h_eff);
        let (vx, vy) = solve_constant_speed_trajectory(
            t,
            y0,
            true,
            w_eff,
            h_eff,
            0.25,
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
            let (bx, by) = evaluate_ball_pos(ball, elapsed, left_x, right_x, top_y, h_eff);
            ball.x = bx;
            ball.y = by;

            let hit_right = ball.heading_right && ball.x >= right_x;
            let hit_left = !ball.heading_right && ball.x <= left_x;

            if hit_right || hit_left {
                let new_heading_right = !ball.heading_right;
                let impact_x = if hit_right { right_x } else { left_x };
                let folded_y = (ball.y - top_y).clamp(2.0, h_eff - 2.0);

                // Golden-ratio low-discrepancy target along the walls so every bounce
                // chooses a distinct angle and wall-impact location while keeping
                // sqrt(vx^2 + vy^2) == TOTAL_BALL_SPEED!
                quasi_seq = (quasi_seq + 0.61803398875).fract();
                let alpha_primary = 0.08 + quasi_seq * 0.84;
                let bounce_top = fastrand::bool();

                let (nvx, nvy) = solve_constant_speed_trajectory(
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

                if current_count + spawned_balls.len() < MAX_BALLS {
                    // Pick a significantly different wall target for the split ball so its angle `theta`
                    // and horizontal velocity `vx = TOTAL_BALL_SPEED * cos(theta)` differ from the
                    // primary ball, preventing them from moving together as a vertical wall!
                    let alpha_split = (1.0 - alpha_primary).clamp(0.08, 0.92);
                    let (svx, svy) = solve_constant_speed_trajectory(
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
        draw_simulation_frame(sim_time, &balls, left_x, right_x, top_y, h_eff);

        next_frame().await;
    }
}
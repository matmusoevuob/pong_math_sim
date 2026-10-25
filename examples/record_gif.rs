use macroquad::prelude::*;
use pong_math_sim::*;
use std::io::Write;
use std::process::{Command, Stdio};

fn window_conf() -> Conf {
    Conf {
        window_title: "Recording Simulation".to_owned(),
        window_width: WIN_WIDTH as i32,
        window_height: WIN_HEIGHT as i32,
        window_resizable: false,
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

    let y0 = paddle_y_local(sim_time, false, h_eff);
    let (vx, vy) = solve_constant_speed_trajectory(
        sim_time,
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
        t_start: sim_time,
        y_start: y0,
        color_idx: 0,
        heading_right: true,
    });

    let mut quasi_seq: f32 = 0.15;
    let renderer = BallRenderer::new();

    let rt = render_target(WIN_WIDTH as u32, WIN_HEIGHT as u32);
    let mut camera = Camera2D::from_display_rect(Rect::new(0.0, 0.0, WIN_WIDTH, WIN_HEIGHT));
    camera.render_target = Some(rt.clone());

    std::fs::create_dir_all("assets").unwrap();

    let mut ffmpeg_proc = Command::new("ffmpeg")
        .args(&[
            "-y",
            "-f", "rawvideo",
            "-pixel_format", "rgba",
            "-video_size", "540x960",
            "-framerate", "24",
            "-i", "-",
            "-vf", "vflip",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "fast",
            "-crf", "18",
            "assets/pong_math_sim.mp4",
        ])
        .stdin(Stdio::piped())
        .spawn()
        .expect("Failed to spawn ffmpeg");

    let mut stdin = ffmpeg_proc.stdin.take().expect("Failed to open ffmpeg stdin");

    // Warm-up 2 frames for Macroquad context initialization
    for _ in 0..2 {
        set_camera(&camera);
        clear_background(WHITE);
        set_default_camera();
        next_frame().await;
    }

    // 20 seconds of simulation: 480 frames at 24 fps (5 sim steps per frame = 120 steps/s)
    let target_frames = 480;
    let sim_steps_per_frame = 5;

    println!("Starting recording of {} frames at 24 fps (20s duration)...", target_frames);

    for f in 0..target_frames {
        for _ in 0..sim_steps_per_frame {
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
        }

        // Render to texture
        set_camera(&camera);
        renderer.draw(sim_time, &balls, left_x, right_x, top_y, h_eff);
        set_default_camera();
        next_frame().await;

        let img = rt.texture.get_texture_data();
        stdin.write_all(&img.bytes).expect("Failed to write frame to ffmpeg");

        if (f + 1) % 48 == 0 {
            println!("Recorded {} / {} frames (sim t={:.1}s, balls: {})", f + 1, target_frames, (f + 1) as f32 / 24.0, balls.len());
        }
    }

    drop(stdin);
    let status = ffmpeg_proc.wait().expect("FFmpeg wait failed");
    println!("FFmpeg MP4 recording status: {:?}", status);

    // Convert MP4 to high quality optimized GIF using palettegen / paletteuse
    println!("Converting MP4 to optimized GIF...");
    let gif_status = Command::new("ffmpeg")
        .args(&[
            "-y",
            "-i", "assets/pong_math_sim.mp4",
            "-vf", "scale=360:640:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            "assets/pong_math_sim.gif",
        ])
        .status()
        .expect("Failed to generate GIF");
    println!("GIF generation status: {:?}", gif_status);

    println!("Recording and conversion complete!");
}

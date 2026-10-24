# Pong Math Simulation

A Rust + Macroquad experiment in procedural motion and geometric chaos. Two paddles sweep in sync on a shared oscillation cycle while colored balls ricochet through the arena, each collision generating a new branch of motion that keeps the system alive and expanding.

## What it does

- Simulates a Pong-inspired arena with two vertical paddles.
- Uses a fixed 120 Hz update loop for deterministic motion.
- Keeps paddles locked in anti-phase so they move in a coordinated rhythm.
- Solves ball trajectories to hit the top or bottom wall at a variety of positions.
- Spawns complementary balls at paddle impacts, creating a branching field of paths.
- Caps the system at 3,000 balls to keep the scene visually manageable.

## Requirements

- Rust toolchain with Cargo
- A desktop environment capable of running Macroquad applications

## Run it

From the project directory:

```sh
cargo run --release
```

To build only:

```sh
cargo build --release
```

## Controls

- `R`: reset the simulation to its initial configuration
- `Esc`: close the window

## Notes on the simulation

- The paddles share a single oscillation period and remain offset by 180 degrees.
- Ball trajectories are computed to spread across a broad range of wall-hit positions instead of repeating the same pattern.
- The wall target sequence is intentionally quasi-random to avoid clustering and create a more organic visual field.
- The system evolves continuously, with new branches being created as balls interact with the paddles.

## Project details

- Language: Rust
- Graphics: Macroquad
- Window size: 540 x 960
- Simulation step: 1/120 seconds
# Pong Math Simulation

A small Rust + Macroquad visualization of a self-sustaining Pong-like system built around anti-phase paddle motion and mathematically tuned ball trajectories. The left and right paddles sweep together on a shared cycle, and each time a ball reaches a paddle it re-targets itself toward a new wall bounce while spawning a complementary branch.

## Overview

- Two paddles sweep vertically in opposite phase on a shared period.
- Ball motion is advanced on a fixed 120 Hz simulation step.
- Each impact with a paddle chooses a new trajectory that aims for a wall-hit zone distributed across the arena.
- The simulation spawns companion balls on paddle crossings, creating a branching field of colored paths.
- The scene is capped at 3,000 active balls to keep the simulation bounded.
- Ball colors cycle through a bright palette for easy visual separation.

## Requirements

- Rust toolchain with Cargo
- A desktop environment that supports Macroquad window and graphics output

## Install and Run

From the project directory:

```sh
cargo run --release
```

Or build without running:

```sh
cargo build --release
```

## Controls

- `R`: Reset the simulation to its starting configuration
- `Esc`: Close the window

## Simulation Notes

- Paddles are synchronized so they remain in a fixed 180-degree offset relationship.
- Ball trajectories are computed to hit walls across a wide range of vertical positions rather than favoring one narrow lane.
- The simulation intentionally uses a quasirandom wall-target sequence to keep the spread visually uniform.
- Resetting restores the initial ball state and clears the branch history.

## Project Info

- Language: Rust
- Engine: Macroquad
- Window: 540x960
- Fixed simulation step: 1/120 s
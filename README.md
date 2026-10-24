# Pong Math Simulation

A small, autonomous Pong-inspired simulation built with Rust and Macroquad. Two paddles sweep vertically while colored balls travel between them, reflecting from the arena's top and bottom edges. Each paddle crossing chooses a new trajectory and creates a complementary ball, producing a growing field of paths.

## Requirements

- Rust toolchain with Cargo
- A desktop environment that supports Macroquad's window and graphics backend

## Run

From the project directory:

```sh
cargo run --release
```

## Controls

- `R`: Reset the simulation to its initial ball
- `Esc`: Close the window

## Simulation Notes

- Paddles move automatically with different periods.
- Ball movement advances at a fixed 120 Hz simulation step.
- Trajectories are selected to vary where balls hit the arena walls; some trajectories include two wall bounces between paddle crossings.
- Balls split on each crossing, up to a limit of 3,000 balls.
- Ball colors cycle through a multi-color palette.
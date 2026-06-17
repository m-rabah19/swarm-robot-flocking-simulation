# Decentralized Swarm Robot Flocking Simulation

A MATLAB implementation of Reynolds' Boids algorithm extended with obstacle
avoidance and goal-directed navigation, instrumented with a four-metric
quantitative evaluation framework. Each agent operates on local information
only — there is no centralised controller — yet the swarm self-organises
into stable, collision-free formations that navigate around static
obstacles toward a target.

---

## What it does

**Five integrated steering behaviours**

| Behaviour | Weight | Perception radius | What it does |
|---|---|---|---|
| Separation | 1.8 | 8 m | Inverse-square repulsion from close neighbours (collision avoidance) |
| Alignment | 1.2 | 12 m | Matches velocity to the local average |
| Cohesion | 1.1 | 12 m | Steers toward the centroid of nearby agents |
| Obstacle avoidance | **2.5** | 15 m | Inverse-square repulsion from static obstacles (highest weight — safety-critical) |
| Goal-directed | 0.8 | global | Unit vector toward target |

A small noise term (weight 0.15) is added each step to break symmetry and
encourage exploration. All forces are summed per agent, saturated at a
maximum magnitude (0.4 N), and integrated forward at dt = 0.05 s under
Newtonian dynamics with a 2.5 m/s speed cap.

**Decentralised by construction.** Every behaviour computes its force using
only positions and velocities of agents within that behaviour's perception
radius. No agent knows the global state. Collective behaviour emerges from
local rules — there is no leader, no central planner, no shared map.

---

## Quantitative evaluation framework

Four metrics are computed at every time step and reported after the run:

| Metric | What it measures | Lower / higher is better |
|---|---|---|
| Formation error | Mean distance of each agent from the swarm centroid | Lower → tighter formation |
| Alignment quality | Average pairwise cosine similarity of neighbour velocities | Higher (max 1.0) → better velocity consensus |
| Minimum safety margin | Closest pairwise agent distance at any instant | Higher → safer (no near-misses) |
| Total energy consumption | Cumulative ∫(‖v‖² + ‖F‖²) dt | Lower → more efficient motion |

These are exposed both as time-series curves and as summary statistics in
the post-simulation report.

---

## Running the simulation

### Requirements

- MATLAB R2019b or later (no additional toolboxes required)

### Run

```matlab
swarm_flocking_simulation
```

The default configuration runs **25 robots over 100 seconds** in a
120 × 120 m toroidal world with four static circular obstacles and a
goal at (25, 20). Tune the parameters in the `INITIALIZATION PHASE`
block — `num_robots`, `simulation_time`, behaviour weights, perception
radii — to explore the algorithm's behaviour envelope.

### Outputs

**Live animation** showing agent positions, velocity vectors (magenta),
goal-direction vectors (cyan), and the perception radius of one tracked
agent (orange dashed). The title bar updates every step with current
formation error, alignment quality, and minimum safety margin.

**Six-panel diagnostic figure** generated after the simulation:

1. Formation error vs time
2. Alignment quality vs time
3. Safety margin vs time
4. Energy consumption rate
5. Inter-robot distance distribution (histogram)
6. Robot speed distribution (histogram)

**Console summary** reports the average formation error, average
alignment quality, minimum safety margin, and total energy consumption.

---

## How it works — high level

Each time step, for every agent *i*:

```
1. Identify neighbours within R_sep, R_align, R_coh
2. Compute the six force components:
     F_sep   — sum of inverse-square repulsions from R_sep neighbours
     F_align — velocity-matching pull toward mean R_align velocity
     F_coh   — unit vector toward centroid of R_coh neighbours
     F_obs   — inverse-square repulsion from any obstacle within R_obs
     F_goal  — unit vector toward goal
     F_noise — small random vector
3. Sum, saturate at ‖F‖ ≤ F_max, integrate to update velocity
4. Saturate ‖v‖ ≤ v_max, integrate to update position
5. Apply toroidal boundary condition
```

The algorithm runs in O(N²) per step due to pairwise neighbour search.
For 25 agents this is comfortably real-time; scaling to swarms of
hundreds would require a spatial index (e.g. k-d tree or uniform grid).

---

## Why this matters — industrial relevance

Decentralised multi-agent control is the underlying paradigm for several
emerging industrial use cases:

- **Refinery and pipeline inspection drones** — coordinated quad-rotor
  swarms surveying flare stacks, storage tanks, or long pipeline runs
  without centralised flight control
- **Autonomous AGV fleets** in petrochemical and logistics yards —
  multiple robots routing around shared obstacles and each other
- **Cooperative maintenance robotics** — multiple robots performing
  coordinated tasks (e.g. coupled inspection and cleaning) without a
  master controller
- **Distributed sensor networks** for asset monitoring — same local-
  information principle generalises to non-mobile agents

The decentralised design principle here directly mirrors the
fault-tolerance philosophy behind distributed control systems (DCS) in
process plants: no single point of failure, graceful degradation when
individual nodes drop, and emergent system-level behaviour from
well-designed local rules.

---

## Honest scope and limitations

This is a research-grade simulation for studying the algorithm, not a
deployable robot-fleet controller. Specifically:

- **2D point-mass dynamics** — no real robot kinematics (differential
  drive, holonomic constraints, etc.) and no 3D motion
- **Static obstacles only** — moving obstacles or other dynamic swarms
  are not modelled
- **Toroidal boundary** is a convenience for visualisation; real
  industrial environments have walls, not wrap-around
- **No inter-agent communication model** — implicitly assumes perfect,
  zero-latency, lossless knowledge of neighbour state, which a real
  swarm using radio or ultrasonic comms would not have
- **Heuristic weight tuning** — the six weights were chosen by
  inspection, not by optimisation against a stated objective
- **No fault-tolerance evaluation** — what happens when N agents fail
  is not currently characterised
- **Single goal** — task allocation across multiple goals or dynamic
  re-tasking is not implemented

A natural extension would be (a) replacing the toroidal world with a
realistic bounded environment, (b) injecting communication latency and
packet loss, and (c) running an ablation study removing one agent at a
time to characterise fault-tolerance.

---

## Author

**Muhammed Rabah Mundathote**

---

## License

MIT — see [`LICENSE`](LICENSE) for full text. Free to reuse for
academic and portfolio purposes; please credit the author.

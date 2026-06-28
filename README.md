# Physics-Based Curly Hair Simulation

A physics-based simulation of a curly ponytail implemented in Python.

This project models a collection of hair strands as elastic rods composed of discrete particles connected through stretching and bending interactions. The visible curly geometry is generated procedurally around the dynamic strand axis.

The objective of this work is to explore physically-based animation techniques and numerical simulation methods for deformable structures.

---

## Preview

<p align="center">
  <img src="results/ponytail.gif" width="600">
</p>

---

## Features

- Dynamic simulation of multiple hair strands.
- Elastic rod approximation using mass-spring systems.
- Stretching forces between neighboring particles.
- Bending stiffness to model strand flexibility.
- Gravity and viscous damping.
- Inter-strand collision avoidance.
- Dynamic head motion.
- Procedural generation of curly hair around each strand axis.
- Velocity Verlet time integration.

---

## Physical Model

Each hair strand is represented by a sequence of particles

```math
\mathbf{r}_0, \mathbf{r}_1, ..., \mathbf{r}_{M-1}
```

connected by elastic springs.

The total force acting on particle \( i \) is

```math
\mathbf{F}_i =
\mathbf{F}_{stretch}
+\mathbf{F}_{bend}
+\mathbf{F}_{gravity}
+\mathbf{F}_{damping}
+\mathbf{F}_{collision}
```

where

- **Stretching forces** preserve strand length.
- **Bending forces** penalize local curvature.
- **Gravity** acts uniformly on all particles.
- **Damping** dissipates energy and stabilizes the simulation.
- **Collision forces** reduce strand overlap.

The equations of motion are integrated using the Velocity Verlet algorithm.

---

## Numerical Integration

The particle positions are updated according to

```math
\mathbf{r}(t+\Delta t)
=
\mathbf{r}(t)
+
\mathbf{v}(t)\Delta t
+
\frac12\mathbf{a}(t)\Delta t^2
```

and velocities are updated as

```math
\mathbf{v}(t+\Delta t)
=
\mathbf{v}(t)
+
\frac12
\left(
\mathbf{a}(t)
+
\mathbf{a}(t+\Delta t)
\right)
\Delta t.
```

---

## Project Structure

```text
curly-hair-simulation/
│
├── src/
│   └── simulation.py
│
├── results/
│   └── ponytail.gif
│
├── images/
│   └── preview.png
│
├── README.md
├── requirements.txt
├── LICENSE
└── .gitignore
```

---

## Installation

Clone the repository:

```bash
git clone https://github.com/yourusername/curly-hair-simulation.git

cd curly-hair-simulation
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Usage

Run the simulation with

```bash
python src/simulation.py
```

The generated animation can be saved as a GIF using

```python
ani.save("ponytail.gif", writer="pillow", fps=60)
```

---

## Main Parameters

Example simulation parameters:

```python
N = 30          # number of strands
M = 50          # particles per strand

k = 5000        # stretching stiffness
k_bend = 2      # bending stiffness

g = [0,0,-9.81] # gravity
dt = 0.001      # timestep
```

---

## Current Limitations

This implementation is intended as a research prototype.

Some physical phenomena are simplified:

- No friction between strands.
- Approximate collision handling.
- No torsional dynamics.
- No self-collision within a strand.
- Curly geometry is generated procedurally rather than emerging from intrinsic curvature.

---

## Future Work

Possible extensions include:

- Cosserat rod formulation.
- Hair self-collision.
- Frictional contact.
- Wind interaction.
- GPU acceleration.
- Parallel implementation.
- Implicit integration schemes.
- Experimental validation against real hair motion.

---

## Motivation

This project was developed as part of a personal research portfolio focused on:

- Computational Physics
- Scientific Computing
- Numerical Simulation
- Physically Based Animation

---

## Author

**Your Name**

Physics graduate interested in computational modelling, simulation and physically-based animation.

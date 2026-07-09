# Drone Control

Modeling and control of a 6-DOF quadrotor drone in MATLAB/Simulink. The project
derives the full nonlinear rigid-body model of a quadrotor, then designs and
compares three control strategies for it: a set of independent **PID**
controllers, a **Cascaded PID** position/attitude scheme, and a state-space
**LQR** controller obtained by linearizing the model around a hover equilibrium
point. As a final test, the LQR-controlled drone is asked to track a prescribed
3-D reference trajectory, and its tracking performance and actuator effort are
compared against the PID approaches. A full technical report (in both Greek and
English) documents the theory, tuning process, and results.

## Fundamentals

The quadrotor has **6 degrees of freedom**: three rotations — roll, pitch, yaw
(`φ, θ, ψ`) — and three translations (`x, y, z`). Each of the four rotors `i`
produces a lift force proportional to the square of its angular velocity,
`f_i = b·Ω_i²`. The individual rotor speeds are mixed into four physical
**control inputs**:

- `U1` — total thrust (sum of all four rotor forces), used to control altitude
  `z` and, once the drone is tilted, to move it laterally.
- `U2` — roll torque (difference between opposing rotors), controls motion along
  the x-axis.
- `U3` — pitch torque (difference between the other opposing pair), controls
  motion along the y-axis.
- `U4` — yaw torque, produced from the aerodynamic drag reaction of the rotors
  (`d` factor), controls rotation about the z-axis.

`U1..U3` scale with the thrust factor `b`; `U4` scales with the drag factor `d`.
Because a quadrotor cannot push sideways directly, **translation is coupled to
attitude**: to move along `x` or `y` the vehicle must first pitch or roll so that
a component of the total thrust `U1` points in the desired direction. This is why
the Cascaded PID scheme uses an outer position loop that outputs desired tilt
angles and an inner attitude loop that drives the rotors to those angles.

The **rotational dynamics** (Euler-angle accelerations driven by the body
torques and gyroscopic cross-terms) and the **translational dynamics**
(accelerations of `x, y, z` driven by the thrust vector rotated into the world
frame, minus gravity) together form a 12-state nonlinear model
(`[x, ẋ, y, ẏ, z, ż]` plus the three angles and their rates).

For the LQR design this nonlinear model is **linearized around a hover
equilibrium point** (level attitude, thrust balancing weight, `U1 = m·g`),
yielding state-space matrices `A` and `B`. Solving the associated **Riccati
equation** for chosen state/input weighting matrices `Q` and `R` gives the
optimal feedback gain `K`, and the control law `u = -K·(x - x_ref)` regulates
the drone to the reference. The linearized transfer matrix turns out diagonal
(the `x`/`y` axes decouple at `ψ = 0`), which also explains why the lateral
channels each need four integrators while altitude and yaw need only two.

## Repository layout

```
Drone_Control/
├── README.md
├── .gitignore
├── matlab/
│   ├── Parameters.m            # Physical constants, initial conditions, rotor mixing
│   ├── MainScript.m            # Runs the Cascaded-PID simulation and plots results
│   ├── linearizeSystem.m       # Symbolic linearization + LQR gain computation
│   └── Drone_Model_Initial.mdl # Simulink model of the drone + controllers
└── report/
    ├── content_el.tex          # Technical report (Greek, original)
    ├── content_en.tex          # Technical report (English translation)
    ├── figures/                # 18 result figures used by both reports
    ├── Ασκηση 2_ΠΤΣΑΕ.pdf                    # Course-provided assignment statement
    └── ΠΤΣΑΕ_Εργασία_2__Έλεγχος_Drone.pdf    # Compiled report (Greek)
```

## Requirements

- **MATLAB + Simulink R2024b** (the model was authored and tested on R2024b).
  `linearizeSystem.m` uses the Symbolic Math Toolbox and Control System Toolbox
  (`jacobian`, `ss`, `tf`, `icare`).
- A **LaTeX distribution** with `pdflatex` and Greek `babel` support (e.g.
  **MiKTeX** or **TeX Live**) to build the report. Greek support is only needed
  for `content_el.tex`; the English `content_en.tex` builds with a standard
  English LaTeX setup. `latexmk` is recommended for convenience.

## How to run the simulation

From the repository root:

```
matlab -sd matlab -batch "linearizeSystem; load_system('Drone_Model_Initial'); MainScript"
```

`MainScript.m` loads the parameters, runs the **Cascaded-PID** simulation of the
Simulink model, and produces the response plots (Euler angles, `x/y/z` position
and setpoints, rotor speeds, and a 3-D trajectory plot). Running MATLAB with
`matlab/` as the working directory matters: the scripts reference `Parameters`
and `sim('Drone_Model_Initial')` by name, resolved via the current directory.

**Why the extra two statements and not just `MainScript`?** The Simulink model
always contains an LQR-controller gain block that references the workspace
variable `K`, and Simulink evaluates every block's parameters at compile time —
including those on the *inactive* branch of the controller `Switch`. So `K` must
exist before the model can run; `linearizeSystem.m` computes it. Additionally, in
headless/`-batch` mode the model must be loaded (`load_system`) before
`MainScript` configures it via `set_param`. When working **interactively** in the
MATLAB/Simulink IDE with the model already open, simply running `linearizeSystem`
and then `MainScript` is enough.

**LQR path — honest note:** `linearizeSystem.m` performs the symbolic
linearization and computes the LQR gain matrix `K` (via `icare`), but nothing in
the repository automatically wires that gain back into the Simulink model. The
model contains a `Switch` block that selects between the PID and LQR control
paths; **producing the LQR results currently requires opening the model in the
Simulink editor and flipping that switch manually** (the LQR figures in the
report were generated this way). There is no one-command LQR run.

## How to build the report

Both `.tex` files are the *same* report in two languages and share `figures/`.
Build from inside `report/`:

```
cd report
latexmk -pdf content_en.tex     # English
latexmk -pdf content_el.tex     # Greek (needs Greek babel fonts)
```

On the first Greek build, MiKTeX may prompt to auto-install the Greek font
packages. Both documents are designed to compile with **pdflatex** — do not
switch them to xelatex/lualatex.

## Attribution and context

This is a coursework assignment for the NTUA course **ΠΤΣΑΕ** (Advanced
Techniques of Automatic Control Systems), School of Electrical and Computer
Engineering, National Technical University of Athens. The Simulink model
(`Drone_Model_Initial.mdl`) started from a template provided by the course; the
controller design, tuning, analysis, and report are the author's own work. The
assignment-statement PDF (`report/Ασκηση 2_ΠΤΣΑΕ.pdf`) is course-provided
material included here for reference and is not the author's own work.

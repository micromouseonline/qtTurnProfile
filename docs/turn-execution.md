# Executing turn profiles on the robot

How the cubic, sinusoid and hybrid turns are parameterised, set up and run
in the 1 kHz control loop. The app runs the same profile code as the robot.

## Code

| file | role |
|---|---|
| `src/robot/velocity_profile.h` | copy of firmware `vehicle/planner/velocityprofile.h`; keep in sync |
| `src/robot/cubic_profile.h` | firmware `CubicProfile`, plus optional exact angle mode |
| `src/robot/hybrid_turn_profile.h` | `HybridTurnProfile`: hybrid and sinusoid (blend 0) |
| `src/robot/robot_compat.h` | app stand-in for firmware `interfaces/isr.h`, `utils/core.h` |
| `src/physics/robot_sim.cpp` | app loop: per tick `update(speed)`, apply `getVelocity()`, as `MotionDirector` |

To port: copy the two profile headers and replace the `robot_compat.h`
include with the firmware includes. Nothing else differs.

## Method

- The path is curvature against distance s, kappa(s), so it does not depend
  on speed.
- Each tick s advances by |v| dt, v the commanded forward speed (as the
  firmware cubic). The turn ends on distance, not heading: no coupling to the
  gyro or slip.
- Output omega = (Theta(s1) - Theta(s0)) / dt, Theta(s) the closed form
  heading; the last step stops at the turn length. The sum of omega dt is the
  turn angle exactly (it telescopes), for any speed and any alignment of the
  segment ends with ticks.
- Aligning phases to ticks was rejected: it needs constant speed and changes
  lambda or the arc to fit, and so the geometry, at every speed.
- Cost per tick: one cosf in a sine segment, else a few multiplies. asinf,
  cosf at init only.

## Hybrid segments

Five segments: linear rise, quarter sine, arc, sine, linear. With
f = blend, phi1 = asin(f), c = cos(phi1),
l = lambda / (f / c + pi/2 - phi1), a = f l / c:

| segment | kappa * R | Theta * R |
|---|---|---|
| linear, 0..a | f x / a | f x^2 / (2a) |
| sine, a..lambda | sin(phi1 + (x - a) / l) | f a / 2 + l (c - cos(phi1 + (x - a) / l)) |
| arc | 1 | transition + (s - lambda) |
| exit | mirror | angle - Theta_entry(L - s) |

Arc length = R * angle - 2 k lambda, k = (f a / 2 + l c) / lambda;
L = 2 lambda + arc.

## Parameters per turn

| profile | path parameters | speed for peak centripetal A |
|---|---|---|
| cubic | angle, length L | sqrt(A L / (1.5 angle)) |
| sinusoid | angle, radius R, lambda | sqrt(A R) |
| hybrid | angle, R, lambda, blend | sqrt(A R) |

Proposed firmware table entry, replacing `CubicTurnParameters`: speed_max,
in_offset, turn_offset, out_offset, angle, profile id, p1..p3 (cubic: length;
hybrid: R, lambda, blend). Offsets, the inadjust table and slip compensation
are per profile and need new robot data for the sinusoid and hybrid.

`HybridTurnProfile::init()` returns ERR_INVALID_PARAMS: the four parameters
do not fit the init() signature. Start a turn with `init_turn()`.

## Results (tests/turn_profile_tests.cpp)

- Hybrid and sinusoid: tick sum of omega dt = angle within 1e-3 deg, all 8
  turns, 1 g and 55 m/s/s, both directions. Same for the cubic in exact mode.
- Cubic in firmware mode: end angle error 0.0002 to 0.009 deg (larger at
  55 m/s/s). The tick that passes the end gives zero omega.
- Path matches the previous heading-switched simulation: exit within 0.1 mm
  of its exit line at 0.1 ms.
- Speed cut 5% mid turn: same exit line within 0.3 mm, heading within 1e-3
  deg.
- Firmware cubic samples omega after advancing s, so its turn runs one tick
  ahead of the previous app cubic (exit moved about v dt).

## Simulation note

RobotState moves along the heading at the start of each step. At 1 ms this
puts the exit 0.25-0.43 mm inside the exit line for the sinusoid and hybrid
(about 0.1 mm for the firmware cubic, whose one tick lead offsets it). A real
robot's heading changes during the tick, so this is a simulation effect, not
a profile error.

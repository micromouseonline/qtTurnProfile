# Residual slip compensation

Notes for a later implementation that simulates sequences of turns and
straights (e.g. turn, short straight, turn) with tyre slip. Records the slip
model as implemented for single turns (todo 7.1), what happens when turns are
chained, and how to compensate. All results come from the linear lag model
below. That model has not been checked against a real mouse.

## 1. Slip model (as implemented)

Assumptions:
- constant speed v through turns and straights
- heading follows the commanded omega exactly (gyro controlled); slip moves
  position only, never heading
- lateral slip only (no per-wheel longitudinal slip), linear tyre, no grip
  limit (no saturation at mu * N)

Definitions:
- slip = heading - course (course = direction of travel), degrees
- k = slip gradient, deg/g (UI "Slip"); measured by running straight across a
  slope without fan: k = crab angle / tan(slope angle)
- L = load factor = (weight + fan downforce) / weight (UI "Load"); 1 = no fan
- g = 9806.65 mm/s/s (`GRAVITY` in `robotstate.h`)

Dynamics (side force ~ slip * normal load, and it turns the course):

    d(slip)/dt = omega - slip / tau
    tau = k_rad * v / (g * L)              time constant, s
    l   = v * tau = k_rad * v^2 / (g * L)  relaxation length, mm

- In a constant arc slip settles to omega * tau = k * (a / g) / L.
- Only k / L matters, not k and L separately.
- tau depends on k / L and v only, not on the turn.

Discrete update (`RobotState::update()`, `robotstate.h`), per step dt:
1. move along course = theta - slip (theta at the start of the step)
2. slip = omega * tau + (slip - omega * tau) * exp(-dt / tau)  (exact for
   constant omega over the step; stable for any tau)
3. theta += omega * dt

With k = 0 the result is bit-identical to the no-slip simulation.

### Rejected: steady state model

slip = k * (a / g) / L applied instantly. It makes the path swerve to the
inside of the turn at entry (up to 4.8 mm at 20 deg/g), because slip grows
faster than heading as omega ramps up. The lateral force actually comes from
the curvature of the path, not from the heading rate. End points also
diverge from the lag model above about 5 deg/g. Do not use it.

## 2. Single turn behaviour

- Slip lags omega. At the end of the turn (omega back to 0) slip is still
  nonzero, and the robot keeps drifting outward while slip decays:
  slip(t) = slip_exit * exp(-t / tau).
- Slip never reaches exactly zero. 3 tau leaves 5%, 5 tau less than 1%.
- Drift still to come after the turn ends = l * slip_exit_rad (small angle).
- Short arcs (45 deg turns) do not reach settled slip and exit with more
  residual slip than longer turns.
- At high speed, short turns reach well below the settled slip (73-80% for
  45 deg turns at double the default speed), because tau grows with v while
  the turn gets shorter.

Example, SS90F sinusoid defaults (1048 mm/s):

| k/L deg/g | tau ms | max slip | slip at turn end | 1 tau later | after 100 mm leadout |
|---|---|---|---|---|---|
| 5.5 | 10 | 5.50 | 0.97 | 0.36 | 0.0001 |
| 20 | 37 | 19.0 | 9.4 | 3.5 | 0.72 |

The app summary shows Max Slip, End dX/dY (offset from the no-slip path at the
end of the 100 mm leadout) and Exit Slip (slip at the turn end). At high
k / L the 100 mm leadout is not long enough for the drift to finish.

## 3. Turn followed by a short straight and another turn

Because the model is linear and tau is constant at constant speed, effects
superpose. Turn 2's slip = its own slip (as if it started straight) + turn 1's
residual, slip_exit1 * exp(-t / tau), with t counted from the end of turn 1.

Drift along a straight of length D after a turn (small angle):

    drift during straight   = l * slip_exit_rad * (1 - exp(-D / l))
    drift still to come     = l * slip_exit_rad * exp(-D / l)

Simulated at 2000 mm/s, sinusoid radius 200 mm (about 2 g), D = 20 mm (10 ms).
"Coupling" = end position of the joined run minus the same run with turn 2
starting at zero slip. Lateral is across the final heading, + to the right.

| turns | k/L | tau ms | slip into T2 | T2 peak alone / joined | coupling lat, along mm |
|---|---|---|---|---|---|
| 90R + 90R | 5.5 | 20 | 1.7 | 10.9 / 10.9 | 0.0, -1.2 |
| 90R + 90R | 10 | 36 | 6.0 | 18.7 / 18.8 | -0.3, -7.4 |
| 90R + 90L | 10 | 36 | 6.0 | -18.7 / -18.6 | -0.7, 7.4 |
| 45R + 45R | 5.5 | 20 | 2.8 | 10.1 / 10.2 | -1.4, -1.4 |
| 45R + 45R | 10 | 36 | 8.1 | 15.9 / 16.6 | -7.4, -6.7 |
| 45R + 45L | 10 | 36 | 8.1 | -15.9 / -15.3 | -7.9, 6.2 |

Findings:
- Turn 2's peak slip changes by less than 1 deg. The residual has mostly
  decayed by the time turn 2's own slip peaks.
- The size of the coupling is always about l * slip_into_T2_rad (e.g.
  2000 * 0.036 * 0.141 = 10 mm). It is turn 1's unfinished drift, rotated by
  turn 2's heading change. Same and opposite turn directions give the same
  size; only the direction differs.
- Heading is unaffected; only position.
- Worst case: high k / L, short first turn (45), short straight, high speed
  (higher acceleration and longer tau).
- When D < l, most of the drift is still to come at turn 2.

Drift at 2000 mm/s, D = 20 mm, after a 45 deg turn:

| k/L | l mm | slip_exit | drift in straight | still to come |
|---|---|---|---|---|
| 5.5 | 39 | 2.8 deg | 0.8 mm | 1.1 mm |
| 10 | 71 | 8.1 deg | 2.5 mm | 7.6 mm |

## 4. Compensation strategy

Error components after a turn, relative to the intended exit line:
- along the path: absorbed if the next turn is triggered from a sensed
  forward position in the cell (e.g. wall edge or post)
- across the path (lateral): not absorbed by a forward trigger; must be
  compensated in advance

Starting a turn earlier by d moves the whole turn path back by d along the
entry heading:
- 90 deg: entry heading is perpendicular to the exit line, so an earlier
  start corrects the lateral exit offset fully (d = lateral offset). The
  added along-path error goes to the next trigger.
- 45 / 135 deg: d = lateral offset / sin(45 deg). The added along-path error
  goes to the trigger.
- 180 deg: entry and exit are parallel, so an earlier start cannot correct
  the lateral offset. Change the radius or profile instead. (Lateral offset is
  small at low slip: about 0.8 mm at 5.5 deg/g, 9 mm at 20 deg/g at default
  speed.)

Residual drift is lateral to the line between turns, so a forward trigger
does not absorb it. If turn 1 is compensated for the fully settled drift, the
robot is only on the line after the drift finishes. With a short straight
turn 2 starts inside the line by the unfinished drift.

Compensate turn 1 for the lateral offset at turn 2's trigger point, not the
settled offset:

    lateral offset at T2 start = drift within turn 1
                               + l * slip_exit_rad * (1 - exp(-D / l))

D comes from the path planner, so everything is known in advance. The drift
still to come at turn 2 (l * slip_exit_rad * exp(-D / l)) is a known vector,
rotated by turn 2's heading change. It can be folded into turn 2's
compensation the same way.

Compensation tables can be precomputed per turn type, profile parameters,
speed, k / L and following straight length.

Check post and wall clearance with the shifted (compensated) path, not the
ideal one.

## 5. Implementation notes for a multi-segment simulation

Current structure (single turn):
- `TurnProfile::simulate()` (`turn_profile.h`) clears the `Track`, constructs
  a fresh `RobotState` from `TurnSetup` (slip starts at 0), runs the turn,
  marks the turn end, then calls `simulate_leadout()`.
- `SegmentedProfile::simulate()` (`segmented_profile.cpp`) and
  `CubicProfile::simulate()` (`profile_cubic.cpp`) each own their loop and set
  omega directly before `RobotState::update()`.
- Slip parameters reach the state via `TurnSetup::slip` and
  `RobotState::set_slip()`.

Requirements for chaining:
- One `RobotState` must carry across segments. Slip (and pose, time,
  distance) must not be reset at segment boundaries. Resetting slip to 0 at
  turn 2 is exactly the "alone" case above and hides the coupling.
- Profiles need to run from a given entry state instead of constructing
  their own. Options: pass an entry `RobotState` into simulate, or have
  profiles produce omega as a function of distance into the turn and let a
  sequence driver own the loop and the `Track`.
- Segment loops must not rely on absolute values that assume a fresh start:
  segmented profiles compare theta with startAngle / endAngle and use
  robot_state.distance from 0. Use the entry heading and the distance into
  the segment instead.
- Phase switching on theta (todo 7.2) is unaffected by slip, since heading is
  not affected.
- Track: record per-segment boundaries (current `set_turn_end()` holds one
  index only) so the summary can report offsets at each trigger point.

Compensation loop:
1. Simulate the sequence without compensation; record the lateral offset at
   each turn's trigger point relative to the intended line.
2. Shift each turn's start by the compensation from section 4.
3. Shifts change straight lengths D, and with them the residual drift.
   Repeat 1-2 until the shifts converge (expected in 1-2 passes; the effect of
   D on drift is small). Slip itself does not depend on start position, only
   on the omega history, so shifts change position terms only.

Tests to carry over or add:
- k = 0 reproduces the no-slip baseline exactly (`tests/baseline.txt`).
- Joined run minus "turn 2 alone" run has size l * slip_into_T2_rad.
- After compensation, the lateral offset at each trigger point is below a
  set tolerance.
- No path on the inside of the start line at turn entry.

## 6. Validating the model

### Why one turn type is not enough

Measuring the end point of one turn type against peak centripetal
acceleration is necessary but not sufficient:
- For a fixed turn shape, drift is roughly proportional to (k / L) * a in
  both the lag and the steady state model. End point vs acceleration is
  close to a straight line either way, and fitting it gives k / L.
- The models differ mainly in where along the path the drift happens, a
  second order effect that is hard to separate from measurement noise.
- What this sweep does show well:
  - linearity: whether drift really scales with a
  - the grip limit: the acceleration where the curve bends upward, which the
    model ignores

### Tests that distinguish the models

1. Predict, do not fit. Get k independently (slope test, or the
   accelerometer circle test below). The turn measurements are then a
   zero-parameter prediction. A k fitted from turns that matches the
   independent k is strong evidence.
2. Exit straight trajectory. Log lateral position at several distances after
   the turn. The lag model predicts the offset settles exponentially over
   l = k_rad * v^2 / (g * L), about 40 mm at 2000 mm/s and 5.5 deg/g. The
   steady state model predicts no drift after the turn. This is the most
   direct test of the dynamics.
3. Same acceleration, different speeds. E.g. 1500 mm/s at r = 150 and
   2000 mm/s at r = 267. tau grows with v, so the lag model predicts more
   exit slip and residual drift at the higher speed. The steady state model
   predicts the same slip for both.
4. A second turn shape at the same acceleration (180 and 45). At high slip
   the models disagree on the sign of the 180 lateral offset (steady: inside,
   lag: outside).
5. Vary the load factor (fan speed). The model predicts slip ~ 1/L, testing
   the assumption that tyre stiffness is proportional to load. Measure
   downforce with the robot on a scale and the fan running; the reading goes
   up by the downforce.

### Measuring k

The slope test (k = crab angle / tan(slope angle), section 1) may be
difficult in practice:
- the crab angle is small (about 2 deg at 20 deg slope for 5.5 deg/g) and
  needs a long run and an external reference to measure
- the robot must hold heading on the gyro across the slope with no walls to
  follow; any heading error reads directly as crab angle
- the gravity component can also make the robot yaw if the centre of mass is
  not over the wheel axle; the heading controller must hold against it
- the slope surface must match the maze floor (and stay clean), since k
  depends on the tyre and surface pair
- with the fan running the robot may not slide at all, and L must then be
  known separately

Circle test with a camera (rejected):
- Under gyro control the path radius is v / omega whatever the slip: on a
  steady circle slip is constant, so the course turns at the same rate as the
  heading. Simulated at r = 250 mm, 5.5 deg/g, 0.1-0.9 g: path radius
  identical with and without slip. Measuring the circle diameter therefore
  shows nothing about slip. It does check speed and gyro calibration
  (diameter = 2 * v / omega_gyro), which is worth doing first since both
  errors would contaminate turn measurements. (If yaw were controlled from
  encoders instead of the gyro, wheel slip would affect yaw and the diameter;
  outside this model.)
- Slip only shows as crabbing: the body points inward of the path tangent by
  the slip angle. Two markers at +/- d along the body trace circles whose
  radii differ by 2 * d * sin(slip) (simulated: 7 mm at 5 deg, d = 40 mm).
  This is unlikely to be measurable accurately enough with a practical camera
  on a moving robot (blur, LED glare, marker height parallax, scale).

### Accelerometer circle test (preferred for k)

Drive constant diameter circles at a range of speeds, logging a two or three
axis accelerometer and the gyro at 1 ms or better.

Principle: on a steady circle at constant speed the acceleration points at the
circle centre, perpendicular to the velocity. Slip rotates the body against
the velocity, so in body axes the acceleration is rotated from the lateral
axis by the slip angle:

    a_x (forward) = a * sin(slip),  a_y (lateral) = a * cos(slip)
    slip = atan(a_x / a_y)

The magnitude of the acceleration does not change with slip; only its
direction in body axes does. Signal: about 0.44 g forward per 5 deg of slip at
5 g.

Simulated (r = 250 mm, k / L = 1 deg/g, sensor point accelerations from
differenced positions): with the sensor at the reference point, the angle
recovers slip to 0.01 deg at 1, 3 and 5 g. A forward offset px adds a constant
error of px / R radians (2.29 deg for px = 10 mm, R = 250 mm), the same at all
speeds.

Error terms and how to remove them. Fit over the speed sweep:

    measured angle = (k / L) * (a / g) + B + C / a

- (k / L): the slope, what is wanted.
- B, constant: forward mounting offset, px / R. Not cancelled by reversing
  direction; absorbed by the fit (or computed from measured px and R).
- C / a: accelerometer forward zero bias, and sensor pitch tilt (gravity into
  a_x). 20 mg gives 0.23 deg at 5 g, 1.1 deg at 1 g. Reduce by subtracting a
  static reading (robot stationary, fan running) before each run.
- Run clockwise and anticlockwise and average. a_y changes sign with
  direction, slip and offset terms do not, so these cancel:
  - sensor yaw misalignment in its mounting (a direct angle error)
  - cross-axis sensitivity (typically 1%, about 0.6 deg)
- Lateral offset py: changes the magnitude of a_y only; second order in the
  angle.
- Floor tilt: g * sin(tilt) rotates with heading around the lap; averages out
  over whole laps.
- Speed variation: du/dt adds directly to a_x (0.05 g ripple = 0.6 deg at
  5 g). Average over whole laps, or subtract du/dt from the encoders.
- Body roll would leak gravity into a_y; negligible for a rigid robot.

Practical notes:
- Slip settles in a few tau (tens of ms), so a slow speed ramp around the
  circle gives the whole sweep in one run. Ramp up then down and average to
  cancel the du/dt of the ramp.
- Speed needed: v = sqrt(a * R). 5 g on a 500 mm diameter circle needs
  3.5 m/s (lap about 0.45 s). A larger circle reduces the px / R error but
  needs more speed for the same acceleration.
- Use a +/- 16 g range for headroom over vibration at 5 g. Averaging over many
  laps at 1 kHz should take noise below the calibration terms; not quantified
  (depends on the sensor).
- The test finds the grip limit: the angle vs acceleration line bends upward
  as the tyres saturate. Beyond it the robot slides out of the circle. This
  matters for turns at 5 g or more.

Gives steady state k / L across the acceleration range, and the grip limit, on
the real surface. Replaces the slope test. Does not test the dynamics (see
the exit tail test below).

Fallback: fit k from one turn type (sweep above). Then use tests 2-5 as the
predictions; they do not depend on that turn type.

### Accelerometer exit tail test (dynamics)

On the exit straight the gyro holds omega = 0, so lateral acceleration comes
only from residual slip. The lag model predicts:

    a_y(t) = (g * L / k_rad) * slip_exit * exp(-t / tau)

starting at about 0.18 g for SS90F sinusoid defaults at 5.5 deg/g. The steady
state model predicts a_y = 0 as soon as omega reaches 0, so the tail shape
tests the model directly.

- Fit an exponential with an offset. tau depends on timing only, not on
  accelerometer scale or bias.
- k / L = g * tau / v. Repeat at several speeds: tau(v) = c * v + d, with
  c = k_rad / (g * L) and d absorbing any fixed delay between gyro and
  accelerometer (internal filters). A straight line through about zero checks
  tau ~ v.
- Mounting offset: the extra terms alpha * px (lateral) and omega^2 * py vanish
  on the straight (omega = alpha = 0), so the tail is immune to sensor
  position. Only axis alignment matters. (During the turn alpha * px can be
  larger than the slip signal: 0.47 g at 154 rad/s/s, px = 30 mm.)
- Use a fixed accelerometer filter setting and sample the gyro and
  accelerometer at the same time.
- Vibration may be comparable to the signal; average several runs aligned on
  the instant omega reaches 0. Higher acceleration gives more signal.
- The heading controller must really hold omega = 0 after the turn.
- With k / L from the circle test, the tail is a zero-parameter prediction:
  tau = k_rad * v / (g * L).

The accelerometer cannot measure fan downforce (steady force); use the scale
method in test 5.

### How to measure (other options)

- The robot's own side sensors on the exit straight: wall distance against
  encoder distance gives the lateral offset and its decay. With gyro heading,
  d(wall distance)/d(distance) gives the crab angle. Practical in a maze;
  measures drift (position) rather than slip directly.
- Optical flow sensor (e.g. a PMW3360 mouse sensor) under the robot: measures
  forward and lateral ground velocity, slip = atan(lateral / forward),
  continuously during real turns. Needs a fixed height above the floor and a
  surface it can track.
- Overhead video: path shape (e.g. the inside swerve question at turn entry),
  but not accurate enough for slip angle.
- Encoders alone cannot see lateral slip; they measure along the heading.
- Log the gyro to confirm heading follows the commanded omega. If it does not,
  controller error is mixed into the slip measurements.

### Suggested minimum set

1. Speed and gyro calibration: circle diameter = 2 * v / omega_gyro
2. Accelerometer circle test, both directions, speed sweep up to the target
   acceleration (k / L, linearity, grip limit)
3. Accelerometer exit tail at two or more speeds (dynamics, tau ~ v, checked
   against tau predicted from step 2)
4. One turn type at 3-4 peak accelerations, end point compared with End dX/dY

The simulator can produce predicted values (exit slip, exit tail a_y(t),
offset at given distances along the exit straight, End dX/dY) for a chosen
test plan, for comparison with measurements.

## 7. Caveats and open questions

- Linear model, no grip limit. Slips of 10-19 deg at 2 g need a fan (L > 1)
  or would saturate on a real mouse. Target turns are 5 g or more, so the
  grip limit found by the circle test may need adding to the model
  (saturation at mu * N).
- Constant speed only. Speed changes between turns would make tau vary along
  the run (superposition with a fixed tau no longer holds exactly).
- Formulas in sections 2-4 use small angle approximations; the simulation
  does not.
- Validation (section 6) is needed before relying on compensation tables.

# Turn profile analysis and recommendation

Compares the turn profiles for fast turns at a fixed peak centripetal
acceleration, with each turn fitted to the same space in the maze. Results
are from the simulator physics (no slip, gyro heading, constant speed).

## Recommendation

Goal: shortest time over a series of turn + straight moves without losing
reliability.

### Summary

- In simulation one ranking holds for all 8 turn types and all peak
  accelerations (1 to 5.5 g): hybrid > sinusoid (share 0.72) > cubic >
  trapezoid > raised cosine. Same footprint, no more peak wheel acceleration
  or combined load than the cubic.
- Use one implementation: the hybrid, with per-turn parameters. Blend 0 is
  exactly the sinusoid, so it covers both.
  - Long turns (SS90F, SS180, SD135, DS135): blend 0.7-0.85, 2.4-3.6%
    quicker than the sinusoid.
  - DD90: blend 0.5, 1.8% quicker.
  - Short turns (SD45, DS45, SS90E): blend 0-0.15, effectively the sinusoid
    (0-0.4% gain).
- Mixing sinusoid and cubic gains nothing: the sinusoid's best share is the
  same in every turn.
- Not proven on the robot. The hybrid holds its limits longest and so has
  the least margin (see Unresolved below). Lowest-risk first step: the
  sinusoid at share 0.72, at the cubic's peaks.

### Profile comparison

Same footprint and limits as the cubic.

| profile | advantages | disadvantages |
|---|---|---|
| Cubic | one parameter, closed form; proven on D5 at 55 m/s/s; empirical slip table exists | 3-6% slower than the hybrid; alpha steps at entry |
| Sinusoid (quarter sine) | 3.0-3.5% quicker than the cubic in every turn, no extra load; one best share (0.72) for all turns and A; cheap to evaluate | two parameters; alpha steps at entry; slip compensation must be recalculated |
| Hybrid | quickest (3.0-6.2% vs cubic); blend 0 = sinusoid | three parameters; holds limits longest, least margin; best blend differs per turn; not run on a robot |
| Trapezoid | simple; sets the wheel limit directly | fits the cubic's limits in 1 of 8 turns (SS90E, 11.6% slower); load peaks at ramp/arc corner; figures approximate |
| Raised cosine | smooth at both ends | 6-16% slower than the cubic; not in the app |

### Margin and contest run tiers

Usual practice is at least three speed runs: about 95% reliable, then
75-80%, then a "hail mary" just over 50% (some owners stop at about 85%).
Profiles should be compared on time at a given run reliability, not at
equal peak load.

Run reliability is the product of the per-turn reliabilities (assuming
independent failures). Per-turn reliability needed:

| run target | 10 turns | 20 turns |
|---|---|---|
| 95% | 99.5% | 99.7% |
| 80% | 97.8% | 98.9% |
| 50% | 93.3% | 96.6% |

Even the hail mary needs every turn above 93%, so all tiers sit below the
knee. D5 SS90F at 60 m/s/s (about 40%) is unusable in any tier; 55 m/s/s
(80-90%) only fits a run with few turns.

Time scales as 1 / sqrt(A), so a profile g quicker at the same A matches the
cubic's time at (1 - g)^2 * A:

| profile | gain at same A | equal to cubic time at |
|---|---|---|
| Hybrid, long turns | 5.6% (SS90F) | 0.89 A |
| Hybrid, short turns | 3.5% | 0.93 A |
| Sinusoid | 3.1% | 0.94 A |

The hybrid stays ahead if its longer time at the limit costs less
reliability than an 11% cut in A. D5's knee is steep (SS90F 55 to 60 m/s/s,
+9%, about 85% to 40% success, from recollection), so a small cut in A
recovers a lot of reliability. This favours the hybrid but is not measured.

Pairing by tier may pay:
- 95% / 85% tier: A well below the knee, exposure hardly matters. Hybrid or
  sinusoid takes the full 3-6%.
- Hail mary: A at the knee, exposure matters most. Profiles with brief peaks
  (cubic, sinusoid) may beat the hybrid. Untested.
- Only turns near their limit matter. A per-tier, per-turn table of A and
  blend extends the firmware's existing acceleration-indexed tables.

### Margin metric

- Direct: measure p(A) on the robot per profile and turn (n runs per A
  step); read A at 95/80/50% and convert to time. No tyre model needed.
- App proxy: peak combined load plus exposure (time or distance above, say,
  90% of peak). Ranks profiles; needs robot data to calibrate.

### Unresolved

1. Slip by profile: exit slip, residual drift and compensation sensitivity,
   especially short turns linked by short straights. Only the cubic has
   empirical compensation (D5).
2. Peak vs sustained load: the cubic peaks briefly at entry, the hybrid
   holds the peak for 20-30 ms. Same peak proven, holding it not.
3. Radius of gyration k: decides whether alpha or centripetal load governs
   the short turns.
4. Constant speed and a single (cubic default) footprint assumed.
5. How reliability falls with exposure vs peak load: unknown; needs robot
   data.

### Next tests

1. SS90F only: cubic at 50 / 55 / 60 m/s/s vs hybrid at about 0.85-0.95 of
   each, 10-20 runs per point. If the hybrid at 0.92 A is as reliable as
   the cubic at A, it wins at every tier.
2. Sinusoid at share 0.72 at the cubic's peaks, as the low-risk step.
3. Add the exposure proxy to the app; compare slip behaviour across
   profiles.
4. Measure k.

The sections below are the supporting evidence.

## Context

- Practical robot: total downforce including gravity about 3-4 g; turns at
  55 m/s/s (5.6 g) centripetal have been achieved.
- Current practice: cubic spiral profile (continuous, one parameter, easy to
  evaluate and adjust; needs more wheel acceleration than some profiles).
- Question: does another profile give shorter turn times without extra load?

## Why the app defaults are not a fair comparison

At default parameters each profile has a different size (radius, lambda,
length), and a larger turn allows more speed for the same peak acceleration.
Comparing defaults at the same peak acceleration therefore mixes shape with
size. For example, at default SS90F the trapezoid appeared least stressful,
but it was running about 7% slower and taking longer.

## Method

For each turn type:
- The cubic default (offset and length) defines the start pose, the exit line
  (through its exit point, along its exit heading) and a reference finish
  point (its exit point).
- Each segmented profile is run at lambda share q, where
  share = lambda / max lambda for the radius. Max lambda is the summary
  "Lambda Max": the two transitions take the whole turn, no constant arc.
  Share 1 = transitions only; share 0 = plain arc.
- For each q the radius is scaled (bisection) until the exit lies on the
  cubic's exit line.
- Speed is set for peak centripetal acceleration A = 55000 mm/s/s
  (`speed_for_acceleration()`).
- Time = turn time + time at turn speed for the along-line distance between
  the profile's exit point and the reference point (subtracted if it exits
  beyond it).
- Wheel acceleration = r_w * alpha, r_w = 37 mm (wheel offset from centre),
  alpha from successive omega records.
- Combined load = peak over the turn of hypot(centripetal, r_w * alpha), in g.
- q scanned 0 to 1 in steps of 0.02. Best q: least time with wheel
  acceleration and combined load no higher than the cubic's.
- Time step 0.1 ms.

## Results: SS90F share sweep

Footprint 150 mm (start 150 mm before the corner, exit on the exit line).

| profile | share | min radius | speed | time | vs plain arc | wheel accel | combined |
|---|---|---|---|---|---|---|---|
| plain arc | 0 | 150 mm | 2872 | 82.1 ms | 0 | step | - |
| Sinusoid | 0.50 | 121 mm | 2584 | 94.4 ms | +15.0% | 4.35 g | 5.61 g |
| Sinusoid | 0.75 | 110 mm | 2464 | 99.9 ms | +21.7% | 3.19 g | 5.61 g |
| Sinusoid | 1.00 | 101 mm | 2356 | 105.1 ms | +28.0% | 2.62 g | 5.61 g |
| Trapezoid | 0.50 | 106 mm | 2413 | 101.2 ms | +23.3% | 2.50 g | 6.15 g |
| Trapezoid | 1.00 | 80 mm | 2100 | 117.2 ms | +42.8% | 2.75 g | 6.24 g |
| Cubic | - | 105 mm | 2405 | 103.1 ms | +25.6% | 3.35 g | 5.61 g |

Speeds in mm/s. Shorter transitions (lower share) allow a larger minimum
radius and so more speed, at the cost of faster changes of omega (higher
wheel acceleration).

## Results: best share per turn, within the cubic's limits

A = 55000 mm/s/s. Same start, exit line and finish point as the cubic
default. Speeds in mm/s, times in ms.

| turn | cubic speed | cubic time | sinusoid 0.72 speed | sinusoid time | time saved | wheel accel g, cubic / sinusoid | combined g, cubic / sinusoid |
|---|---|---|---|---|---|---|---|
| SS90F | 2406 | 103.1 | 2479 | 100.0 | 3.1% | 3.34 / 3.28 | 5.61 / 5.61 |
| SS180 | 2078 | 178.2 | 2132 | 172.0 | 3.5% | 2.24 / 2.22 | 5.61 / 5.61 |
| SD45 | 2501 | 53.6 | 2578 | 52.0 | 3.0% | 6.18 / 6.07 | 6.18 / 6.07 |
| SD135 | 2035 | 130.8 | 2094 | 126.6 | 3.2% | 3.12 / 3.07 | 5.61 / 5.61 |
| DS45 | 2593 | 55.6 | 2673 | 53.9 | 3.0% | 5.75 / 5.65 | 5.75 / 5.65 |
| DS135 | 2000 | 128.6 | 2058 | 124.5 | 3.2% | 3.23 / 3.18 | 5.61 / 5.61 |
| DD90 | 2010 | 86.1 | 2070 | 83.5 | 3.1% | 4.79 / 4.71 | 5.61 / 5.61 |
| SS90E | 1701 | 73.0 | 1753 | 70.7 | 3.1% | 6.68 / 6.57 | 6.68 / 6.57 |

Other profiles:
- Trapezoid: no share fits within the cubic's limits, except SS90E (share
  0.84, 11.6% slower). Its angular acceleration steps add to the tyre load at
  the ramp corners.

Sinusoid wheel acceleration either side of the chosen share:

| share | sinusoid wheel accel vs cubic |
|---|---|
| 0.70 | +0.2% to +1.4% |
| 0.72 | -1.0% to -1.8% |

The share that exactly matches the cubic's wheel acceleration is about
0.70-0.71, saving a further 0.3-0.6% of turn time. The slightly lower wheel
acceleration at 0.72 is a result of the 0.02 scan step, not an extra
advantage.

## Findings

- The sinusoid at share about 0.72 is 3.0-3.5% quicker than the cubic in
  every turn type, 2.5-3% faster in speed, with no more wheel acceleration and
  no more combined tyre load.
- The extra speed also applies to the linking straights wherever they run at
  turn speed, so the gain over a run is more than 3% per turn.
- The best share is the same for every turn: the wheel acceleration vs time
  trade depends on the profile shape, not on the turn angle.
- For SS90F the result (R = 112) is the app's sinusoid default (R = 112,
  lambda 100, Lambda Max 138, share about 0.72).
- Larger gains need more wheel acceleration: sinusoid share 0.5 is about 8%
  quicker than the cubic in SS90F but needs about 30% more wheel acceleration
  (4.35 g vs 3.35 g).
- The cubic's remaining advantages are one parameter and simple closed form
  expressions. Evaluating a sine on the robot is cheap.

## Variation with peak centripetal acceleration (1 g to 5.5 g)

Same method, A from 1 g to 5.5 g in steps of 0.5 g. SS90F rows; the last
column covers all eight turn types. Speeds in mm/s, times in ms.

| peak | cubic speed | cubic time | sinusoid speed | sinusoid time | wheel accel g, cubic / sinusoid | all turns: time saved |
|---|---|---|---|---|---|---|
| 1.0 g | 1016 | 244.1 | 1047 | 236.6 | 0.60 / 0.59 | 3.08-3.48% |
| 1.5 g | 1244 | 199.3 | 1282 | 193.2 | 0.89 / 0.88 | 3.03-3.48% |
| 2.0 g | 1437 | 172.6 | 1480 | 167.3 | 1.19 / 1.17 | 3.03-3.48% |
| 2.5 g | 1606 | 154.4 | 1655 | 149.7 | 1.49 / 1.46 | 3.03-3.48% |
| 3.0 g | 1760 | 141.0 | 1813 | 136.6 | 1.79 / 1.76 | 3.03-3.48% |
| 3.5 g | 1901 | 130.5 | 1958 | 126.5 | 2.09 / 2.05 | 3.03-3.48% |
| 4.0 g | 2032 | 122.1 | 2094 | 118.3 | 2.39 / 2.34 | 3.03-3.48% |
| 4.5 g | 2155 | 115.1 | 2221 | 111.6 | 2.68 / 2.64 | 3.03-3.48% |
| 5.0 g | 2272 | 109.2 | 2341 | 105.9 | 2.98 / 2.93 | 3.03-3.48% |
| 5.5 g | 2383 | 104.1 | 2455 | 100.9 | 3.28 / 3.22 | 3.03-3.48% |

At every level:
- sinusoid best share 0.72 for all turns (one turn at 1 g came out 0.70,
  from simulation step effects at low speed)
- time saved 3.0-3.5%, speed 2.6-3.1% higher (3.5% for one turn at 1 g)
- trapezoid fits within the cubic's limits in 1 of 8 turns
- in the short turns (45 deg, SS90E) the transition load stays at a fixed
  ratio to the centripetal peak (about 1.03-1.2)

Reason: with a fixed footprint the path shape does not depend on speed.
Speed scales as sqrt(A), time as 1 / sqrt(A), and wheel acceleration as A
(alpha ~ v^2). All ratios between profiles, and so the best share, are
independent of A. The profile choice is therefore the same whatever sets the
limit (grip or motor acceleration).

## Short turns: tyre load peaks in the transitions

In SD45, DS45 and SS90E the combined load peaks at the start and end of the
turn, where centripetal acceleration is near zero and r_w * alpha is largest:
5.7-6.7 g, above the 5.6 g centripetal peak. There, angular acceleration and
not centripetal acceleration sets the grip requirement.

This assumes r_w * alpha acts on the tyres as a full acceleration, which holds
only if the yaw inertia is as if all the mass were at the wheels. The wheel
force for yaw is I * alpha / (2 * r_w) per wheel, I = m * k^2 (k = radius of
gyration), so the equivalent acceleration is (k / r_w)^2 * r_w * alpha. For
k = 25-30 mm that is about 45-65% of the figures above, and the centripetal
peak governs again. The wheel acceleration values themselves (rate of change
of wheel speed) are exact. Measuring or estimating k would settle which load
governs the short turns.

## Other profiles and the fastest transition shape

### Angular acceleration at the ends

The app's "Sinusoid" is a quarter sine, omega = W * sin((pi/2) * x / lambda),
not a raised cosine. It and the cubic both have a nonzero slope
of omega at x = 0, so angular acceleration steps at turn entry (and exit). For
the cubic, omega ~ s * (L - s) has slope L at s = 0. They are smooth only into
the arc (or, for the cubic, at mid turn). Practical cubic runs show that steps
of this size are tolerated.

### Raised cosine

omega = W * (1 - cos(pi * x / lambda)) / 2: zero angular acceleration at both
ends. Same method (best share within the cubic's limits), A = 55000 mm/s/s:

| turn | sinusoid vs cubic | raised cosine vs cubic |
|---|---|---|
| SS90F | -3.1% | +5.7% |
| SS180 | -3.5% | +7.5% |
| SD45 | -3.0% | +12.4% |
| SD135 | -3.2% | +6.0% |
| DS45 | -3.0% | +15.9% |
| DS135 | -3.2% | +6.0% |
| DD90 | -3.1% | +12.4% |
| SS90E | -3.1% | +10.5% |

6-16% slower than the cubic. Each transition turns W * lambda / (2v), against
(2/pi) * W * lambda / v for the quarter sine, and its peak angular
acceleration is 1.57 times the trapezoid's at the same lambda. Smoothness
costs speed.

A jerk-limited trapezoid was not simulated. Expected between the trapezoid
and the sinusoid: rounding the entry corner costs time, and the trapezoid's
real problem is the corner at the end of the ramp (full centripetal and full
alpha together), which would also need rounding, at which point it
approaches a sine shape.

### Fastest shape (reasoning, not a proof)

The limits all scale with v^2, so the fastest transition rides whichever
limit binds:
- separate limits on centripetal and wheel acceleration: linear omega ramp at
  the wheel limit (trapezoid / clothoid)
- combined load limit only, hypot(centripetal, r_w * alpha) <= C: curvature
  = c * sin(s / l) with l = r_w, i.e. the quarter sine with
  lambda = pi * r_w / 2 (58 mm). The combined load is constant at the limit
  through the whole transition. With real yaw inertia l = k^2 / r_w
  (k = radius of gyration).
- both (wheel limit W below the combined limit C): hybrid. Linear ramp at W
  until the combined limit is reached, then the sine onto the arc.

Other known shapes (minimum jerk quintic, Bezier or spline blends, logistic)
are smooth approximations to these and should not beat them for the same
limits.

### Ideal hybrid (simulated directly from the limits)

A = 55000 mm/s/s, same footprint and finish point as the cubic. W and C are
the cubic's peak wheel acceleration and combined load for each turn. Times
in ms.

| turn | cubic | sinusoid 0.72 | hybrid, cubic W and C | vs cubic | hybrid, W limited only by C | vs cubic | wheel accel needed |
|---|---|---|---|---|---|---|---|
| SS90F | 103.1 | 100.0 | 97.3 | -5.6% | 91.8 | -11.0% | 5.61 g |
| SS180 | 178.1 | 172.0 | 166.8 | -6.3% | 145.4 | -18.4% | 5.61 g |
| SD45 | 53.6 | 52.0 | 51.4 | -4.1% | 51.4 | -4.1% | 6.18 g |
| SD135 | 130.8 | 126.6 | 123.1 | -5.9% | 114.2 | -12.7% | 5.61 g |
| DS45 | 55.6 | 53.9 | 53.6 | -3.6% | 53.6 | -3.6% | 5.75 g |
| DS135 | 128.6 | 124.5 | 121.1 | -5.8% | 112.8 | -12.3% | 5.61 g |
| DD90 | 86.1 | 83.5 | 81.9 | -4.9% | 80.7 | -6.3% | 5.61 g |
| SS90E | 72.9 | 70.7 | 69.6 | -4.5% | 69.6 | -4.5% | 6.68 g |

- Within the cubic's limits the hybrid is 0.6-3.0% quicker than the sinusoid
  and 3.6-6.3% quicker than the cubic. Angular acceleration is continuous at
  the linear-to-sine join (slopes match where the combined limit is reached)
  and at the arc join where C = A; it still steps at entry.
- Limited by combined load only, the hybrid becomes the pure sine with
  lambda = pi * r_w / 2 = 58 mm: 8-15% quicker than the sinusoid in SS90F,
  SD135, DS135 and SS180 (3.4% in DD90), needing about 5.6 g wheel
  acceleration (from 2.2-3.3 g).
  In the short turns the cubic already uses its full combined load in the
  transitions, so nothing more is available.
- The large gain needs more wheel acceleration from the motors, not a better
  shape.

### Hybrid profile in the app

Added as profile "Hybrid" (`profile_hybrid.cpp`). Parameters: radius, lambda
and blend f (fraction of the arc omega where the linear part hands over to the
sine; 0 = sinusoid exactly, near 1 approaches the trapezoid). The sine length
is set from f so the slopes match at the join. The sine always ends at
pi/2 (smooth into the arc), so the short-turn optimum with C > A (sine ending
early with a step into the arc) is approximated, not reproduced.

Defaults fitted with the same method, scanning blend (step 0.05) and share
(step 0.02) within the cubic's limits, same footprint as the cubic. A =
55000 mm/s/s:

| turn | cubic speed | cubic time | hybrid speed | hybrid time | time vs cubic | speed vs cubic | radius | lambda | blend |
|---|---|---|---|---|---|---|---|---|---|
| SS90F | 2406 | 103.1 | 2539 | 97.4 | -5.6% | +5.5% | 117 | 73 | 0.80 |
| SS180 | 2078 | 178.2 | 2170 | 167.1 | -6.2% | +4.4% | 86 | 105 | 0.85 |
| SD45 | 2501 | 53.6 | 2588 | 51.8 | -3.4% | +3.5% | 122 | 52 | 0.15 |
| SD135 | 2035 | 130.8 | 2133 | 123.7 | -5.4% | +4.8% | 83 | 85 | 0.70 |
| DS45 | 2593 | 55.6 | 2673 | 53.9 | -3.0% | +3.1% | 130 | 57 | 0.00 |
| DS135 | 2000 | 128.6 | 2096 | 121.6 | -5.4% | +4.8% | 80 | 82 | 0.70 |
| DD90 | 2010 | 86.1 | 2103 | 82.0 | -4.8% | +4.7% | 80 | 60 | 0.50 |
| SS90E | 1701 | 73.0 | 1759 | 70.4 | -3.5% | +3.4% | 56 | 48 | 0.15 |

- Within 0.5% of the ideal hybrid in the long turns; the short turns lose a
  little (e.g. SS90E 70.4 vs 69.6 ms) from the smooth arc join.
- Best blend follows f = sqrt(1 - (W / A)^2) where the wheel limit binds
  (SS90F: W / A = 0.595, f = 0.80).
- Defaults in the app use speeds for 9800 mm/s/s (speed = sqrt(9800 * R)),
  like the other profiles.
- With blend 0 the hybrid reproduces the sinusoid to 1e-5 mm.

### Does the cubic's record imply the hybrid is achievable?

The hybrid within the cubic's limits needs no higher peak wheel acceleration
or combined load than the cubic in the same turn, so runs at 55 m/s/s with the
cubic show those peaks are reached, with two provisos:
- Only for the turn types actually run at 55 m/s/s. The cubic's wheel
  acceleration varies by turn: about 2.2 g (SS180), 3.3 g (SS90F),
  6.2-6.7 g (45 deg turns, SS90E).
- Peak vs sustained. The cubic demands its peak wheel acceleration only at
  entry, as a step that the controller and motors probably smooth in
  practice. The hybrid holds that peak through the whole linear part (about
  20-30 ms), and the combined load at the limit through the sine part. The
  same peak is proven; holding it is not, and the hybrid leaves less margin at
  the grip limit.

With those provisos: expect about 3-6% shorter turn times and 3-5.5% higher
turn speeds, if the model holds.

## Caveats

- No slip in this comparison. Slip does not change turn times (gyro heading)
  but changes exit positions, differently for each profile; tau grows with
  speed, so drift rises slightly (about 2.5%) with the sinusoid's higher
  speed. Compensation values (see residual-slip-compensation.md) would need
  recalculating for the sinusoid.
- Trapezoid ramps overshoot the arc omega (todo 7.2), and its exits do not
  quite fit (SS90F exit x = 144 mm vs 150 mm), so its figures are approximate.
- Combined load uses the yaw inertia assumption above.
- Assumes wheel acceleration is a binding limit. If the motors and the tyres'
  longitudinal grip have spare capacity, shares below 0.72 give larger gains.
- Fixed footprint taken from the app's cubic defaults; a different footprint
  (offsets) scales the speeds; that it leaves the ranking unchanged is
  expected from the shape argument above but has not been checked.
- Constant speed through each turn.

## Reproducing

The comparison was a scratch program built against the physics sources
(`track.cpp`, `turn_profile.cpp`, `segmented_profile.cpp`, `profile_*.cpp`),
not part of the repository. The method section above is enough to rewrite
it. Key settings: A = 55000 mm/s/s, r_w = 37 mm, dt = 0.1 ms, bisection on
radius between 15 and 400 mm (40 iterations), share step 0.02. Hybrid
defaults: blend step 0.05, share 0.2-1.0. The ideal hybrid was simulated
directly with RobotState (ramp built from W, C and A at each trial speed,
bisection on speed to fit the exit line).

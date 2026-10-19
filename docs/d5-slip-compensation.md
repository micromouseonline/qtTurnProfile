# D5 slip compensation vs app slip model

Compares the empirical `inAdjustCubic` table from robot D5 (Feb 2018) with
the app's linear lag slip model (see `residual-slip-compensation.md`,
`SlipParams` in `robotstate.h`). Question: does the model reproduce the
observed corrections, and is a single slip coefficient consistent across
turn types?

## 1. The D5 table

- Values: mm of early turn start (entry straight shortened) so the exit
  lies centred on its path. Lateral exit error only; forward error after
  the turn is a separate process.
- Columns: peak centripetal acceleration 25, 30, ... 60 m/s/s. Firmware
  interpolates between columns. Turns are defined by acceleration; speed is
  computed at turn start.
- Zero entries may mean the turn is not used at that acceleration (e.g.
  unstable), not "no correction".
- L/R asymmetry is unexplained; the L/R average is used here.
- SS180 entries correct forward (along-track) error, not lateral, and could
  be zero. SS180 is excluded. SS90S and SS90E rows are all zero and excluded.

```
constexpr int16_t inAdjustCubic[][LAST_OFFSET_INDEX + 1] = {
  //  0    1    2    3    4    5    6    7
  // 25   30   35   40   45   50   55   60
  {   0,   0,   0,   0,   0,   0,   0,   0 }, //SS90SR
  {   0,   0,   0,   0,   0,   0,   0,   0 }, //SS90SL
  {   0,   0,   5,   7,  10,  15,  30,  30 }, //SS90FR 24/02/18
  {   0,   5,  10,  15,  20,  20,  35,  35 }, //SS90FL 24/02/18
  {   0,   0,   5,  10,  15,  20,  25,  30 }, //SS180R 24/02/18
  {   0,   0,   5,  10,  15,  20,  25,  30 }, //SS180L 24/02/18
  {   5,  12,  15,  17,  18,  20,  20,  15 }, //SD45R   25/02/18
  {   0,   5,  10,  12,  18,  20,  15,  15 }, //SD45L   25/02/18
  {   0,   0,   0,   5,   5,   5,  10,   5 }, //SD135R   26/02/18
  {   0,   0,   5,   7,   8,  12,  15,   5 }, //SD135L   26/02/18
  {   3,  10,  15,  18,  22,  25,   0,   0 }, //DS45R   26/02/18
  {   0,   3,   7,   9,  12,  15,   0,   0 }, //DS45L   26/02/18
  {   0,   5,   5,   5,   5,   5,  10,   0 }, //DS135R  26/02/18
  {  10,  10,  10,  15,  15,  15,  20,   0 }, //DS135L  26/02/18
  {   0,   1,   2,   4,   7,  10,   0,   0 }, //DD90R
  {   0,   5,   7,  10,  11,  12,   0,   0 }, //DD90L
  {   0,   0,   0,   0,   0,   0,   0,   0 }, //SS90ER 24/02/18
  {   0,   0,   0,   0,   0,   0,   0,   0 }, //SS90EL 24/02/18
};
```

L/R averages used (mm; "-" = unused, excluded):

| Turn | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 |
|---|---|---|---|---|---|---|---|---|
| SS90F | 0 | 2.5 | 7.5 | 11 | 15 | 17.5 | 32.5 | 32.5 |
| SD45 | 2.5 | 8.5 | 12.5 | 14.5 | 18 | 20 | 17.5 | 15 |
| SD135 | 0 | 0 | 2.5 | 6 | 6.5 | 8.5 | 12.5 | 5 |
| DS45 | 1.5 | 6.5 | 11 | 13.5 | 17 | 20 | - | - |
| DS135 | 5 | 7.5 | 7.5 | 10 | 10 | 10 | 15 | - |
| DD90 | 0 | 3 | 4.5 | 7 | 9 | 11 | - | - |

## 2. Method

- Cubic spiral profile, app default lengths: SS90F 246, SD45 134,
  SD135 266, DS45 142, DS135 257, DD90 173 mm.
- Load factor 1.0 (no fan). Only k / load matters in the model, so a fitted
  k at load 1 is really k / load.
- Speed from `speed_for_acceleration()` for each column acceleration.
- 1500 mm leadout so post-turn drift has finished.
- Lateral error: perpendicular distance of the slipped end point from the
  no-slip exit line. Predicted inadjust: the start shift back along the
  entry heading that zeroes it (solved from two runs, 0 and 10 mm back;
  the error is linear in the shift). Equivalent to lateral error /
  sin(turn angle).
- Fits: best k through the origin (grid 0.1 to 6.0 deg/g, least squares),
  and a straight line with intercept fitted to the empirical data, its
  slope compared with the model slope at k = 1.

## 3. Model prediction

Predicted inadjust is almost exactly proportional to acceleration and k,
through the origin, and nearly independent of turn type and length:
about 0.14 to 0.22 mm per m/s/s per deg/g.

Example, SS90F (mm):

| k deg/g | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 |
|---|---|---|---|---|---|---|---|---|
| 1 | 4.7 | 5.7 | 6.7 | 7.6 | 8.6 | 9.6 | 10.6 | 11.6 |
| 2 | 9.6 | 11.6 | 13.6 | 15.7 | 17.7 | 19.8 | 21.9 | 24.0 |
| 3 | 14.7 | 17.7 | 20.9 | 24.0 | 27.2 | 30.5 | 33.8 | 37.1 |
| 5 | 25.1 | 30.5 | 36.0 | 41.5 | 47.1 | 52.8 | 58.6 | 64.4 |

Model slope at k = 1 (mm per m/s/s): SS90F 0.197, SD45 0.207,
SD135 0.154, DS45 0.219, DS135 0.148, DD90 0.138.

## 4. Results

| Turn | Emp slope mm per m/s/s | Zero crossing m/s/s | R2 | Slope-matched k deg/g | Best k through origin | rms at best k mm |
|---|---|---|---|---|---|---|
| SS90F | 0.98 | 27 | 0.94 | 5.0 | 2.0 | 7.3 |
| DS45 | 0.72 | 22 | 0.99 | 3.3 | 1.5 | 3.5 |
| DD90 | 0.43 | 24 | 0.99 | 3.1 | 1.2 | 2.3 |
| SD45 | 0.38 | 7 | 0.65 | 1.8 | 1.6 | 3.2 |
| SD135 | 0.28 | 24 | 0.63 | 1.8 | 0.9 | 3.0 |
| DS135 | 0.27 | 5 | 0.85 | 1.8 | 1.6 | 1.1 |

## 5. Findings

Agreement (qualitative):
- Sign correct: slip requires an early start.
- Correction rises roughly linearly with acceleration.
- Implied k of 1 to 5 deg/g is physically plausible.

Disagreement:
1. Threshold. 4 of 6 turns need no correction up to about 22 to 27 m/s/s,
   then rise steeply. The model has no threshold. Possible cause: base turn
   offsets tuned at low acceleration already absorb slip there. If so the
   slopes, not absolute values, are the valid comparison.
2. Inconsistent k. Slope-matched k ranges 1.8 to 5.0 deg/g (factor ~3).
   The model predicts SS90F and SD45 slopes about equal; observed SS90F is
   2.6x SD45. The model predicts SD45 and DS45 about equal; observed DS45 is
   1.9x SD45. SD135 and DS135 agree with each other.
3. SD45 and SD135 drop at 60 m/s/s (15 and 5 mm). Slip cannot produce
   this; likely noise or an unstable turn.

Caveats:
- Table is hand tuned in 5 mm steps and includes effects beyond lateral
  slip (controller lag, longitudinal wheel slip).
- App default cubic lengths may differ from D5's.
- At load 1, 55 m/s/s implies a tyre friction coefficient above 5. If D5
  ran a fan, real k = fitted k * load.

## 6. Refit with base turn conditions

Sections 3 to 5 assume the table is the whole correction. It is not:

- Base turns (offsets) were tuned at 1 g (9.81 m/s/s) turn acceleration,
  no fan (load 1). They already absorb the slip at that condition.
- All table entries were run with a fixed extra downforce, not measured,
  probably about load 3.

So the table is a correction relative to the base:

    table(a) = P(k, L, a) - P(k, 1, 1 g)

P = predicted early start (section 2 method), k = slip gradient deg/g,
L = table load factor. Since P is close to proportional to k * a / L,
the table should cross zero at about a = L * 9.81 m/s/s (29 at L = 3),
whatever k is, and be negative (start late) below that. This explains the
threshold in section 5.

Entries were only ever tuned down to 0 (no late starts), so predictions
are floored at 0 before comparison: residual = max(0, predicted) - table.

Fits (k grid 0.5 to 30 deg/g, L grid 1.5 to 5.0, zero floor):

| Turn | Free fit k | Free fit L | rms mm | k at L = 3 | rms at L = 3 mm |
|---|---|---|---|---|---|
| SS90F | 13.25 | 2.9 | 2.6 | 14.25 | 2.6 |
| SD45 | 3.5 | 1.5 (grid edge) | 3.4 | 11.0 | 6.2 |
| SD135 | 4.0 | 2.4 | 2.5 | 6.0 | 2.6 |
| DS45 | 6.5 | 2.2 | 0.7 | 13.25 | 3.4 |
| DS135 | 3.25 | 1.5 (grid edge) | 1.6 | 9.75 | 4.1 |
| DD90 | 7.0 | 2.5 | 0.3 | 10.75 | 1.3 |

Joint fit, all turns: k 4.75, L 1.9, rms 4.3 mm. Joint at L = 3: k 10.5,
rms 5.2 mm. The zero floor changes rms by at most 0.8 mm and k by at most
1 deg/g compared with an unfloored fit.

Section 6 uses the D5 cubic lengths (`cubic_params`, dated Mar 2019):
SS90F 250, SD45 130, SD135 265, DS45 155, DS135 260, DD90 185 mm. Results
differ from the app default lengths by at most 0.75 deg/g in k and 0.1 mm
rms; the model is insensitive to length at fixed acceleration. The table is
dated Feb 2018, so the lengths in use then are not certain.

Example, SS90F at L = 3, k = 14.25 (mm):

| a m/s/s | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 |
|---|---|---|---|---|---|---|---|---|
| table avg | 0 | 2.5 | 7.5 | 11 | 15 | 17.5 | 32.5 | 32.5 |
| predicted | -4.6 | 0.6 | 5.9 | 11.2 | 16.6 | 22.1 | 27.6 | 33.1 |
| floored | 0 | 0.6 | 5.9 | 11.2 | 16.6 | 22.1 | 27.6 | 33.1 |

Findings:
1. The baseline explains the threshold. Four turns (SS90F, SD135, DS45,
   DD90) fit well, and their free-fit L of 2.2 to 2.9 agrees with the
   estimated load of about 3, found independently of k.
2. SD45 and DS135 have no threshold (non-zero at 25 m/s/s). Their fits run
   to the lower L limit. These rows do not fit the model.
3. k is still not consistent. For the four good turns k / L is 4.6 (SS90F),
   2.8 (DD90), 3.0 (DS45), 1.7 (SD135): a spread of about 2.7. Absolute k
   at L about 3 is roughly 6 to 14 deg/g.
4. At 25 m/s/s the model predicts a small negative (late start)
   correction, -1.4 to -4.8 mm. The table holds 0 there because entries
   were only tuned down to 0; the zero floor accounts for this.

## 7. Turn angle pattern

Observation: the poorer fits seem to be the 45 and 135 degree turns, the
better ones multiples of 90. Checked against the zero floor fits:

| Turn | Angle | Free fit rms mm | rms at L = 3 mm |
|---|---|---|---|
| DD90 | 90 | 0.3 | 1.3 |
| DS45 | 45 | 0.7 | 3.4 |
| DS135 | 135 | 1.6 | 4.1 |
| SD135 | 135 | 2.5 | 2.6 |
| SS90F | 90 | 2.6 | 2.6 |
| SD45 | 45 | 3.4 | 6.2 |

- Free fit: pattern does not hold. DS45 is second best, SS90F fifth.
- L fixed at 3 (more realistic): pattern partly holds. DD90 best, SD45
  worst, but SD135 ties SS90F.
- The clearer split is SD45 and DS135, the two rows with no threshold
  (non-zero at 25 m/s/s). One is orthogonal entry, one diagonal entry, so
  entry type does not explain it either.

Possible causes, not verified:
1. Tuning coupling. For 45 / 135 turns an early start d moves the exit both
   laterally (d sin(angle)) and along the exit path (d cos(angle)). For 90
   turns these are independent. If along-track error influenced the
   judgement of "centred" during tuning, 45 / 135 rows would be biased.
2. Reference quality. Diagonals have no parallel walls: SD exits have no
   good lateral reference, DS entries a less certain start position. Either
   error is absorbed into the table.
3. Missing physics. The model assumes exact heading and no grip limit. A
   heading error (under-rotation) or grip saturation would act differently
   with turn angle. SD45 flattening then dropping at 55 to 60 m/s/s looks
   more like saturation or measurement than linear slip.

Proposed simulator tests:
- add a small exit heading error and see how the predicted correction
  scales with turn angle;
- add a grip limit (saturation at mu * N) to the slip model.

## 8. High acceleration behaviour

### 8.1 Fan droop check

Fan PWM is not compensated for battery voltage, so downforce probably
falls at high speed (drive current, battery sag). Lower L at high a
would make the table rise above the fixed-L model there.

Test: fixed L, k fitted on columns 25 to 45 only, residual
(table - predicted, floored) at 50 to 60. At fixed L the model is close to
linear in a, so upward curvature must come from the data.

Residuals, L = 2 (mm; L = 3 shows the same pattern):

| Turn | 25 | 30 | 35 | 40 | 45 | 50 | 55 | 60 |
|---|---|---|---|---|---|---|---|---|
| SS90F | -2.7 | -2.8 | -0.4 | 0.4 | 1.8 | 1.6 | 13.8 | 11.1 |
| SD135 | -1.2 | -2.4 | -1.0 | 1.3 | 0.5 | 1.3 | 4.0 | -4.8 |
| DD90 | -1.7 | -0.3 | -0.4 | 0.4 | 0.8 | 1.1 | - | - |
| DS45 | -2.1 | -0.4 | 0.8 | -0.1 | 0.0 | -0.3 | - | - |
| DS135 | 2.6 | 2.9 | 0.5 | 0.6 | -1.9 | -4.5 | -2.1 | - |
| SD45 | -1.5 | 0.8 | 1.1 | -0.6 | -0.9 | -2.6 | -8.9 | -15.1 |

- Only SS90F is clearly above the model, and as a step (17.5 to 32.5 mm,
  50 to 55 m/s/s), not the gradual rise droop would give.
- SD45 and DS135 fall below the model (opposite of droop).
- DD90 and DS45 are not used above 50 and show no excess up to 50.
- Resolution: hand tuned 5 mm steps, L/R averaged; differences under
  about 3 to 5 mm are not meaningful.

Result: the table neither supports nor rules out droop. Battery voltage
or fan current logged during high speed turns would settle it.

### 8.2 SS90F at 55 and 60 m/s/s: grip limit

- Only SS90F and SS180 are set up for the top speeds. Entries for other
  turns at 55 to 60 may not have been properly tuned.
- Success rates (recollection, not measured): 55 m/s/s about 80 to 90%,
  60 m/s/s about 40%. Linear slip is repeatable; a sharp loss of
  reliability means the turn is at or past the grip limit.
- A real tyre's slip angle rises faster than linearly near the limit, so
  the needed correction jumps over a narrow range of a: consistent with
  the step between 50 and 55.
- The flat 55 / 60 values (R 30 / 30, L 35 / 35) do not argue against
  this: at 40% success there is no single correct value.
- Rough grip estimate: knee at 55 to 60 m/s/s gives mu * L about 5.6 to
  6.1; at L 2.5 to 3, mu about 1.9 to 2.4. Includes any droop.
- Caveat: failures at 60 could also come from wheelspin or controller
  saturation, not only lateral grip.
- SS90F 55 and 60 should be excluded from linear slip fits. Without them
  SS90F residuals stay within about 3 mm up to 50 m/s/s (L = 2 fit).
- SS180 cannot confirm or rule out a knee: tested only at 10 m/s/s
  steps, filled in linearly, and not tuned closely because it mainly
  affects forward error.

## 9. Fits over the linear range, 25 to 50 m/s/s

Columns above 50 m/s/s are excluded: SS90F is grip limited there, and the
other turns are not set up for those speeds (entries may be untuned).
Zero floor as in section 6.

### 9.1 Each turn separately (k and L free)

| Turn | k deg/g | L | k / L | rms mm | R2 |
|---|---|---|---|---|---|
| DD90 | 7.0 | 2.5 | 2.8 | 0.3 | 0.99 |
| SS90F | 9.0 | 2.6 | 3.5 | 0.6 | 0.99 |
| SD135 | 6.75 | 2.9 | 2.3 | 0.6 | 0.96 |
| DS45 | 6.5 | 2.2 | 3.0 | 0.7 | 0.99 |
| SD45 | 6.25 | 1.9 | 3.3 | 1.1 | 0.96 |
| DS135 | 3.25 | 1.5 (grid edge) | 2.2 | 1.7 | 0.18 |

- Five turns fit within about 1 mm rms, well under the 5 mm tuning step.
- k / L spans 2.3 to 3.5 (1.5x), against 2.7x when 55 to 60 m/s/s was
  included. Most of the earlier spread came from the top columns.
- L 1.9 to 2.9 across turns, consistent with one real load.
- DS135 is not slip shaped: R flat at 5 mm, L flat at 10 to 15 mm. Looks
  like a constant offset or tuning artefact.

### 9.2 One k and L for five turns (DS135 excluded)

Best fit: k 6.25 deg/g, L 2.2, k / L 2.8, rms 2.7 mm.

| Turn | rms mm | Residual pattern |
|---|---|---|
| DS45 | 1.0 | fits |
| DD90 | 1.3 | fits |
| SS90F | 1.4 | fits |
| SD45 | 3.7 | table about 4 mm above model |
| SD135 | 4.2 | table 3 to 6 mm below model |

Three turns within half a tuning step, SD45 and SD135 within about one
step, in opposite directions.

### 9.3 Caveats on the empirical data

- Hand tuned in 5 mm steps, L/R averaged, with unexplained L/R
  differences up to 10 mm.
- Load not measured; battery droop not compensated.
- SD135 may behave differently because it approaches the SS180 case: as
  the turn angle grows past 90, more of the slip drift falls along the
  exit path and less across it, and an early start moves the exit both
  across (d sin(angle)) and along (d cos(angle)) the exit path. 90 degree
  turns are the cleanest case: an early start maps one to one onto the
  lateral exit error, with no along-track coupling.
- Robot layout not modelled. D5 has two drive wheels per side that do not
  carry equal shares of the downforce, and the fan may change that split.
  The model treats the robot as one point with a single slip angle; uneven
  axle loads give different slip at front and rear wheels and an effective
  k that can change with fan load. Not predicted, let alone accounted for.

## 10. Conclusions

In decreasing order of likelihood:

1. The table is mainly a slip-type lateral correction. Sign is right, and
   over 25 to 50 m/s/s five turns follow the model closely (rms 0.3 to
   1.1 mm, R2 0.96 to 0.99).
2. The near-zero region up to about 25 m/s/s comes from the base turns
   being tuned at 1 g with no fan, while the table was run with the fan.
   The model puts the zero crossing at about L * 9.81 m/s/s; fitted L of
   about 2 to 3 is consistent with the estimate of about 3.
3. SS90F at 55 to 60 m/s/s is at the grip limit, not in the linear slip
   regime (step in the table, falling success rate). The linear model
   should not be used there.
4. Working value: k / L about 3 deg/g. The two 90 degree turns, the
   cleanest case, give 2.8 (DD90) and 3.5 (SS90F); a single fit to five
   turns gives 2.8, within about one tuning step for all of them. Good
   enough to predict positions within a small error band for later fine
   tuning.
5. In the app, about 3 deg/g with Load 1 is equivalent, since only k / L
   matters. With Load set to the fan load of about 2.2 to 3, the Slip
   value is about 6 to 9 deg/g. Absolute k is uncertain because L is not
   measured.
6. DS135 does not follow the model; cause unknown (section 7). SD45 and
   SD135 differ from the common value by about one tuning step, possibly
   from angle-dependent coupling or the wheel load split (section 9.3).
7. Fan droop at high speed: no clear evidence either way in this table.

A cross-slope measurement of k (crab angle / tan(slope)), no fan, would
give absolute k and so L. It may not match exactly if the wheel load
split changes with the fan.

## 11. Open questions

1. Which of the section 7 causes, if any, explains DS135?
2. Does a tyre saturation model with a knee between 50 and 55 m/s/s
   reproduce the SS90F step with a plausible mu?
3. How much does the uneven wheel load split, and its change with fan
   load, alter the effective slip coefficient?

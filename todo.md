# TODO

Work top to bottom. Build, run and test after each item before moving on.
Commit after each verified step.

## 1. Bugs

- [x] 1.1 Remove duplicate signal connections
  - `mainwindow.cpp` ctor manually connects `btnReset` and `turnSpeedSpinBox`;
    `connectSlotsByName` already does this, so slots fire twice.
  - Test: add temporary qDebug in slots, confirm one call per click/change.

- [x] 1.2 Reset uses wrong table for Trapezoid
  - `on_btnReset_clicked` always restores `defaultParams`; Trapezoid uses `trapezoidParams`.
  - Test: Trapezoid, edit radius, Reset -> radius returns to trapezoid default (e.g. SS90F = 96).

- [x] 1.3 Profile switch discards user edits for all turns
  - `on_rb<Profile>_clicked` overwrite every `currentParams[]` entry.
  - Decision: retain edits until explicitly reset (Reset button). Profiles stay
    fully separate: different behaviour, rationale and parameter sets.
  - Plan: keep a separate `currentParams` set per profile, initialised from that
    profile's defaults; switching profile only selects the set. Reset restores the
    current turn of the current profile to its defaults.
  - Test: edit SS90F radius, switch profile and back -> edit still present;
    Reset -> profile default restored.

- [x] 1.4 Turn speed not stored/restored per turn
  - `currentParams[].speed` written, never read (`setParams` line commented out).
  - Test: set SS90F speed, change to SS180, check SS180 default speed; back to SS90F, check edited speed.

- [x] 1.5 Infinite loop when lambda too large for radius/angle (was: lambda = 0 NaN)
  - Lambda = 0 tested: harmless, gives a pure arc. Original claim wrong.
  - Cause: 2 * transition angle > turn angle -> negative mid arc, phase 3 overruns,
    omega goes negative, theta never reaches end -> hang. Quadratic SD45 defaults
    hung (pre-existing). Very small radius hung quadratic via step-size error.
  - Fix: lambda clamped to max (spinbox + stored value), "Lambda Max" in summary;
    quadratic has own defaults table (SD45 lambda 60); turn loops also end at
    turnDistance.
  - Tested: all 3 profiles x 8 turns at defaults, lambda 200, radius 15; defaults and
    lambda 200 results unchanged by the distance limit.

- [x] 1.6 `MouseItem::paint` changes item geometry
  - Move `setPos`/`setRotation` from `paint()` into `setPose()`.
  - Test: slider moves mouse correctly, no visual lag or repaint storm.

- [x] 1.7 `Track` index and empty-track safety
  - `set_turn_end()` stores record count (first leadout record), not last turn record:
    off by one for turn length, time, exit pose.
  - `get_max_*` dereference `max_element` on possibly empty vectors.
  - Result: finish index now last phase-3 record; exit x/y, distance -1 step (<= 1.09 mm),
    time -1 ms; exit theta unchanged (leadout omega = 0). Empty track returns 0 / origin.

- [x] 1.8 Trapezoid omits initial `add_record` (other profiles have it)
  - Test: graph and breadcrumbs start at the same point for all profiles.

## 2. Performance

- [x] 2.1 Progress slider only moves the mouse
  - Currently slider tick re-simulates turn, rebuilds scene and graph.
  - Split `rePaint()` into recalculate and update-mouse-position.
  - Test: drag slider smoothly; label and mouse position still correct.

- [x] 2.2 Avoid cascade of repaints when selecting a turn/profile
  - `setParams()` sets 4 spinboxes, each triggers `rePaint()`, then handler repaints again.
  - Block signals during bulk update, repaint once.
  - Test: one recalculation per turn/profile click (temporary qDebug counter).
  - Result: turn 5-6 -> 1, profile 3 -> 1. Also fixed: start X/Y not updated when
    new turn had same offset but different base/angle (e.g. SS90E -> DS45).

- [ ] 2.3 Draw maze walls once
  - Walls/grid are static; stop clearing and rebuilding them each repaint.
    Keep breadcrumbs in a separate group that is cleared on recalculation.
  - Test: visual output unchanged; breadcrumbs update on parameter change.

## 3. Structure

- [x] 3.1 Remove `setValue(v + 1); setValue(v);` workaround (TODO in profile slots)
  - Done as part of 1.4 (workaround overwrote restored speed).
  - Call the recalculation directly instead.
  - Test: profile switch updates acceleration and plot as before.

- [ ] 3.2 Merge repeated radio button slots
  - Use `QButtonGroup` with ids for the 8 turn types and the 4 profiles.
  - Test: every turn type and profile selectable, parameters load correctly.

- [ ] 3.3 Move turn physics out of `MainWindow`
  - Plain C++ module (no Qt): input params struct -> `Track`.
  - Share common setup between trapezoid, quadratic and sine (only the omega
    transition function differs).
  - Test: summary values identical to pre-refactor for each turn/profile
    (record a reference table first).

- [ ] 3.4 Extensible profile types (builds on 3.2, 3.3)
  - Goal: adding a profile = one new source file + one registry line, no MainWindow edits.
  - Each profile defines:
    - name and description
    - its own parameter list (id, label, units, range, step); profiles do not share
      a common parameter mapping (e.g. cubic uses length/gamma, not radius/lambda)
    - per-turn default values
    - calculate(turn geometry, params) -> `Track`
  - UI built from the registry: profile radio buttons and parameter spinboxes
    generated per profile, replacing fixed enable/disable logic.
  - Per-profile, per-turn edited values stored separately (see 1.3).
  - Common turn geometry (start offset, base x/y, start angle, turn angle) stays
    shared and outside profiles.
  - Test: existing 4 profiles reproduce reference values from 3.3; add a dummy
    profile to confirm no other files need changing, then remove it.

- [ ] 3.5 Store gamma per turn like other parameters (likely absorbed by 3.4)

- [ ] 3.6 Single definition of PI / RADIANS / DEGREES
  - Replace 3 duplicate macros with one header of constexpr functions; consistent float/double use.

- [ ] 3.7 Remove dead code and leaks
  - Unused `PathItem` / `thePath`; `mouseScene` and `thePath` never deleted (give parents).
  - Unused locals in `MouseItem::paint`; empty `closeEvent`; commented-out code.
  - Rename public `g_track` member, make it private.
  - Pass `QPoint` by const ref in `drawSensor`.

- [ ] 3.8 Add tests for the physics module (after 3.3)
  - e.g. exit angle within tolerance, turn distance matches formula, no NaN.

## 4. Build and portability

- [ ] 4.1 Designated initializers in param tables need C++20 (MSVC rejects in C++17)
  - Switch to C++20 or positional initializers.
- [ ] 4.2 Raise `cmake_minimum_required` to 3.16 (removes deprecation warning, matches README).
- [ ] 4.3 New-style `connect()` (function pointers) instead of `SIGNAL()/SLOT()` where manual connects remain.
- [ ] 4.4 Include hygiene: `<algorithm>` not `"algorithm"`; drop `<QtGui>` umbrella include from header.

## 5. UI

- [ ] 5.1 Replace hand-edited `ui_mainwindow.h`
  - Real `.ui` file (AUTOUIC) or layout-based code; removes fixed geometry and
    deprecated `QApplication::translate(..., 0)`; fix `gpTurnType` object name.
  - Test: all widgets present and named, app behaves as before; check Fusion and Breeze.
- [ ] 5.2 Auto-scale graph axes (currently fixed 0-0.5 s, 0-1600 deg/s, 0-3200 mm/s)
  - Test: SS180 at low speed fully visible.
- [ ] 5.3 Allow window resize (drop `setFixedSize`) once layouts are in place (after 5.1).
- [ ] 5.4 Fix invalid stylesheet `font-weight: xbold` on `lblPathView`.
- [ ] 5.5 Remove or repurpose redundant Refresh button.
- [ ] 5.6 Avoid hard-coded font in summary text where possible (use system fixed font).

## 6. Deployment (optional)

- [ ] 6.1 Release build for deploy (separate `build-release` dir).
- [ ] 6.2 `undeploy` target to remove binary, icon and `.desktop` file.
- [ ] 6.3 Square 256x256 icon for theme install.
- [ ] 6.4 AppImage for machines without Qt6.

## 7. Features

- [ ] 7.1 Simulate tyre slip and slip angle
  - Currently ideal kinematics: velocity always along heading, no slip.
  - Model lateral slip: velocity direction differs from heading by slip angle;
    lateral tyre force from slip angle (e.g. linear cornering stiffness,
    saturating at mu * normal load).
  - Possibly longitudinal slip per wheel (relevant for cubic gamma braking).
  - New robot/tyre parameters (mass, mu, cornering stiffness, etc.), shared
    across profiles, not per profile.
  - Outputs: slip angle vs time on graph, actual vs ideal path in path view,
    exit position/heading error in summary.
  - Depends on 3.3 (physics module) and 3.4 (parameter handling).
  - Test: slip off (or very high stiffness) reproduces ideal results; slip angle
    grows with speed/centripetal acceleration; path drifts outward.
  - Open: model choice and parameter values to be decided before starting.

- [ ] 7.2 Rework phase calculation for segmented turns (trapezoid, quadratic, sinusoid)
  - Phases currently switch on theta, which lags by a step; turn loops also stop at turnDistance (1.5).
  - Known effect: trapezoid phase 1 ramp overshoots turn omega, simulated peak
    centripetal acceleration up to 0.6% above analytic value (e.g. DS45 9864 vs 9802).
  - Test: simulated peak matches acceleration box for all turns; exit angle within tolerance.

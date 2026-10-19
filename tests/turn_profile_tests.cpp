/**
 * @file turn_profile_tests.cpp
 * @brief Unit tests and baseline regression for the turn profile physics (no Qt).
 *
 * Usage:
 *   turn_profile_tests <baseline file>            run checks, compare with baseline
 *   turn_profile_tests <baseline file> --update   run checks, rewrite the baseline
 *
 * The baseline holds one line per case: record count, turn end index, exit
 * pose, turn length and time, peak centripetal acceleration, peak omega and
 * exit speed. Update it only when a physics change is intended, and review
 * the diff.
 *
 * Cases use the profile defaults, with the start position worked out as the
 * UI does (offset back from the turn base point, truncated to whole mm).
 * Variants: "def" (defaults), "lam200" (lambda 200, then clamped),
 * "r15" (radius 15, lambda clamped), "L60" (cubic length 60).
 *
 * Slip checks (not in the baseline) compare each default case with slip on
 * against the same case without slip (see checkSlip()).
 *
 * Exit code 0 when everything passes.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "angles.h"
#include "robot/cubic_profile.h"
#include "robot/hybrid_turn_profile.h"
#include "robot_sim.h"
#include "turn_profile.h"

namespace {

int failures = 0;

/// Report a failed check
void fail(const std::string &where, const std::string &what) {
  std::printf("FAIL %s: %s\n", where.c_str(), what.c_str());
  failures++;
}

/// Check a condition, reporting where and what on failure
void check(bool ok, const std::string &where, const std::string &what) {
  if (!ok) {
    fail(where, what);
  }
}

const char *turnNames[TURN_TYPE_COUNT] = {"SS90F", "SS180", "SD45", "SD135", "DS45", "DS135", "DD90", "SS90E"};

/// One simulation case
struct Case {
  std::string name;  ///< "<profile> <turn> <variant>", no spaces in the parts
  const TurnProfile *profile;
  Turn_t turn;
  float speed;
  float offset;
  std::vector<float> values;
};

/**
 * @brief Simulation setup for a turn, as the UI builds it.
 *
 * The start point is offset back from the turn base point along the start
 * heading; the UI holds it in integer spinboxes, so it is truncated.
 */
TurnSetup setupFor(Turn_t turn, float offset) {
  const TurnGeometry &g = turnGeometry[turn];
  float start_x = g.base_x - offset * std::cos(PI / 2 + radians(g.start_angle));
  float start_y = g.base_y - offset * std::sin(PI / 2 + radians(g.start_angle));
  TurnSetup setup;
  setup.start_x = int(start_x);
  setup.start_y = -int(start_y);
  setup.start_angle = -g.start_angle;
  setup.turn_angle = g.angle;
  setup.interval = 0.001f;
  return setup;
}

/// Profile name without spaces, for case names
std::string shortName(const TurnProfile *profile) {
  std::string n = profile->name();
  for (char &c : n) {
    if (c == ' ') {
      c = '_';
    }
  }
  return n;
}

/// Index of a parameter by id, or -1
int paramIndex(const TurnProfile *profile, const std::string &id) {
  std::vector<ParamDef> defs = profile->params();
  for (size_t i = 0; i < defs.size(); i++) {
    if (defs[i].id == id) {
      return int(i);
    }
  }
  return -1;
}

/// All cases: every profile and turn, defaults plus variants for the profile's parameters
std::vector<Case> buildCases() {
  std::vector<Case> cases;
  for (const TurnProfile *profile : profile_registry()) {
    for (int t = 0; t < TURN_TYPE_COUNT; t++) {
      Turn_t turn = Turn_t(t);
      TurnDefaults d = profile->defaults(turn);
      std::string base = shortName(profile) + " " + turnNames[t] + " ";
      cases.push_back({base + "def", profile, turn, d.speed, d.offset, d.values});
      int lambda = paramIndex(profile, "lambda");
      int radius = paramIndex(profile, "radius");
      int length = paramIndex(profile, "length");
      if (lambda >= 0) {
        Case c{base + "lam200", profile, turn, d.speed, d.offset, d.values};
        c.values[lambda] = 200;
        cases.push_back(c);
      }
      if (radius >= 0) {
        Case c{base + "r15", profile, turn, d.speed, d.offset, d.values};
        c.values[radius] = 15;
        cases.push_back(c);
      }
      if (length >= 0) {
        Case c{base + "L60", profile, turn, d.speed, d.offset, d.values};
        c.values[length] = 60;
        cases.push_back(c);
      }
    }
  }
  return cases;
}

/// Registry and parameter definitions are consistent
void checkRegistry() {
  const auto &profiles = profile_registry();
  check(profiles.size() >= 4, "registry", "expected at least 4 profiles");
  std::set<std::string> names;
  for (const TurnProfile *profile : profiles) {
    std::string where = "registry " + profile->name();
    check(names.insert(profile->name()).second, where, "duplicate profile name");
    std::vector<ParamDef> defs = profile->params();
    check(!defs.empty(), where, "no parameters");
    std::set<std::string> ids;
    for (const ParamDef &def : defs) {
      check(ids.insert(def.id).second, where, "duplicate parameter id " + def.id);
      check(def.min < def.max && def.step > 0, where, "bad range for " + def.id);
    }
    for (int t = 0; t < TURN_TYPE_COUNT; t++) {
      TurnDefaults d = profile->defaults(Turn_t(t));
      std::string w = where + " " + turnNames[t];
      check(d.values.size() == defs.size(), w, "defaults do not match params");
      for (size_t i = 0; i < d.values.size() && i < defs.size(); i++) {
        check(d.values[i] >= defs[i].min && d.values[i] <= defs[i].max, w, "default " + defs[i].id + " out of range");
      }
      check(d.speed > 0, w, "speed not positive");
    }
  }
}

/**
 * @brief Run one case: constrain, simulate, check properties, return the baseline line.
 */
std::string runCase(Case c) {
  TurnSetup setup = setupFor(c.turn, c.offset);
  c.profile->constrain(setup, c.values);
  Track track;
  c.profile->simulate(setup, c.speed, c.values, track);

  const std::string &w = c.name;
  bool isDefault = c.name.size() > 3 && c.name.compare(c.name.size() - 3, 3, "def") == 0;

  // records are finite; turn end is the last turn record, followed by leadout
  check(!track.empty(), w, "empty track");
  bool finite = true;
  for (int i = 0; i < track.record_count(); i++) {
    Pose p = track.get_pose_at(i);
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.theta()) || !std::isfinite(track.get_omega_at(i))) {
      finite = false;
    }
  }
  check(finite, w, "non-finite values in track");
  int fin = track.finish_index();
  check(fin > 0 && fin < track.record_count() - 1, w, "turn end index out of range");
  if (fin > 0 && fin < track.record_count() - 1) {
    check(track.get_phase_at(fin) != 4 && track.get_phase_at(fin + 1) == 4, w, "turn end not before leadout");
  }

  // exit heading close to the target
  float target = setup.start_angle + setup.turn_angle;
  float error = std::fabs(track.get_exit_pose().theta() - target);
  float allowed = isDefault ? 0.15f : 0.6f;
  char buf[160];
  std::snprintf(buf, sizeof buf, "exit angle error %.3f deg > %.2f", error, allowed);
  check(error <= allowed, w, buf);

  // simulated peak centripetal acceleration matches the profile formula
  float formula = c.profile->peak_acceleration(setup, c.speed, c.values);
  if (isDefault) {
    float rel = std::fabs(track.get_max_acceleration() - formula) / formula;
    std::snprintf(buf, sizeof buf, "peak acceleration %.1f vs formula %.1f", track.get_max_acceleration(), formula);
    check(rel < 0.01f, w, buf);
  }

  // speed_for_acceleration inverts peak_acceleration
  float back = c.profile->speed_for_acceleration(setup, formula, c.values);
  std::snprintf(buf, sizeof buf, "speed round trip %.4f vs %.4f", back, c.speed);
  check(std::fabs(back - c.speed) / c.speed < 1e-5f, w, buf);

  // constrained values are stable: constraining again changes nothing
  std::vector<float> again = c.values;
  check(!c.profile->constrain(setup, again), w, "constrain not idempotent");

  Pose e = track.get_exit_pose();
  std::ostringstream line;
  line.setf(std::ios::fixed);
  line.precision(6);
  line << c.name << " n " << track.record_count() << " fin " << fin << " x " << e.x() << " y " << e.y() << " th " << e.theta() << " len " << track.turn_length()
       << " t " << track.get_turn_time() << " acc " << track.get_max_acceleration() << " om " << track.get_max_omega() << " vex " << track.get_exit_speed();
  return line.str();
}

/**
 * @brief Check the peak slip of a track against the settled value.
 *
 * Slip lags the acceleration, so it reaches the settled value
 * gradient * (a / g) / load factor only if the arc is long enough, and never
 * exceeds it.
 *
 * @param min_ratio smallest allowed peak / settled; short turns at high speed
 *        (tau grows with speed while the turn gets shorter) reach less
 */
void checkPeakSlip(const Track &track, const SlipParams &slip, const std::string &w, const char *what, float min_ratio) {
  float settled = slip.gradient * track.get_max_acceleration() / GRAVITY / slip.load_factor;
  float ratio = track.get_max_slip() / settled;
  char buf[160];
  std::snprintf(buf, sizeof buf, "%s: max slip %.4f is %.3f of settled %.4f", what, track.get_max_slip(), ratio, settled);
  check(ratio > min_ratio && ratio < 1.001f, w, buf);
}

/**
 * @brief Tyre slip checks for every profile and turn at the defaults.
 *
 * With slip on, heading, distance and record count must equal the no slip
 * case; only the position changes. Peak slip is checked against the settled
 * value (checkPeakSlip()) at the defaults, at load factor 2 and at double
 * speed (upper bound only, and at least double the default slip). The path must never cross to the inside of the start line, slip must
 * die away in the leadout, and the exit point must move to the outside of the
 * turn, taken as the outward normal at mid turn.
 */
void checkSlip() {
  for (const TurnProfile *profile : profile_registry()) {
    for (int t = 0; t < TURN_TYPE_COUNT; t++) {
      Turn_t turn = Turn_t(t);
      TurnDefaults d = profile->defaults(turn);
      std::string w = "slip " + shortName(profile) + " " + turnNames[t];
      TurnSetup setup = setupFor(turn, d.offset);
      std::vector<float> values = d.values;
      profile->constrain(setup, values);

      Track ideal;
      profile->simulate(setup, d.speed, values, ideal);
      check(ideal.get_max_slip() == 0, w, "slip without slip model");

      setup.slip.gradient = 5;
      Track slipped;
      profile->simulate(setup, d.speed, values, slipped);
      check(slipped.record_count() == ideal.record_count() && slipped.finish_index() == ideal.finish_index(), w, "record count or turn end changed by slip");
      check(slipped.get_exit_pose().theta() == ideal.get_exit_pose().theta(), w, "exit heading changed by slip");
      check(slipped.turn_length() == ideal.turn_length(), w, "turn length changed by slip");
      bool finite = true;
      for (int i = 0; i < slipped.record_count(); i++) {
        Pose p = slipped.get_pose_at(i);
        finite = finite && std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(slipped.get_slip_at(i));
      }
      check(finite, w, "non-finite values with slip");
      checkPeakSlip(slipped, setup.slip, w, "defaults", 0.9f);

      char buf[160];
      // heading h = (sin th, -cos th), turning towards n = (cos th, sin th)
      float start = radians(setup.start_angle);
      Pose p0 = slipped.get_pose_at(0);
      float inside = 0;
      for (int i = 0; i < slipped.record_count(); i++) {
        Pose p = slipped.get_pose_at(i);
        inside = std::min(inside, (p.x() - p0.x()) * std::cos(start) + (p.y() - p0.y()) * std::sin(start));
      }
      std::snprintf(buf, sizeof buf, "path %.4f mm on the wrong side of the start line", -inside);
      check(inside > -1e-3f, w, buf);

      float last = slipped.get_slip_at(slipped.record_count() - 1);
      std::snprintf(buf, sizeof buf, "leadout end slip %.4f not below 1%% of max %.4f", last, slipped.get_max_slip());
      check(std::fabs(last) < 0.01f * slipped.get_max_slip(), w, buf);

      float mid = radians(setup.start_angle + setup.turn_angle / 2);
      float dx = slipped.get_exit_pose().x() - ideal.get_exit_pose().x();
      float dy = slipped.get_exit_pose().y() - ideal.get_exit_pose().y();
      float outward = -(dx * std::cos(mid) + dy * std::sin(mid));
      std::snprintf(buf, sizeof buf, "exit displacement %.3f mm not outward", outward);
      check(outward > 0, w, buf);

      setup.slip.load_factor = 2;
      Track loaded;
      profile->simulate(setup, d.speed, values, loaded);
      checkPeakSlip(loaded, setup.slip, w, "load factor 2", 0.9f);
      check(loaded.get_max_slip() < slipped.get_max_slip(), w, "load factor 2 does not reduce slip");

      setup.slip.load_factor = 1;
      Track fast;
      profile->simulate(setup, 2 * d.speed, values, fast);
      checkPeakSlip(fast, setup.slip, w, "double speed", 0);
      check(fast.get_max_slip() > 2 * slipped.get_max_slip(), w, "double speed does not double slip");
    }
  }
}

/// Split a baseline line into its case name and the value fields keyed by label
std::pair<std::string, std::map<std::string, double>> parseLine(const std::string &line) {
  std::istringstream in(line);
  std::string profile, turn, variant;
  in >> profile >> turn >> variant;
  std::map<std::string, double> fields;
  std::string key;
  double value;
  while (in >> key >> value) {
    fields[key] = value;
  }
  return {profile + " " + turn + " " + variant, fields};
}

/**
 * @brief Compare results with the baseline file.
 *
 * Counts must match exactly; other values within 1e-4 absolute or 1e-6
 * relative, to allow for compiler differences in float rounding.
 */
void compareBaseline(const std::vector<std::string> &lines, const std::string &path) {
  std::ifstream file(path);
  if (!file) {
    fail("baseline", "cannot read " + path + " (run with --update to create it)");
    return;
  }
  std::map<std::string, std::map<std::string, double>> expected;
  std::string text;
  while (std::getline(file, text)) {
    if (!text.empty() && text[0] != '#') {
      auto parsed = parseLine(text);
      expected[parsed.first] = parsed.second;
    }
  }
  std::set<std::string> seen;
  for (const std::string &l : lines) {
    auto actual = parseLine(l);
    seen.insert(actual.first);
    auto it = expected.find(actual.first);
    if (it == expected.end()) {
      fail(actual.first, "not in baseline");
      continue;
    }
    for (const auto &kv : actual.second) {
      double want = it->second[kv.first];
      double got = kv.second;
      bool exact = (kv.first == "n" || kv.first == "fin");
      double tol = exact ? 0 : std::max(1e-4, 1e-6 * std::fabs(want));
      if (std::fabs(got - want) > tol) {
        char buf[120];
        std::snprintf(buf, sizeof buf, "%s %.6f, baseline %.6f", kv.first.c_str(), got, want);
        fail(actual.first, buf);
      }
    }
  }
  for (const auto &kv : expected) {
    if (!seen.count(kv.first)) {
      fail(kv.first, "in baseline but not run");
    }
  }
}

/// Profile in the registry by name, or nullptr
const TurnProfile *findProfile(const std::string &name) {
  for (const TurnProfile *profile : profile_registry()) {
    if (profile->name() == name) {
      return profile;
    }
  }
  return nullptr;
}

/// Exit pose of the previous heading-switched simulation, for checkExitLine()
struct ReferenceExit {
  const char *profile;  ///< registry name
  Turn_t turn;
  float x;   ///< exit x, mm
  float y;   ///< exit y, mm
  float th;  ///< exit heading, degrees
};

/**
 * Exit poses at the defaults from the simulation before the robot profiles
 * (phases switched on heading, cubic omega sampled before each step), at
 * interval 0.1 ms. The heading-switched turns stop up to about 0.01 deg
 * short, so they are compared only across the exit line.
 */
const ReferenceExit referenceExits[] = {
    {"Sinusoid", TURN_SS90F, 147.402f, 0.874f, 89.991f},      {"Sinusoid", TURN_SS180, 180.012f, 158.543f, 179.991f},
    {"Sinusoid", TURN_SD45, 48.284f, 42.041f, 44.991f},       {"Sinusoid", TURN_SD135, 168.284f, 78.737f, 134.990f},
    {"Sinusoid", TURN_DS45, 161.103f, 0.555f, 89.990f},       {"Sinusoid", TURN_DS135, 179.418f, 140.781f, 179.991f},
    {"Sinusoid", TURN_DD90, 161.915f, 73.119f, 134.991f},     {"Sinusoid", TURN_SS90E, 73.285f, 0.820f, 89.991f},
    {"Cubic Spiral", TURN_SS90F, 148.838f, 1.035f, 90.003f},  {"Cubic Spiral", TURN_SS180, 179.858f, 159.829f, 179.989f},
    {"Cubic Spiral", TURN_SD45, 49.307f, 40.757f, 45.001f},   {"Cubic Spiral", TURN_SD135, 169.713f, 79.561f, 134.995f},
    {"Cubic Spiral", TURN_DS45, 162.293f, 0.597f, 90.000f},   {"Cubic Spiral", TURN_DS135, 179.063f, 141.872f, 180.005f},
    {"Cubic Spiral", TURN_DD90, 163.108f, 73.934f, 135.000f}, {"Cubic Spiral", TURN_SS90E, 74.426f, 0.495f, 90.000f},
    {"Hybrid", TURN_SS90F, 148.044f, 0.386f, 89.991f},        {"Hybrid", TURN_SS180, 180.434f, 158.026f, 179.990f},
    {"Hybrid", TURN_SD45, 48.440f, 41.764f, 44.991f},         {"Hybrid", TURN_SD135, 167.639f, 78.960f, 134.990f},
    {"Hybrid", TURN_DS45, 160.671f, 0.800f, 89.991f},         {"Hybrid", TURN_DS135, 179.164f, 140.529f, 179.990f},
    {"Hybrid", TURN_DD90, 161.167f, 73.152f, 134.991f},       {"Hybrid", TURN_SS90E, 73.683f, 0.396f, 89.990f},
};

/**
 * @brief Distance of a pose from an exit line, positive to the left of travel.
 * @param p pose to measure
 * @param x0 point on the line, mm
 * @param y0 point on the line, mm
 * @param th line heading, degrees (simulation convention: travel along (sin th, -cos th))
 */
float crossTrack(const Pose &p, float x0, float y0, float th) {
  float t = radians(th);
  return (p.x() - x0) * std::cos(t) + (p.y() - y0) * std::sin(t);
}

/// Blend for a segmented profile's values: the hybrid's own, 0 for the sinusoid
float blendOf(const TurnProfile *profile, const std::vector<float> &values) {
  return profile->name() == "Hybrid" ? values[2] : 0.0f;
}

/**
 * @brief HybridTurnProfile path functions for every turn and several blends.
 *
 * Heading continuous at the segment joins, zero at the start and the turn
 * angle at the end; its slope (central difference) equals curvature_at();
 * curvature 1 / R in the arc; length = 2 lambda + arc. Both turn directions.
 */
void checkHybridPath() {
  const TurnProfile *hybrid = findProfile("Hybrid");
  for (int t = 0; t < TURN_TYPE_COUNT; t++) {
    TurnDefaults d = hybrid->defaults(Turn_t(t));
    TurnSetup setup = setupFor(Turn_t(t), d.offset);
    for (float blend : {0.0f, 0.5f, d.values[2], 0.95f}) {
      for (float sign : {1.0f, -1.0f}) {
        std::vector<float> values = d.values;
        values[2] = blend;
        hybrid->constrain(setup, values);
        char where[80];
        std::snprintf(where, sizeof where, "hybrid path %s blend %.2f sign %+.0f", turnNames[t], blend, sign);
        float angle = sign * setup.turn_angle;
        HybridTurnProfile p;
        bool ok = p.init_turn(angle, values[0], values[1], blend) == ProfileStatus::SUCCESS;
        check(ok, where, "init_turn failed");
        if (!ok) {
          continue;
        }
        float L = p.length();
        float lam = p.transition_length();
        char buf[160];
        check(p.heading_at(0) == 0 && std::fabs(p.heading_at(L) - angle) < 1e-4f, where, "end headings");
        float joins[] = {lam, L - lam};
        for (float j : joins) {
          // change expected across the join from the curvature there
          float h = 1e-3f;
          float jump = p.heading_at(j + h) - p.heading_at(j - h) - degrees(2 * h * p.curvature_at(j));
          std::snprintf(buf, sizeof buf, "heading jump %.6f deg at s = %.2f", jump, j);
          check(std::fabs(jump) < 1e-4f, where, buf);
        }
        for (int i = 1; i < 20; i++) {
          // h large enough for float precision, small for the difference error
          float s = L * i / 20;
          float h = 0.5f;
          float slope = radians(p.heading_at(s + h) - p.heading_at(s - h)) / (2 * h);
          float k = p.curvature_at(s);
          std::snprintf(buf, sizeof buf, "slope %.6f vs curvature %.6f at s = %.1f", slope, k, s);
          check(std::fabs(slope - k) < 2e-4f / values[0] + 1e-5f, where, buf);
        }
        std::snprintf(buf, sizeof buf, "arc curvature %.6f vs 1/R %.6f", p.curvature_at(L / 2), sign / values[0]);
        check(std::fabs(p.curvature_at(L / 2) - sign / values[0]) < 1e-6f, where, buf);
      }
    }
  }
}

/**
 * @brief Sum of omega * dt over the ticks of a profile at constant speed.
 * @return total heading change, degrees
 */
float tickAngle(VelocityProfile &profile, float speed, float dt) {
  double sum = 0;
  int ticks = 0;
  do {
    profile.update(speed);
    sum += double(profile.getVelocity()) * dt;
  } while (!profile.isComplete() && ++ticks < 100000);
  return float(sum);
}

/**
 * @brief Total angle delivered by the robot profiles at a 1 ms tick.
 *
 * Hybrid, sinusoid and the cubic in exact mode: sum of omega * dt equals the
 * turn angle within 1e-3 deg at speeds for 1 g and 55 m/s/s, both directions.
 * The cubic in firmware mode is reported, not checked.
 */
void checkTickAngle() {
  const float dt = 0.001f;
  const TurnProfile *cubic = findProfile("Cubic Spiral");
  for (int t = 0; t < TURN_TYPE_COUNT; t++) {
    for (float accel : {GRAVITY, 55000.0f}) {
      for (float sign : {1.0f, -1.0f}) {
        float angle = sign * turnGeometry[t].angle;
        char where[80];
        std::snprintf(where, sizeof where, "tick angle %s %.0f mm/s/s sign %+.0f", turnNames[t], accel, sign);
        char buf[160];
        for (const char *name : {"Sinusoid", "Hybrid"}) {
          const TurnProfile *profile = findProfile(name);
          TurnDefaults d = profile->defaults(Turn_t(t));
          HybridTurnProfile p(dt);
          p.init_turn(angle, d.values[0], d.values[1], blendOf(profile, d.values));
          float error = tickAngle(p, std::sqrt(accel * d.values[0]), dt) - angle;
          std::snprintf(buf, sizeof buf, "%s angle error %.6f deg", name, error);
          check(std::fabs(error) < 1e-3f, where, buf);
        }
        TurnDefaults d = cubic->defaults(Turn_t(t));
        TurnSetup setup = setupFor(Turn_t(t), d.offset);
        float speed = cubic->speed_for_acceleration(setup, accel, d.values);
        CubicProfile exact(dt);
        exact.set_exact_angle(true);
        exact.init(d.values[0], 0, 0, 0, 0, angle);
        float exact_error = tickAngle(exact, speed, dt) - angle;
        std::snprintf(buf, sizeof buf, "cubic exact angle error %.6f deg", exact_error);
        check(std::fabs(exact_error) < 1e-3f, where, buf);
        CubicProfile firmware(dt);
        firmware.init(d.values[0], 0, 0, 0, 0, angle);
        float firmware_error = tickAngle(firmware, speed, dt) - angle;
        if (sign > 0) {
          std::printf("info %s: cubic firmware mode angle error %.4f deg, exact mode %.6f deg\n", where, firmware_error, exact_error);
        }
      }
    }
  }
}

/**
 * @brief Exit of the robot profiles against the reference exits, both at 0.1 ms.
 *
 * The reference is the previous simulation (referenceExits). The exit must
 * lie within 0.1 mm of the reference exit line, heading within 0.02 deg of
 * the target. Sinusoid, hybrid and the cubic in exact mode are checked; the
 * cubic in firmware mode (the app's simulation) is reported. The same run at
 * 1 ms is also reported: RobotState moves along the heading at the start of
 * each step, which offsets the exit by roughly half a step.
 */
void checkExitLine() {
  char buf[160];
  for (const ReferenceExit &r : referenceExits) {
    const TurnProfile *profile = findProfile(r.profile);
    TurnDefaults d = profile->defaults(r.turn);
    TurnSetup setup = setupFor(r.turn, d.offset);
    setup.interval = 0.0001f;
    std::vector<float> values = d.values;
    profile->constrain(setup, values);
    std::string where = std::string("exit line ") + r.profile + " " + turnNames[r.turn];
    TurnSetup coarse = setup;
    coarse.interval = 0.001f;
    Track coarse_track;
    profile->simulate(coarse, d.speed, values, coarse_track);
    std::printf("info %s: at 1 ms, %.3f mm across the exit line\n", where.c_str(),
                crossTrack(coarse_track.get_exit_pose(), r.x, r.y, setup.start_angle + setup.turn_angle));
    float target = setup.start_angle + setup.turn_angle;
    Track track;
    bool is_cubic = profile->name() == "Cubic Spiral";
    if (is_cubic) {
      CubicProfile exact(setup.interval);
      exact.set_exact_angle(true);
      exact.init(values[0], 0, 0, 0, 0, setup.turn_angle);
      simulate_robot_turn(exact, true, [] { return 1; }, setup, d.speed, track);
      Track firmware;
      profile->simulate(setup, d.speed, values, firmware);
      std::printf("info %s: cubic across exit line: firmware mode %.3f mm, exact mode %.3f mm\n", where.c_str(),
                  crossTrack(firmware.get_exit_pose(), r.x, r.y, target), crossTrack(track.get_exit_pose(), r.x, r.y, target));
    } else {
      profile->simulate(setup, d.speed, values, track);
    }
    Pose e = track.get_exit_pose();
    float off = crossTrack(e, r.x, r.y, target);
    std::snprintf(buf, sizeof buf, "%.3f mm off the reference exit line", off);
    check(std::fabs(off) < 0.1f, where, buf);
    std::snprintf(buf, sizeof buf, "exit heading %.4f vs %.4f", e.theta(), target);
    check(std::fabs(e.theta() - target) < 0.02f, where, buf);
  }
}

/**
 * @brief Path independent of speed: hybrid with the speed changed mid turn.
 *
 * Runs each default hybrid turn at constant speed and again with the speed
 * cut by 5% halfway through (both with RobotState speed following). The exit
 * points must agree within 0.3 mm and the headings within 1e-3 deg.
 */
void checkSpeedChange() {
  const TurnProfile *hybrid = findProfile("Hybrid");
  for (int t = 0; t < TURN_TYPE_COUNT; t++) {
    TurnDefaults d = hybrid->defaults(Turn_t(t));
    TurnSetup setup = setupFor(Turn_t(t), d.offset);
    Pose exits[2];
    for (int run = 0; run < 2; run++) {
      HybridTurnProfile p(setup.interval);
      p.init_turn(setup.turn_angle, d.values[0], d.values[1], d.values[2]);
      RobotState rs(setup.start_x, setup.start_y, setup.start_angle, d.speed, setup.interval);
      int ticks = 0;
      do {
        if (run == 1 && p.getPosition() > p.length() / 2) {
          rs.speed = 0.95f * d.speed;
        }
        p.update(rs.speed);
        rs.omega = p.getVelocity();
        rs.update();
      } while (!p.isComplete() && ++ticks < 100000);
      exits[run] = rs.pose;
    }
    // the profile's last step stops at the end of the turn; the robot state moves a full step
    std::string where = std::string("speed change ") + turnNames[t];
    float target = setup.start_angle + setup.turn_angle;
    char buf[160];
    float off = crossTrack(exits[1], exits[0].x(), exits[0].y(), target);
    std::snprintf(buf, sizeof buf, "exit %.3f mm across the constant speed exit line", off);
    check(std::fabs(off) < 0.3f, where, buf);
    std::snprintf(buf, sizeof buf, "exit heading %.5f vs %.5f", exits[1].theta(), exits[0].theta());
    check(std::fabs(exits[1].theta() - exits[0].theta()) < 1e-3f, where, buf);
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::printf("usage: %s <baseline file> [--update]\n", argv[0]);
    return 2;
  }
  std::string path = argv[1];
  bool update = (argc > 2 && std::string(argv[2]) == "--update");

  checkRegistry();
  std::vector<Case> cases = buildCases();
  std::vector<std::string> lines;
  for (const Case &c : cases) {
    lines.push_back(runCase(c));
  }
  checkSlip();
  checkHybridPath();
  checkTickAngle();
  checkExitLine();
  checkSpeedChange();

  if (update) {
    std::ofstream file(path);
    file << "# turn_profile_tests baseline: one line per case. Regenerate with --update\n";
    file << "# only for intended physics changes, and review the diff.\n";
    for (const std::string &l : lines) {
      file << l << "\n";
    }
    std::printf("baseline written: %zu cases to %s\n", lines.size(), path.c_str());
  } else {
    compareBaseline(lines, path);
  }

  std::printf("%zu cases, %d failure(s)\n", cases.size(), failures);
  return failures == 0 ? 0 : 1;
}

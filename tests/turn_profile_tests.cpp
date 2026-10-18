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
    std::string name;               ///< "<profile> <turn> <variant>", no spaces in the parts
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
                check(d.values[i] >= defs[i].min && d.values[i] <= defs[i].max, w,
                      "default " + defs[i].id + " out of range");
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
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.theta()) ||
            !std::isfinite(track.get_omega_at(i))) {
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
    line << c.name << " n " << track.record_count() << " fin " << fin << " x " << e.x() << " y " << e.y()
         << " th " << e.theta() << " len " << track.turn_length() << " t " << track.get_turn_time() << " acc "
         << track.get_max_acceleration() << " om " << track.get_max_omega() << " vex " << track.get_exit_speed();
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
void checkPeakSlip(const Track &track, const SlipParams &slip, const std::string &w, const char *what,
                   float min_ratio) {
    float settled = slip.gradient * track.get_max_acceleration() / GRAVITY / slip.load_factor;
    float ratio = track.get_max_slip() / settled;
    char buf[160];
    std::snprintf(buf, sizeof buf, "%s: max slip %.4f is %.3f of settled %.4f", what, track.get_max_slip(), ratio,
                  settled);
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
            check(slipped.record_count() == ideal.record_count() && slipped.finish_index() == ideal.finish_index(),
                  w, "record count or turn end changed by slip");
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
            std::snprintf(buf, sizeof buf, "leadout end slip %.4f not below 1%% of max %.4f", last,
                          slipped.get_max_slip());
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

/**
 * @file turn_profile.h
 * @brief Interface for turn profile types, the shared turn geometry and the profile registry.
 *
 * Independent of Qt. To add a profile:
 *  1. create profile_<name>.cpp with a TurnProfile subclass and an accessor
 *     function returning its single instance,
 *  2. add it to profile_registry() in profile_registry.cpp,
 *  3. add the file to CMakeLists.txt.
 * The UI builds its profile buttons and parameter spinboxes from the registry.
 *
 * Angles are in degrees and distances in mm. Simulation headings use the
 * RobotState convention: positive angles turn clockwise on screen.
 */

#ifndef TURN_PROFILE_H
#define TURN_PROFILE_H

#include <string>
#include <vector>
#include "robotstate.h"
#include "track.h"

/// Turn types; index into turnGeometry and the per-turn defaults
enum Turn_t {
    TURN_SS90F = 0, TURN_SS180, TURN_SD45, TURN_SD135, TURN_DS45, TURN_DS135, TURN_DD90, TURN_SS90E, TURN_TYPE_COUNT
};

/// Geometry of a turn type in the maze; the same for every profile
struct TurnGeometry {
    float base_x;       ///< x of the turn reference point, mm
    float base_y;       ///< y of the turn reference point, mm
    float start_angle;  ///< heading at the start of the turn, degrees (UI convention)
    float angle;        ///< total turn angle, degrees
};

/// Geometry for each turn type, indexed by Turn_t
extern const TurnGeometry turnGeometry[TURN_TYPE_COUNT];

/// Where and how a turn is simulated; everything except the profile's own parameters
struct TurnSetup {
    float start_x = 0;       ///< start position, mm
    float start_y = 0;       ///< start position, mm
    float start_angle = 0;   ///< start heading, degrees (simulation convention)
    float turn_angle = 0;    ///< total turn angle, degrees
    float interval = 0.001f; ///< simulation time step, s
    float leadout = 100;     ///< straight distance recorded after the turn, mm
    SlipParams slip{};       ///< tyre slip model; default is no slip
};

/// Description of one profile parameter, used to build its spinbox
struct ParamDef {
    std::string id;      ///< short identifier, used for the widget object name
    std::string label;   ///< label shown in the UI
    std::string units;   ///< units, shown as a tooltip
    double min;          ///< minimum value
    double max;          ///< maximum value
    double step;         ///< spinbox step
    int decimals;        ///< decimals shown
};

/// Values a profile starts from, and resets to, for one turn type
struct TurnDefaults {
    float offset;               ///< start offset back from the turn base point, mm
    float speed;                ///< turn speed, mm/s
    std::vector<float> values;  ///< profile parameters, in the order of TurnProfile::params()
};

/// An extra line for the turn summary
struct SummaryLine {
    std::string label;  ///< e.g. "Lambda Max"
    double value;
    int decimals;
    std::string units;
};

/**
 * @brief A turn profile type: its parameters, defaults and physics.
 *
 * Parameter values are passed as a vector in the order of params().
 */
class TurnProfile {
public:
    virtual ~TurnProfile() = default;

    /// Name shown on the profile button
    virtual std::string name() const = 0;

    /// One line description of the profile
    virtual std::string description() const = 0;

    /// The profile's own parameters; profiles do not share a common mapping
    virtual std::vector<ParamDef> params() const = 0;

    /// Default offset, speed and parameter values for a turn type
    virtual TurnDefaults defaults(Turn_t turn) const = 0;

    /**
     * @brief Simulate a turn and record it.
     *
     * Clears the track, records the turn, marks the turn end, then records a
     * straight leadout (see simulate_leadout()).
     */
    virtual void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values,
                          Track &track) const = 0;

    /// Peak centripetal acceleration at a given speed, mm/s/s
    virtual float peak_acceleration(const TurnSetup &setup, float speed, const std::vector<float> &values) const = 0;

    /// Speed giving a required peak centripetal acceleration; inverse of peak_acceleration()
    virtual float speed_for_acceleration(const TurnSetup &setup, float acceleration,
                                         const std::vector<float> &values) const = 0;

    /**
     * @brief Adjust parameter values that are invalid for this turn.
     * @return true if any value was changed
     */
    virtual bool constrain(const TurnSetup &setup, std::vector<float> &values) const {
        (void) setup;
        (void) values;
        return false;
    }

    /// Profile specific lines for the turn summary
    virtual std::vector<SummaryLine> summary(const TurnSetup &setup, const std::vector<float> &values) const {
        (void) setup;
        (void) values;
        return {};
    }
};

/**
 * @brief Record a straight leadout after the turn.
 *
 * @param robot_state state at the end of the turn; advanced in place
 * @param distance leadout distance, mm
 * @param track receives the records
 */
void simulate_leadout(RobotState &robot_state, float distance, Track &track);

/**
 * @brief All available profiles, in the order shown in the UI.
 *
 * The UI identifies a profile by its index in this list.
 */
const std::vector<const TurnProfile *> &profile_registry();

#endif  // TURN_PROFILE_H

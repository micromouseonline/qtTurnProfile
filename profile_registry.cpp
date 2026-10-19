/**
 * @file profile_registry.cpp
 * @brief The list of available turn profiles.
 *
 * To add a profile, declare its accessor here and add it to the list.
 */

#include "turn_profile.h"

// accessors defined in the profile_<name>.cpp files
const TurnProfile &sinusoid_profile();
const TurnProfile &trapezoid_profile();
const TurnProfile &cubic_profile();
const TurnProfile &hybrid_profile();

const std::vector<const TurnProfile *> &profile_registry() {
    static const std::vector<const TurnProfile *> profiles = {
            &sinusoid_profile(),
            &trapezoid_profile(),
            &cubic_profile(),
            &hybrid_profile(),
    };
    return profiles;
}

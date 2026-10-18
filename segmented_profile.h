/**
 * @file segmented_profile.h
 * @brief Base class for three phase turn profiles: ramp up, constant arc, ramp down.
 */

#ifndef SEGMENTED_PROFILE_H
#define SEGMENTED_PROFILE_H

#include "turn_profile.h"

/**
 * @brief A turn with a constant radius arc between two transitions.
 *
 * Phase 1 raises omega from zero to the arc value over a distance lambda,
 * phase 2 holds it, phase 3 mirrors phase 1. Phases switch on the heading
 * reached. Subclasses define the transition shape.
 *
 * Parameters: radius (index 0) and lambda (index 1).
 */
class SegmentedProfile : public TurnProfile {
public:
    enum { RADIUS = 0, LAMBDA = 1 };

    std::vector<ParamDef> params() const override;

    void simulate(const TurnSetup &setup, float speed, const std::vector<float> &values, Track &track) const override;

    /// a = v^2 / r: the peak is in the constant radius arc (valid while the arc exists)
    float peak_acceleration(const TurnSetup &setup, float speed, const std::vector<float> &values) const override;

    float speed_for_acceleration(const TurnSetup &setup, float acceleration,
                                 const std::vector<float> &values) const override;

    /// Clamps lambda to max_lambda()
    bool constrain(const TurnSetup &setup, std::vector<float> &values) const override;

    /// Adds "Lambda Max"
    std::vector<SummaryLine> summary(const TurnSetup &setup, const std::vector<float> &values) const override;

    /**
     * @brief Largest lambda that leaves a non-negative constant radius arc.
     *
     * Each transition turns through k * lambda / radius radians. Both must fit
     * in the turn angle, so lambda <= angle * radius / (2k). A larger lambda
     * makes phase 3 overrun and the turn falls short.
     *
     * @param radius arc radius, mm
     * @param turn_angle total turn angle, degrees
     * @return maximum lambda, mm
     */
    float max_lambda(float radius, float turn_angle) const;

protected:
    /// Angle turned in one transition, degrees, for lambda, arc omega (deg/s) and speed
    virtual float transition_angle(float lambda, float turn_omega, float speed) const = 0;

    /// Omega at distance x from the zero-omega end of a transition
    virtual float ramp(float turn_omega, float x, float lambda) const = 0;

    /// 1 / (2k): max_lambda() = lambda_limit() * angle in radians * radius
    virtual float lambda_limit() const = 0;

    /// The turn ends when theta is within this many degrees of the end angle
    virtual double tolerance() const { return 0.01; }
};

#endif  // SEGMENTED_PROFILE_H

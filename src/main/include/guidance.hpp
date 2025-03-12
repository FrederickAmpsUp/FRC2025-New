#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <swerve.hpp>
#include <studica/AHRS.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#define M_PI 3.1415926535

namespace ss {

class Guidance {
public:
    Guidance(ss::SwerveDrive& drive, ctre::phoenix::motorcontrol::can::TalonSRX& outtake, glm::vec2 frameStartPosition, studica::AHRS& navx);

    struct Info {
        /**
         * Velocity relative to the field (bird's eye) in m/s
        */
        glm::vec2 fieldVelocity;
        /**
         * Angular velocity (positive = clockwise) in radians/s
        */
        float angularVelocity;

        /**
         * Position relative to the field (bird's eye) in meters
        */
        glm::dvec2 fieldPosition;
        /**
         * Angle relative to the field (0 = away from DS, positive = clockwise) in radians
        */
        float fieldAngle;

        float outtakeAngle;
    };

    void update_guidance();
    void set_field_position(const glm::vec2& fieldPosition) { this->m_info->fieldPosition = fieldPosition; }

    std::shared_ptr<Info> info() { return this->m_info; }
private:
    ctre::phoenix::motorcontrol::can::TalonSRX& m_outtake;
    std::shared_ptr<Info> m_info;
    ss::SwerveDrive& m_drive;
    studica::AHRS& m_navx;
    std::vector<glm::dvec2> m_modulePositions;
};
}
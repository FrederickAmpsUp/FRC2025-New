#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <swerve.hpp>
#include <studica/AHRS.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#define M_PI 3.1415926535

namespace ss {

class Guidance {
public:
    Guidance(ss::SwerveDrive& drive, ctre::phoenix::motorcontrol::can::TalonSRX& outtake, studica::AHRS& navx);

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
    void set_field_position(const glm::vec2& fieldPosition) {
        int i = 0;
        for (const auto& mod : this->m_drive.m_modules) {
            glm::dvec2 modFrameOffset = mod.m_framePosition;
            double modFrameAngle = glm::atan(modFrameOffset.y, modFrameOffset.x);
            double modFrameDistance = glm::length(modFrameOffset);

            modFrameAngle += this->m_info->fieldAngle;

            this->m_modulePositions[i] = glm::dvec2(fieldPosition) + modFrameDistance * glm::dvec2(glm::cos(modFrameAngle), glm::sin(modFrameAngle));
            i++;
        }
        this->m_info->fieldPosition = fieldPosition;
    }

    std::shared_ptr<Info> info() { return this->m_info; }
private:
    ctre::phoenix::motorcontrol::can::TalonSRX& m_outtake;
    std::shared_ptr<Info> m_info;
    ss::SwerveDrive& m_drive;
    studica::AHRS& m_navx;
    std::vector<glm::dvec2> m_modulePositions;
    photon::PhotonCamera m_frontCamera;
    const frc::Pose3d m_frameToFrontCameraTransform = frc::Pose3d(units::meter_t(-0.3683), units::meter_t(-0.1524), units::meter_t(0.3556), frc::Rotation3d(Eigen::Vector3d{0, 1, 0}, units::angle::turn_t(0.0)));

    frc::AprilTagFieldLayout m_apriltagLayout;
};
}
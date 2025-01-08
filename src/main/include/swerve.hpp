#pragma once

#include <vector>
#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

#include <glm/gtx/string_cast.hpp>
#include <iostream>

namespace ss {

class SwerveModule {
public:
    SwerveModule(ctre::phoenix6::hardware::TalonFX& drive, ctre::phoenix6::hardware::TalonFX& turn, ctre::phoenix6::hardware::CANcoder& encoder, float encoderOffset, glm::vec2 framePos);

    void set(glm::vec2 cartesian);
    /**
     * @param speed The speed in m/s
     * @param angle The angle relative to the frame in radians
    */
    void set(float speed, float angle);

    // wheel radius in meters
    static const float wheelRadius;
    static const float motorToWheelRatio;
    static const float motorToTurnRatio;

    glm::vec2 framePosition() { return this->m_framePosition; }
private:
    /**
     * Position relative to the center of the frame in meters
    */
    glm::vec2 m_framePosition;

    /**
     * Number of loop iterations (frames) while the module isn't moving
    */
    uint32_t m_framesAtNull = 0;

    // turn offset for encoders
    float m_encoderOffset;

    // drive motor
    ctre::phoenix6::hardware::TalonFX& m_drive;
    // turn motor
    ctre::phoenix6::hardware::TalonFX& m_turn;
    // turn encoder
    ctre::phoenix6::hardware::CANcoder& m_encoder;
};

class SwerveDrive {
public:
    SwerveDrive(std::vector<SwerveModule> modules) : m_modules(modules) {}
    /**
     * @param frameVelocity Target velocity (m/s) relative to the frame (+y = forward +x = right)
    */
    void set(glm::vec2 frameVelocity, float angularVelocity);
private:
    std::vector<SwerveModule> m_modules;
};
}
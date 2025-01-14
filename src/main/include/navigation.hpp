#pragma once

#include <glm/glm.hpp>
#include <frc/joystick.h>
#include <guidance.hpp>

namespace ss {

class TeleopNavigation {
public:
    TeleopNavigation(frc::Joystick& joy, ss::Guidance& guidance) : m_joystick(joy), m_guidance(guidance) {}

    void begin_navigation();
    void update_navigation();

    glm::vec2 get_desired_drive() const { return this->m_desiredDrive; }
    float get_desired_turn() const { return this->m_desiredTurn; }
private:
    glm::vec2 m_desiredDrive = glm::vec2(0.0); // m/s
    float m_desiredTurn; // radians/s

    const float c_maxDriveSpeed = 5.0; // m/s
    const float c_minDriveSpeed = 0.5; // m/s
    const float c_maxTurnSpeed = 2.0 * M_PI; // radians/s
    const float c_minTurnSpeed = 0.5 * M_PI; // radians/s

    ss::Guidance& m_guidance;

    frc::Joystick& m_joystick;
};

class AutonNavigation {
public:
    struct PathNode {
        float duration;
            // some slightly cursed use of C features
        union {
            struct {
                glm::vec2 pos;
                float heading;
                float (*ease_fn)(float);
            } drive_to_pos;
        };
        enum class Type {
            DRIVE_TO_POSITION
        } type;
    };

    AutonNavigation(ss::Guidance& guidance, std::vector<PathNode> path) : m_guidance(guidance), m_path(path) {}

    void begin_navigation();
    void update_navigation();

    glm::vec2 get_desired_drive() const { return this->m_desiredDrive; }
    float get_desired_turn() const { return this->m_desiredTurn; }

private:
    float m_startTime;
    float m_nodeStartTime;
    std::shared_ptr<ss::Guidance::Info> m_nodeStartGuidance;
    unsigned int m_pathIndex = 0;
    std::vector<PathNode> m_path;
    glm::vec2 m_desiredDrive;
    float m_desiredTurn;

    glm::vec2 m_driveError;
    glm::vec2 m_driveErrorInt;

    float m_turnError;
    float m_turnErrorInt;

    const float drive_kP = 0.10f;
    const float drive_kI = 0.005f;
    const float drive_kD = 0.01f;

    const float turn_kP = 0.20f;
    const float turn_kI = 0.02f;
    const float turn_kD = 0.02f;

    ss::Guidance& m_guidance;
};

} // end namespace ss
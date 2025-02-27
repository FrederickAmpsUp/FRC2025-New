#pragma once

#include <glm/glm.hpp>
#include <frc/joystick.h>
#include <guidance.hpp>
#define M_PI 3.1415926535

namespace ss {

class TeleopNavigation {
public:
    TeleopNavigation(frc::Joystick& joy, ss::Guidance& guidance) : m_joystick(joy), m_guidance(guidance) {}

    void begin_navigation();
    void update_navigation();

    glm::vec2 get_desired_drive() const { return this->m_desiredDrive; }
    float get_desired_turn() const { return this->m_desiredTurn; }

    float get_desired_algae_intake_power() const { return this->m_desiredAlgaeIntakePower; }
    float get_desired_algae_intake_angle() const { return this->m_desiredAlgaeIntakeAngle; }
    float get_desired_outtake_power() const { return this->m_desiredOuttakePower; }
    float get_desired_outtake_angle() const { return this->m_desiredOuttakeAngle; }
    
    float get_desired_elevator_height() const { return this->m_desiredElevatorHeight; }
    float get_desired_capstan_angle() const { return this->m_desiredCapstanAngle; }

private:
    glm::vec2 m_desiredDrive = glm::vec2(0.0); // m/s
    float m_desiredTurn; // radians/s

    float m_desiredAlgaeIntakePower;
    float m_desiredOuttakePower;
    float m_desiredOuttakeAngle;
    // 0.0 is straight down, pi/2 is straight forward
    float m_desiredAlgaeIntakeAngle;

    float m_desiredElevatorHeight;

    float m_desiredCapstanAngle;
public:
    static constexpr float c_maxDriveSpeed = 5.0; // m/s
    static constexpr float c_minDriveSpeed = 0.5; // m/s
    static constexpr float c_maxTurnSpeed = 2.0 * M_PI; // radians/s
    static constexpr float c_minTurnSpeed = 0.5 * M_PI; // radians/s

    static constexpr float c_algaeIntakeIdle = 0.0;
    static constexpr float c_algaeIntakeLower = 1.75;
    static constexpr float c_algaeIntakeUpper = 2.15;

    // TODO: offset these for starting position
    static constexpr float c_outtakeIdle = -2.25;
    static constexpr float c_outtakeHoldAlgae = -1.5;
    static constexpr float c_outtakeReleaseAlgae = -1.6;
    static constexpr float c_outtakeIntakeAlgae = -1.2;

    static constexpr float c_outtakeIntakeCoral = 0.0;
    static constexpr float c_outtakeReleaseCoral = -1.5;

private:
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
            struct {
#define LOWER 0
#define UPPER 1
                int level;
            } clean_algae;
        };
        enum class Type {
            DRIVE_TO_POSITION,
            CLEAN_ALGAE
        } type;
    };

    AutonNavigation(ss::Guidance& guidance, std::vector<PathNode> path) : m_guidance(guidance), m_path(path) {}

    void begin_navigation();
    void update_navigation();

    glm::vec2 get_desired_drive() const { return this->m_desiredDrive; }
    float get_desired_turn() const { return this->m_desiredTurn; }

    float get_desired_algae_intake_power() const { return this->m_desiredAlgaeIntakePower; }
    float get_desired_algae_intake_angle() const { return this->m_desiredAlgaeIntakeAngle; }
    float get_desired_outtake_power() const { return 0; }
    float get_desired_outtake_angle() const { return 0; }
    float get_desired_elevator_height() const { return 0; }
    float get_desired_capstan_angle() const { return 0; }

private:
    float m_startTime;
    float m_nodeStartTime;
    std::shared_ptr<ss::Guidance::Info> m_nodeStartGuidance;
    unsigned int m_pathIndex = 0;
    std::vector<PathNode> m_path;
    glm::vec2 m_desiredDrive;
    float m_desiredTurn;
    
    float m_desiredAlgaeIntakePower;
    float m_desiredAlgaeIntakeAngle;

    glm::vec2 m_driveError;
    glm::vec2 m_driveErrorInt;

    float m_turnError;
    float m_turnErrorInt;

    const float drive_kP = 0.15f;
    const float drive_kI = 0.005f;
    const float drive_kD = 0.01f;

    const float turn_kP = 0.20f;
    const float turn_kI = 0.02f;
    const float turn_kD = 0.02f;

    ss::Guidance& m_guidance;
};

} // end namespace ss
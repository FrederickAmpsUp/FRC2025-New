#pragma once

#include <glm/glm.hpp>
#include <frc/joystick.h>
#include <guidance.hpp>
#include <frc/geometry/Pose2d.h>
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
    float get_desired_capstan_slowness() const { return this->m_desiredCapstanSlowness; }
    float get_desired_coral_intake_speed() const { return this->m_desiredCoralIntakeSpeed; }

    float get_desired_climber_power() const { return this->m_desiredClimberPower; }
private:
    glm::vec2 m_desiredDrive = glm::vec2(0.0); // m/s
    float m_desiredTurn; // radians/s

    float m_desiredAlgaeIntakePower;
    // 0.0 is straight down, pi/2 is straight forward
    float m_desiredAlgaeIntakeAngle;

    float m_desiredElevatorHeight;

    float m_desiredCapstanAngle;
    float m_desiredCoralIntakeSpeed;

    float m_desiredClimberPower;
public:
    static constexpr float c_maxDriveSpeed = 5.0; // m/s
    static constexpr float c_minDriveSpeed = 0.25; // m/s
    static constexpr float c_maxTurnSpeed = 2.0 * M_PI; // radians/s
    static constexpr float c_minTurnSpeed = 0.25 * M_PI; // radians/s

    static constexpr float c_algaeIntakeIdle = 0.0;
    static constexpr float c_algaeIntakeLower = 1.75;
    static constexpr float c_algaeIntakeUpper = 2.35;

    static constexpr float c_outtakeOffset = -2.38;

    static constexpr float c_outtakeIdle = -2.25 - c_outtakeOffset;
    static constexpr float c_outtakeHoldAlgae = -1.3 - c_outtakeOffset;
    static constexpr float c_outtakeReleaseAlgae = -1.6 - c_outtakeOffset;
    static constexpr float c_outtakeIntakeAlgae = -0.8 - c_outtakeOffset;

    static constexpr float c_outtakeIntakeCoral = 0.0 - c_outtakeOffset;
    static constexpr float c_outtakeReleaseCoral = -1.5 - c_outtakeOffset;

private:
    float m_desiredOuttakePower;
    float m_desiredOuttakeAngle = c_outtakeIdle;

    float m_desiredCapstanSlowness = 0.8;

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
            struct {
            } release_outtake_coral;
        };
        enum class Type {
            DRIVE_TO_POSITION,
            CLEAN_ALGAE,
            RELEASE_OUTTAKE_CORAL
        } type;
    };

    AutonNavigation(ss::Guidance& guidance, std::vector<PathNode> path) : m_guidance(guidance), m_path(path) {}

    void set_path(std::vector<PathNode> path) { this->m_path = path; }

    void begin_navigation();
    void update_navigation();

    glm::vec2 get_desired_drive() const { return this->m_desiredDrive; }
    float get_desired_turn() const { return this->m_desiredTurn; }

    std::vector<frc::Pose2d> where_are_we_going() const {
        glm::vec3 whereAreWeAre = glm::vec3(this->m_guidance.info()->fieldPosition, this->m_guidance.info()->fieldAngle / (2.0*M_PI));

        if (this->m_pathIndex >= this->m_path.size()) return {};

        const auto& node = this->m_path[this->m_pathIndex];

        glm::vec3 whereAreWeGoing = whereAreWeAre;

        if (node.type == PathNode::Type::DRIVE_TO_POSITION) {
            whereAreWeGoing.x = node.drive_to_pos.pos.x;
            whereAreWeGoing.y = node.drive_to_pos.pos.y;
            whereAreWeGoing.z = node.drive_to_pos.heading;
        }

        std::vector<frc::Pose2d> nodes;

        constexpr int iterations = 9;
        for (int i = 0; i < iterations; i++) {
            float t = (float)(i+1) / (float)iterations;

            glm::vec3 whereWeWillBe = glm::mix(whereAreWeAre, whereAreWeGoing, t);

            nodes.push_back(frc::Pose2d(
                units::meter_t(whereWeWillBe.y),
                -units::meter_t(whereWeWillBe.x),
                -units::turn_t(whereWeWillBe.z)
            ));
        }

        return nodes;
    }

    float get_desired_algae_intake_power() const { return this->m_desiredAlgaeIntakePower; }
    float get_desired_algae_intake_angle() const { return this->m_desiredAlgaeIntakeAngle; }
    float get_desired_outtake_power() const { return this->m_desiredOuttakePower; }
    float get_desired_outtake_angle() const { return this->m_desiredOuttakeAngle; }
    float get_desired_elevator_height() const { return 0; }
    float get_desired_capstan_angle() const { return 0; }
    float get_desired_capstan_slowness() const { return 0; }
    float get_desired_coral_intake_speed() const { return 0; }

    float get_desired_climber_power() const { return 0; }
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

    float m_desiredOuttakePower;
    float m_desiredOuttakeAngle;

    glm::dvec2 m_driveError;
    glm::dvec2 m_driveErrorInt;

    float m_turnError;
    float m_turnErrorInt;

    const float drive_kP = 0.15f;
    const float drive_kI = 0.005f;
    const float drive_kD = 0.01f;

    const float turn_kP = 0.1f;
    const float turn_kI = 0.002f;
    const float turn_kD = 0.006f;

    ss::Guidance& m_guidance;
};

} // end namespace ss
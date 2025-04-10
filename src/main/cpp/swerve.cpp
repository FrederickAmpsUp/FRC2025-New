#include <swerve.hpp>
#include <string>

using namespace ctre::phoenix6;

namespace ss {

SwerveModule::SwerveModule(hardware::TalonFX& drive, hardware::TalonFX& turn, ctre::phoenix6::hardware::CANcoder& enc, float encoderOffset, glm::vec2 framePos) : m_drive(drive), m_turn(turn), m_encoder(enc) {
    this->m_framePosition = framePos;
    this->m_encoderOffset = encoderOffset;

    turn.SetInverted(true);

    configs::Slot0Configs turnConf{};
    
    turnConf.kP = 24.0;
    turnConf.kI =  4.0;
    turnConf.kD =  0.25;
    
    configs::FeedbackConfigs turnFeedbConf{};

    turnFeedbConf.FeedbackRemoteSensorID = this->m_encoder.GetDeviceID();
    turnFeedbConf.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    turnFeedbConf.RotorToSensorRatio = motorToTurnRatio;

    configs::Slot0Configs driveConf{};

    driveConf.kP = 0.1;

    turn.GetConfigurator().Apply(turnConf);
    turn.GetConfigurator().Apply(turnFeedbConf);
    drive.GetConfigurator().Apply(driveConf);
}

void SwerveModule::set(glm::vec2 cartesian) {
    this->set(glm::length(cartesian), glm::atan(cartesian.x, cartesian.y));
}

static void optimizeAngle(float& speed, float& angle, float physicalAngle) {
    while (angle - physicalAngle < -0.5) angle += 1.0;
    while (angle - physicalAngle >  0.5) angle -= 1.0;

    glm::vec2 driveVector = glm::vec2(glm::cos(angle*2.0*M_PI), glm::sin(angle*2.0*M_PI));
    glm::vec2 physicalDriveVector = glm::vec2(glm::cos(physicalAngle*2.0*M_PI), glm::sin(physicalAngle*2.0*M_PI));

        // if the wheel is facing the wrong direction, go slower
    float ddp = glm::dot(driveVector, physicalDriveVector);
    speed *= ddp;

    if (ddp < 0.0f) {
        angle -= 0.5;
    }

    while (angle - physicalAngle < -0.5) angle += 1.0;
    while (angle - physicalAngle >  0.5) angle -= 1.0;
}

void SwerveModule::set(float speed, float angle) {
    angle = angle / (2.0 * M_PI);

    float physicalAngle = (float)this->m_turn.GetPosition().GetValue()+this->m_encoderOffset;

    if (glm::abs(speed) < 0.1) {
        speed = 0.0f;
        angle = physicalAngle;
    }
    
    optimizeAngle(speed, angle, physicalAngle);


    auto p_ctr = controls::PositionVoltage(units::angle::turn_t(angle-this->m_encoderOffset)).WithSlot(0);
    this->m_turn.SetControl(p_ctr);

    float wheelRPS = speed / (2.0 * M_PI * wheelRadius);
    auto v_ctr = controls::VelocityVoltage(units::angular_velocity::turns_per_second_t(2.0 * M_PI * wheelRPS / motorToWheelRatio)).WithSlot(0);
    this->m_drive.SetControl(v_ctr);
}

const float SwerveModule::wheelRadius = 0.0508;
const float SwerveModule::motorToWheelRatio = 1.0 / 5.01;
const float SwerveModule::motorToTurnRatio = 1.0 / 13.3714;

static glm::vec2 rotate(const glm::vec2& v, float radians) {
    return glm::vec2(
        v.x * glm::cos(radians) - v.y * glm::sin(radians),
        v.y * glm::cos(radians) + v.x * glm::sin(radians)
    );
}

static glm::vec2 projectModulePos(float t, glm::vec2 drv, float turn, glm::vec2 modPos) {
    return rotate(modPos, turn*t) + drv*t - modPos;
}

void SwerveDrive::set(glm::vec2 frameVelocity, float angularVelocity) {
    for (SwerveModule& mod : m_modules) {
        glm::vec2 pos = mod.framePosition();
        float dt = 1.0 / 20.0;
        glm::vec2 vel = projectModulePos(dt, frameVelocity, angularVelocity, pos) * (1.0f / dt);
        mod.set(vel);
    }
}

void SwerveDrive::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("SwerveDrive");

    static const char *names[] = {
        "Front Left Angle",
        "Front Left Velocity",
        
        "Front Right Angle",
        "Front Right Velocity",

        "Back Left Angle",
        "Back Left Velocity",

        "Back Right Angle",
        "Back Right Velocity",

    };

    int i = 0;
    for (const SwerveModule& mod : this->m_modules) {
        builder.AddDoubleProperty(names[i], [mod]() { return -0.5 * M_PI + (double)((units::angle::radian_t)mod.m_turn.GetPosition().GetValue()) + 2.0*M_PI*mod.m_encoderOffset; }, [](double) {});
        builder.AddDoubleProperty(names[i+1], [mod]() { return -2.0 * M_PI * (double)mod.m_drive.GetVelocity().GetValue() * SwerveModule::motorToWheelRatio * SwerveModule::wheelRadius; }, [](double) {});
        i += 2;
    }

    builder.AddDoubleProperty("Robot Angle", [this]() { return -glm::radians(this->m_navx.GetAngle()); }, [](double) {});
}
}
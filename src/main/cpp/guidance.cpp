#include <guidance.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ss {

Guidance::Guidance(ss::SwerveDrive& swerve, ctre::phoenix6::hardware::TalonFX& outtakeActuator, glm::vec2 frameStartPosition) : m_drive(swerve), m_outtakeActuator(outtakeActuator), m_navx(studica::AHRS::NavXComType::kMXP_SPI) {
    this->m_info = std::make_shared<Info>();
    this->m_info->fieldPosition = glm::vec2(0.0);
    for (const auto& mod : swerve.m_modules) {
        this->m_modulePositions.push_back(mod.m_framePosition + frameStartPosition);
    }
}

void Guidance::update_guidance() {
    this->m_info->fieldAngle = glm::radians(this->m_navx.GetAngle());
    frc::SmartDashboard::PutNumber("angle", glm::degrees(this->m_info->fieldAngle));

    unsigned int i = 0;
    glm::vec2 center;
    for (const auto& mod : this->m_drive.m_modules) {
        glm::vec2 &modPos = this->m_modulePositions[i];
        float modHeading = (float)mod.m_turn.GetPosition().GetValue() - (this->m_info->fieldAngle / (2.0*M_PI)) + mod.m_encoderOffset;
        float modSpeed = (float)mod.m_drive.GetVelocity().GetValue() * ss::SwerveModule::motorToWheelRatio * ss::SwerveModule::wheelRadius;
            // sin and cos are backwards here but it works for some reason
        glm::vec2 modVel = modSpeed * glm::vec2(glm::sin(modHeading*2.0f*M_PI), -glm::cos(modHeading*2.0f*M_PI));
        modPos += modVel * (1.0f / 20.0f);

        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " heading", modHeading * 360.0f);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos x", modPos.x);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos y", modPos.y);

        center += modPos; // assumes all modules are equidistant from the center, should work ok
        i++;
    }
    center /= (float)i;

    frc::SmartDashboard::PutNumber("robot center x", center.x);
    frc::SmartDashboard::PutNumber("robot center y", center.y);

    this->m_info->fieldPosition = center;

    this->m_info->angularVelocity = 0.0;
    this->m_info->fieldVelocity = glm::vec2(0.0);

    this->m_info->outtakeAngle = (float)this->m_outtakeActuator.GetPosition().GetValue() * 2.0 * M_PI / 108.0;
}
}
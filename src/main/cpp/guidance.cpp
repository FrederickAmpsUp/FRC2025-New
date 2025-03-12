#include <guidance.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ss {

Guidance::Guidance(ss::SwerveDrive& swerve, ctre::phoenix::motorcontrol::can::TalonSRX& outtake, glm::vec2 frameStartPosition, studica::AHRS& navx) : m_drive(swerve), m_outtake(outtake), m_navx(navx) {
    this->m_info = std::make_shared<Info>();
    this->m_info->fieldPosition = frameStartPosition;
    for (const auto& mod : swerve.m_modules) {
        this->m_modulePositions.push_back(mod.m_framePosition + frameStartPosition);
    }
}

void Guidance::update_guidance() {
    this->m_info->fieldAngle = glm::radians(this->m_navx.GetAngle());
    frc::SmartDashboard::PutNumber("angle", glm::degrees(this->m_info->fieldAngle));

    unsigned int i = 0;
    glm::dvec2 center;
    for (const auto& mod : this->m_drive.m_modules) {
        glm::dvec2 modPos = this->m_modulePositions[i];
        double modHeading = 0.25 - (double)mod.m_turn.GetPosition().GetValue() + (this->m_info->fieldAngle / (2.0*M_PI)) - mod.m_encoderOffset;
        double modSpeed = (double)mod.m_drive.GetVelocity().GetValue() * (double)ss::SwerveModule::motorToWheelRatio * (double)ss::SwerveModule::wheelRadius;

        glm::dvec2 modVel = modSpeed * glm::dvec2(glm::cos(modHeading*2.0*M_PI), glm::sin(modHeading*2.0*M_PI));
        modPos += modVel * (1.0 / 20.0);

        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " heading", modHeading * 360.0f);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos x", modPos.x);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos y", modPos.y);

        // not sure why everything has to be rotated
        center += modPos; // assumes all modules are equidistant from the center, should work ok
        i++;
    }
    center /= (double)i;

    i = 0;
    for (const auto& mod : this->m_drive.m_modules) {
        glm::dvec2 modFrameOffset = mod.m_framePosition;
        double modFrameAngle = glm::atan(modFrameOffset.y, modFrameOffset.x);
        double modFrameDistance = glm::length(modFrameOffset);

        modFrameAngle += this->m_info->fieldAngle;

        this->m_modulePositions[i] = center + modFrameDistance * glm::dvec2(glm::cos(modFrameAngle), glm::sin(modFrameAngle));
        i++;
    }

    frc::SmartDashboard::PutNumber("robot center x", center.x);
    frc::SmartDashboard::PutNumber("robot center y", center.y);

    this->m_info->fieldPosition = center;

    this->m_info->angularVelocity = 0.0;
    this->m_info->fieldVelocity = glm::vec2(0.0);

    this->m_info->outtakeAngle = -(float)this->m_outtake.GetSelectedSensorPosition() / 4096.0f * 2.0 * M_PI;

    frc::SmartDashboard::PutNumber("outtake", this->m_info->outtakeAngle);
}
}
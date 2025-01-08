#include <guidance.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ss {

Guidance::Guidance(ss::SwerveDrive& swerve) : m_drive(swerve), m_navx(studica::AHRS::NavXComType::kUSB1) {
    this->m_info = std::make_shared<Info>();
}

void Guidance::update_guidance() {
    this->m_info->fieldAngle = -glm::radians(this->m_navx.GetAngle());
    this->m_info->fieldPosition = glm::vec2(0.0);

    this->m_info->angularVelocity = 0.0;
    this->m_info->fieldVelocity = glm::vec2(0.0);

    frc::SmartDashboard::PutNumber("angle", this->m_info->fieldAngle);
}
}
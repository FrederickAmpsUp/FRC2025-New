#include <navigation.hpp>

namespace ss {

void Navigation::update_navigation() {
    std::shared_ptr<ss::Guidance::Info> guidanceInf = this->m_guidance.info();

    glm::vec2 drivePower = glm::vec2(-this->m_joystick.GetX(), this->m_joystick.GetY());
    if (glm::length(drivePower) < 0.1) drivePower = glm::vec2(0.0);

    float throttle = this->m_joystick.GetThrottle() * -0.5 + 0.5;
    
    glm::vec2 driveSpeed = drivePower * glm::mix(c_minDriveSpeed, c_maxDriveSpeed, throttle);

    float turnPower = this->m_joystick.GetTwist();
    if (glm::abs(turnPower) < 0.1) turnPower = 0.0;

    float turnSpeed = turnPower * glm::mix(c_minTurnSpeed, c_maxTurnSpeed, throttle);

    float driveMag = glm::length(driveSpeed);
    float driveAngle = glm::atan(driveSpeed.y, driveSpeed.x);
    driveAngle -= guidanceInf->fieldAngle;
    driveSpeed = driveMag * glm::vec2(glm::cos(driveAngle), glm::sin(driveAngle));

    this->m_desiredDrive = driveSpeed * 0.1f + this->m_desiredDrive * 0.9f; // TODO: fix this
    this->m_desiredTurn = turnSpeed;
}
} // end namespace ss
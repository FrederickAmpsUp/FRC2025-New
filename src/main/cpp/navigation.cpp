#include <navigation.hpp>
#include <frc/RobotController.h>

namespace ss {

void TeleopNavigation::update_navigation() {
    std::shared_ptr<ss::Guidance::Info> guidanceInf = this->m_guidance.info();

    glm::vec2 drivePower = glm::vec2(this->m_joystick.GetX(), this->m_joystick.GetY());
    if (glm::length(drivePower) < 0.1) drivePower = glm::vec2(0.0);
    if (glm::length(this->m_desiredDrive) < 0.1) this->m_desiredDrive = glm::vec2();

    float throttle = this->m_joystick.GetThrottle() * -0.5 + 0.5;
    
    glm::vec2 driveSpeed = drivePower * glm::mix(c_minDriveSpeed, c_maxDriveSpeed, throttle);

    float turnPower = this->m_joystick.GetRawAxis(4); //change me back
    if (glm::abs(turnPower) < 0.1) turnPower = 0.0;

    float turnSpeed = turnPower * glm::mix(c_minTurnSpeed, c_maxTurnSpeed, throttle);

    float driveMag = glm::length(driveSpeed);
    float driveAngle = glm::atan(driveSpeed.y, driveSpeed.x);
    driveAngle -= guidanceInf->fieldAngle;
    driveSpeed = driveMag * glm::vec2(glm::cos(driveAngle), glm::sin(driveAngle));

    this->m_desiredDrive = driveSpeed * 0.1f + this->m_desiredDrive * 0.9f; // TODO: fix this
    this->m_desiredTurn = turnSpeed;
}

void TeleopNavigation::begin_navigation() {
    // nothing here...
}

void AutonNavigation::begin_navigation() {
    this->m_pathIndex = 0;
    this->m_startTime = (float)(frc::RobotController::GetFPGATime()/1000) / 1000.0f;
    this->m_nodeStartTime = this->m_startTime;
    this->m_nodeStartGuidance = this->m_guidance.info();
}

void AutonNavigation::update_navigation() {
    if (this->m_pathIndex >= this->m_path.size()) {
        this->m_desiredDrive = glm::vec2();
        this->m_desiredTurn = 0.0f;
        return;
    }
    float currentTime = (float)(frc::RobotController::GetFPGATime()/1000) / 1000.0f;
    const PathNode& node = this->m_path[this->m_pathIndex];

    switch (node.type) {
        case PathNode::Type::DRIVE_TO_POSITION: {
            float nodeT = (currentTime - this->m_nodeStartTime) / node.duration;
            float easedNodeT = node.drive_to_pos.ease_fn? node.drive_to_pos.ease_fn(nodeT) : nodeT;
            
            glm::vec2 fieldPos = this->m_guidance.info()->fieldPosition;
            float fieldAng = this->m_guidance.info()->fieldAngle;
            glm::vec2 desiredPos = (node.drive_to_pos.pos - this->m_nodeStartGuidance->fieldPosition) * easedNodeT + this->m_nodeStartGuidance->fieldPosition;
            float desiredAng = node.drive_to_pos.heading;

            frc::SmartDashboard::PutNumber("target pos x", desiredPos.x);
            frc::SmartDashboard::PutNumber("target pos y", desiredPos.y);

            glm::vec2 delPos = desiredPos - fieldPos;
            float delAng = desiredAng - fieldAng;

            frc::SmartDashboard::PutNumber("del pos x", delPos.x);
            frc::SmartDashboard::PutNumber("del pos y", delPos.y);
            frc::SmartDashboard::PutNumber("del ang", delAng);

            float driveAngle = glm::atan(delPos.y, delPos.x);
            driveAngle -= this->m_guidance.info()->fieldAngle;
            delPos = glm::vec2(glm::cos(driveAngle), glm::sin(driveAngle)) * glm::length(delPos);

            this->m_desiredDrive = glm::vec2(delPos.x, -delPos.y) * 10.0f; // i believe this should actually be 20 but 20 makes it go way too fast
            this->m_desiredTurn = delAng * 10.0f;
        } break;
        default:
            std::cerr << "ERR: invalid auto path node!" << std::endl;
    }

    if (currentTime - this->m_nodeStartTime > node.duration) {
        this->m_desiredDrive = glm::vec2();
        this->m_desiredTurn = 0.0f;
        this->m_nodeStartTime = currentTime;
        this->m_pathIndex++;
            // copy
        this->m_nodeStartGuidance = std::make_shared<ss::Guidance::Info>(*this->m_guidance.info());
    }
}

} // end namespace ss
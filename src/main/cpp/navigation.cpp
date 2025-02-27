#include <navigation.hpp>
#include <frc/RobotController.h>

namespace ss {

void TeleopNavigation::update_navigation() {
    std::shared_ptr<ss::Guidance::Info> guidanceInf = this->m_guidance.info();

    glm::vec2 drivePower = glm::vec2(this->m_joystick.GetX(), this->m_joystick.GetY());
    if (glm::length(drivePower) < 0.05) drivePower = glm::vec2(0.0);

    float throttle = this->m_joystick.GetThrottle() * -0.5 + 0.5;
    
    glm::vec2 driveSpeed = drivePower * glm::mix(c_minDriveSpeed, c_maxDriveSpeed, throttle);

    float turnPower = this->m_joystick.GetTwist();
    if (glm::abs(turnPower) < 0.1) turnPower = 0.0;

    float turnSpeed = turnPower * glm::mix(c_minTurnSpeed, c_maxTurnSpeed, throttle);

    float driveMag = glm::length(driveSpeed);
    float driveAngle = glm::atan(driveSpeed.y, driveSpeed.x);
    driveAngle -= guidanceInf->fieldAngle;
    driveSpeed = driveMag * glm::vec2(glm::cos(driveAngle), glm::sin(driveAngle));

    this->m_desiredDrive = driveSpeed;
    this->m_desiredTurn = turnSpeed;

    if (this->m_joystick.GetRawButton(4)) {
        this->m_desiredAlgaeIntakePower = 0.5;
        this->m_desiredAlgaeIntakeAngle = c_algaeIntakeLower;
        this->m_desiredOuttakePower = -0.3;
        this->m_desiredOuttakeAngle = c_outtakeHoldAlgae;
    } else if (this->m_joystick.GetRawButton(6)) {
        this->m_desiredAlgaeIntakePower = -0.5;
        this->m_desiredAlgaeIntakeAngle = c_algaeIntakeUpper;
        this->m_desiredOuttakePower = -0.3;
        this->m_desiredOuttakeAngle = c_outtakeHoldAlgae;
    }

    if (this->m_joystick.GetRawButton(2)) {
        this->m_desiredAlgaeIntakeAngle = c_algaeIntakeIdle;
        this->m_desiredAlgaeIntakePower = 0.0;
        this->m_desiredOuttakePower = 0;
        this->m_desiredOuttakeAngle = c_outtakeHoldAlgae;
    }

    if (this->m_joystick.GetRawButton(1)) {
        this->m_desiredAlgaeIntakeAngle = c_algaeIntakeIdle;
        this->m_desiredAlgaeIntakePower = 0.0;
        this->m_desiredOuttakeAngle = c_outtakeReleaseAlgae;
        this->m_desiredOuttakePower = -0.5;
    }

    if (this->m_joystick.GetRawButtonReleased(1)) {
        this->m_desiredAlgaeIntakeAngle = c_algaeIntakeIdle;
        this->m_desiredAlgaeIntakePower = 0.0;
        this->m_desiredOuttakePower = 0;
        this->m_desiredOuttakeAngle = c_outtakeIdle;
    }

    if (this->m_joystick.GetRawButton(3)) {
        this->m_desiredOuttakePower = 0.4;
        this->m_desiredOuttakeAngle = c_outtakeIntakeAlgae;
    }

    if (this->m_joystick.GetRawButtonReleased(3)) {
        this->m_desiredOuttakePower = 0.0;
        this->m_desiredOuttakeAngle = c_outtakeHoldAlgae;
    }

    if (this->m_joystick.GetRawButton(9)) {
        this->m_desiredOuttakePower = 1.0;
        this->m_desiredOuttakeAngle = glm::mix(c_outtakeIntakeCoral, guidanceInf->outtakeAngle, 0.8);
    }
    if (this->m_joystick.GetRawButtonReleased(9)) {
        this->m_desiredOuttakePower = 0.0;
        this->m_desiredOuttakeAngle = c_outtakeIdle;
    }

        if (this->m_joystick.GetRawButton(10)) {
        this->m_desiredOuttakePower = -1.0;
        this->m_desiredOuttakeAngle = c_outtakeReleaseCoral;
    }
    if (this->m_joystick.GetRawButtonReleased(10)) {
        this->m_desiredOuttakePower = 0.0;
        this->m_desiredOuttakeAngle = c_outtakeIdle;
    }

    unsigned int elevatorPos = this->m_joystick.GetRawButton(7) | (this->m_joystick.GetRawButton(8) << 1);
    static const float elevatorPositions[] = {
        0.0, 0.0, 0.45, 0.3 // TODO: determine the real positions
    };

    static const float capstanPositions[] = {
        0.0, 0.33, 0.33, 0.125 // TODO: determine real positions
    };

    this->m_desiredCapstanAngle = capstanPositions[elevatorPos];
    this->m_desiredElevatorHeight = elevatorPositions[elevatorPos];
}

void TeleopNavigation::begin_navigation() {
    // nothing here...
}

void AutonNavigation::begin_navigation() {
    this->m_pathIndex = 0;
    this->m_startTime = (float)(frc::RobotController::GetFPGATime()/1000) / 1000.0f;
    this->m_nodeStartTime = this->m_startTime;
    this->m_nodeStartGuidance = this->m_guidance.info();
    this->m_driveErrorInt = glm::vec2(0);
    this->m_driveError = glm::vec2(0);
}

void AutonNavigation::update_navigation() {
    if (this->m_pathIndex >= this->m_path.size()) {
        this->m_desiredDrive = glm::vec2();
        this->m_desiredTurn = 0.0f;
        this->m_driveErrorInt = glm::vec2(0);
        this->m_driveError = glm::vec2(0);
        this->m_turnError = 0.0f;
        this->m_turnErrorInt = 0.0f;
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
            float desiredAng = (node.drive_to_pos.heading*2.0*M_PI - this->m_nodeStartGuidance->fieldAngle) * easedNodeT + this->m_nodeStartGuidance->fieldAngle;

            frc::SmartDashboard::PutNumber("target pos x", desiredPos.x);
            frc::SmartDashboard::PutNumber("target pos y", desiredPos.y);

            glm::vec2 delPos = desiredPos - fieldPos;
            float delAng = desiredAng - fieldAng;
            while (delAng > 1.0) delAng -= 1.0;
            while (delAng < 0.0) delAng += 1.0;

            frc::SmartDashboard::PutNumber("del pos x", delPos.x);
            frc::SmartDashboard::PutNumber("del pos y", delPos.y);
            frc::SmartDashboard::PutNumber("del ang", delAng);

            glm::vec2 driveError = delPos * 20.0f;

            glm::vec2 dedt = (driveError - this->m_driveError) * 20.0f;
            this->m_driveErrorInt += driveError / 20.0f;
            this->m_driveError = driveError;

            float turnError = delAng * 20.0f;
            float detdt = (turnError - this->m_turnError) * 20.0f;
            this->m_turnErrorInt += turnError / 20.0f;
            this->m_turnError = turnError;

            this->m_desiredDrive = driveError * drive_kP + this->m_driveErrorInt * drive_kI + dedt * drive_kD;
            
            float driveAngle = glm::atan(this->m_desiredDrive.y, this->m_desiredDrive.x);
            driveAngle += this->m_guidance.info()->fieldAngle;
            this->m_desiredDrive = glm::vec2(glm::cos(driveAngle), -glm::sin(driveAngle)) * glm::length(this->m_desiredDrive);

            this->m_desiredTurn = turnError * turn_kP + this->m_turnErrorInt * turn_kI + detdt * turn_kD;
        } break;
        case PathNode::Type::CLEAN_ALGAE: {
            if (node.clean_algae.level == LOWER) {
                this->m_desiredAlgaeIntakeAngle = TeleopNavigation::c_algaeIntakeLower;
                this->m_desiredAlgaeIntakePower = 0.5;
            } else {
                this->m_desiredAlgaeIntakeAngle = TeleopNavigation::c_algaeIntakeUpper;
                this->m_desiredAlgaeIntakePower = -0.5;
            }
        } break;
        default:
            std::cerr << "ERR: invalid auto path node!" << std::endl;
    }

    if (currentTime - this->m_nodeStartTime > node.duration) {
        this->m_desiredDrive = glm::vec2();
        this->m_desiredTurn = 0.0f;
        this->m_desiredAlgaeIntakeAngle = 0.0f;
        this->m_desiredAlgaeIntakePower = 0.0f;
        this->m_nodeStartTime = currentTime;
        this->m_pathIndex++;

        this->m_driveErrorInt = glm::vec2(0);
        this->m_driveError = glm::vec2(0);
        this->m_turnError = 0.0f;
        this->m_turnErrorInt = 0.0f;
            // copy (i hope)
        this->m_nodeStartGuidance = std::make_shared<ss::Guidance::Info>(*this->m_guidance.info());
    }
}

} // end namespace ss
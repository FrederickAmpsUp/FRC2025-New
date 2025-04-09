#include <guidance.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photon/PhotonUtils.h>

namespace ss {

Guidance::Guidance(ss::SwerveDrive& swerve, ctre::phoenix::motorcontrol::can::TalonSRX& outtake, studica::AHRS& navx) : m_drive(swerve), m_outtake(outtake), m_navx(navx), m_frontCamera("Front"), m_apriltagLayout("/home/lvuser/deploy/reefscape-apriltags.json") {
    this->m_info = std::make_shared<Info>();
    this->m_info->fieldPosition = glm::vec2(0.0);
    for (const auto& mod : swerve.m_modules) {
        this->m_modulePositions.push_back(mod.m_framePosition);
    }
}

void Guidance::update_guidance() {
    this->m_info->fieldAngle = glm::radians(this->m_navx.GetAngle());
    frc::SmartDashboard::PutNumber("navx angle", glm::degrees(this->m_info->fieldAngle));

    unsigned int i = 0;
    glm::dvec2 center;
    for (const auto& mod : this->m_drive.m_modules) {
        glm::dvec2 modPos = this->m_modulePositions[i];
        double modHeading = 0.25 - (double)mod.m_turn.GetPosition().GetValue() + (this->m_info->fieldAngle / (2.0*M_PI)) - mod.m_encoderOffset;
        double modSpeed = (double)mod.m_drive.GetVelocity().GetValue() * (double)ss::SwerveModule::motorToWheelRatio * (double)ss::SwerveModule::wheelRadius;

        glm::dvec2 modVel = modSpeed * glm::dvec2(glm::cos(-modHeading*2.0*M_PI), glm::sin(-modHeading*2.0*M_PI));
        modPos += modVel * (1.0 / 20.0);

        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " heading", modHeading * 360.0f);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos x", modPos.x);
        frc::SmartDashboard::PutNumber("module " + std::to_string(i) + " pos y", modPos.y);

        // not sure why everything has to be rotated
        center += modPos; // assumes all modules are equidistant from the center, should work ok
        i++;
    }
    center /= (double)i;

    photon::PhotonPipelineResult frontCamResult = this->m_frontCamera.GetLatestResult();
    // TODO: latency checking

    frc::Transform2d apriltagPose;
    bool goodApriltag = false;

        // fallback to single-tag estimation if multi-tag isn't available
    if (!frontCamResult.MultiTagResult().has_value()) {
        if (frontCamResult.HasTargets()) {
            auto tag = frontCamResult.GetBestTarget();

            if (this->m_apriltagLayout.GetTagPose(tag.fiducialId).has_value()) {
                frc::Transform3d cameraToTarget = tag.GetBestCameraToTarget();
                frc::Transform2d cameraToTarget2d = frc::Transform2d(cameraToTarget.X(), cameraToTarget.Y(), frc::Rotation2d(cameraToTarget.Rotation().ToRotation2d()));

                frc::Pose2d robotPose = photon::PhotonUtils::EstimateFieldToRobot(cameraToTarget2d, this->m_apriltagLayout.GetTagPose(tag.fiducialId).value().ToPose2d(), frc::Transform2d(this->m_frameToFrontCameraTransform.ToPose2d().ToMatrix()));
            
                apriltagPose = frc::Transform2d(robotPose.ToMatrix());
                goodApriltag = true;
            }
        }
    } else {
        apriltagPose = frc::Transform2d(frc::Pose3d(frontCamResult.MultiTagResult().value().estimatedPose.best.ToMatrix()).ToPose2d().ToMatrix());
        goodApriltag = true;
    }

    if (goodApriltag) {
        this->m_info->fieldPosition = glm::mix(glm::dvec2(-apriltagPose.Y(), apriltagPose.X()), center, 0.9);
        
        float apriltagAngle = -(float)apriltagPose.Rotation().Radians();
        if (abs(apriltagAngle - this->m_info->fieldAngle) > glm::radians(5.0)) {
            //this->m_navx.Reset();
            //this->m_navx.SetAngleAdjustment(glm::degrees(apriltagAngle));
        }
        //this->m_info->fieldAngle = apriltagAngle;
    } else {
        this->m_info->fieldPosition = center;
    }

    i = 0;
    for (const auto& mod : this->m_drive.m_modules) {
        glm::dvec2 modFrameOffset = mod.m_framePosition;
        double modFrameAngle = glm::atan(modFrameOffset.y, modFrameOffset.x);
        double modFrameDistance = glm::length(modFrameOffset);

        modFrameAngle += this->m_info->fieldAngle;

        this->m_modulePositions[i] = this->m_info->fieldPosition + modFrameDistance * glm::dvec2(glm::cos(modFrameAngle), glm::sin(modFrameAngle));
        i++;
    }

    frc::SmartDashboard::PutNumber("robot center x", center.x);
    frc::SmartDashboard::PutNumber("robot center y", center.y);
    frc::SmartDashboard::PutNumber("angle", glm::degrees(this->m_info->fieldAngle));

    //this->m_info->fieldPosition = center;

    this->m_info->angularVelocity = 0.0;
    this->m_info->fieldVelocity = glm::vec2(0.0);

    this->m_info->outtakeAngle = -(float)this->m_outtake.GetSelectedSensorPosition() / 4096.0f * 2.0 * M_PI;

    frc::SmartDashboard::PutNumber("outtake", this->m_info->outtakeAngle);
}
}
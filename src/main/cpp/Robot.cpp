// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <glm/gtx/string_cast.hpp>
#include <cameraserver/CameraServer.h>
#include <iostream>
#include <fstream>
#include <frc/Filesystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/button/NetworkButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc/DriverStation.h>
#include <elasticlib.h>

static float smoothstep(float x) {
  return 3.0f*x*x - 2.0f*x*x*x;
}

void Robot::RobotInit() {
  //frc::CameraServer::StartAutomaticCapture();

  elastic::SelectTab("Autonomous");

  this->frd = new ctre::phoenix6::hardware::TalonFX(0b000 + 1);
  this->frt = new ctre::phoenix6::hardware::TalonFX(0b001 + 1);
  this->fld = new ctre::phoenix6::hardware::TalonFX(0b010 + 1);
  this->flt = new ctre::phoenix6::hardware::TalonFX(0b011 + 1);
  this->brd = new ctre::phoenix6::hardware::TalonFX(0b100 + 1);
  this->brt = new ctre::phoenix6::hardware::TalonFX(0b101 + 1);
  this->bld = new ctre::phoenix6::hardware::TalonFX(0b110 + 1);
  this->blt = new ctre::phoenix6::hardware::TalonFX(0b111 + 1);

  this->fre = new ctre::phoenix6::hardware::CANcoder( 9);
  this->fle = new ctre::phoenix6::hardware::CANcoder(10);
  this->bre = new ctre::phoenix6::hardware::CANcoder(11);
  this->ble = new ctre::phoenix6::hardware::CANcoder(12);

  this->orchestra = new ctre::phoenix6::Orchestra();
  this->orchestra->AddInstrument(*this->fld);
  this->orchestra->AddInstrument(*this->flt);
  this->orchestra->AddInstrument(*this->frd);
  this->orchestra->AddInstrument(*this->frt);
  this->orchestra->AddInstrument(*this->bld);
  this->orchestra->AddInstrument(*this->blt);
  this->orchestra->AddInstrument(*this->brd);
  this->orchestra->AddInstrument(*this->brt);

  this->orchestra->LoadMusic("thunderstruck.chrp");

  constexpr float frameHalfWidth = 0.5588f / 2.0f;
  constexpr float frameHalfDepth = 0.5588f / 2.0f;

  this->fl = new ss::SwerveModule(*this->fld, *this->flt, *this->fle, -0.25, glm::vec2(-frameHalfWidth, -frameHalfDepth));
  this->fr = new ss::SwerveModule(*this->frd, *this->frt, *this->fre, -0.07, glm::vec2( frameHalfWidth, -frameHalfDepth));
  this->bl = new ss::SwerveModule(*this->bld, *this->blt, *this->ble, -0.19, glm::vec2(-frameHalfWidth, frameHalfDepth));
  this->br = new ss::SwerveModule(*this->brd, *this->brt, *this->bre,  0.02, glm::vec2( frameHalfWidth, frameHalfDepth));

  this->drive = new ss::SwerveDrive({*this->bl, *this->br, *this->fl, *this->fr}, this->navx);

  this->outtake = new ctre::phoenix::motorcontrol::can::TalonSRX(17);

  this->autoChooser.AddOption("test auto 1", {
    glm::vec2(0.0, 1.0),
    {
      {
        .duration = 2.5,
        .drive_to_pos = {
          .pos = glm::vec2(0.0),
          .heading = 0.0,
          .ease_fn = smoothstep
        },
        .type= ss::AutonNavigation::PathNode::Type::DRIVE_TO_POSITION
      }
    }
  });

  this->autoChooser.AddOption("test auto 2", {
    glm::vec2(-1.0, 0.0),
    {}
  });

  this->autoChooser.AddOption("right side coral -> l1", {
    glm::vec2(-0.515, 7.110),
    {
      ss::AutonNavigation::PathNode {
        .duration = 2.07,
        .drive_to_pos = {
          .pos = glm::vec2(-2.858, 5.175),
          .heading = 0.159,
          .ease_fn = smoothstep
        },
        .type = ss::AutonNavigation::PathNode::Type::DRIVE_TO_POSITION
      },
      ss::AutonNavigation::PathNode {
        .duration = 2.0,
        .type = ss::AutonNavigation::PathNode::Type::RELEASE_OUTTAKE_CORAL
      }
    }
  });

  this->autoChooser.AddOption("left side coral -> l1", {
    glm::vec2(-7.525, 7.110),
    {
      ss::AutonNavigation::PathNode {
        .duration = 2.07,
        .drive_to_pos = {
          .pos = glm::vec2(-5.212, 5.175),
          .heading = -0.105,
          .ease_fn = smoothstep
        },
        .type = ss::AutonNavigation::PathNode::Type::DRIVE_TO_POSITION
      },
      ss::AutonNavigation::PathNode {
        .duration = 2.0,
        .type = ss::AutonNavigation::PathNode::Type::RELEASE_OUTTAKE_CORAL
      }
    }
  });

  this->autoChooser.AddOption("center coral -> l1", {
    glm::vec2(-4.000, 7.110),
    {
      ss::AutonNavigation::PathNode {
        .duration = 2.07,
        .drive_to_pos = {
          .pos = glm::vec2(-4.000, 5.833),
          .heading = 0.0,
          .ease_fn = smoothstep
        },
        .type = ss::AutonNavigation::PathNode::Type::DRIVE_TO_POSITION
      },
      ss::AutonNavigation::PathNode {
        .duration = 2.0,
        .type = ss::AutonNavigation::PathNode::Type::RELEASE_OUTTAKE_CORAL
      }
    }
  });

  this->autoChooser.AddOption("dummy coral", {
    glm::vec2(0.0),
    {
      ss::AutonNavigation::PathNode {
        .duration = 2.0,
        .type = ss::AutonNavigation::PathNode::Type::RELEASE_OUTTAKE_CORAL
      }
    }
  });

  this->autoChooser.AddOption("do nothing (start right wall)", {
      glm::vec2(-0.515, 7.110),
      { /* nothing */ }
  });

  this->autoChooser.AddOption("do nothing (start left wall)", {
      glm::vec2(-7.525, 7.110),
      { /* nothing */ }
  });
  
  this->autoChooser.AddOption("do nothing (start center)", {
      glm::vec2(-4.000, 7.110),
      { /* nothing */ }
  });

  this->autoChooser.AddOption("leave", {
      glm::vec2(0),
      {
        ss::AutonNavigation::PathNode {
          .duration = 13,
          .drive_to_pos = {
            .pos = glm::vec2(0, -1),
            .heading = 0.0,
            .ease_fn = smoothstep
          },
          .type = ss::AutonNavigation::PathNode::Type::DRIVE_TO_POSITION
        },
      }
  });

  this->guidance = new ss::Guidance(*this->drive, *this->outtake, this->navx);
  this->tnav = new ss::TeleopNavigation(this->joystick, *this->guidance);
  using PathNode = ss::AutonNavigation::PathNode;
  this->anav = new ss::AutonNavigation(*this->guidance, {});
  this->control = new ss::Control(*this->drive, *this->outtake);

  frc::SmartDashboard::PutBoolean("NAVX/Reset", false);
  frc::SmartDashboard::PutString("NAVX/IMORTANT", "Make sure robot is facing away from DS!");

  frc::SmartDashboard::PutBoolean("Capstan/Reset", false);
}
void Robot::RobotPeriodic() {
  this->guidance->update_guidance();
  frc::SmartDashboard::PutData("swerve", this->drive);

  this->fieldTelem.SetRobotPose(frc::Pose2d(units::meter_t(this->guidance->info()->fieldPosition.y), units::meter_t(-this->guidance->info()->fieldPosition.x), frc::Rotation2d(units::radian_t(-this->guidance->info()->fieldAngle))));

  frc::FieldObject2d *whereAreWeGoing = this->fieldTelem.GetObject("where going?");
  frc::FieldObject2d *whereWeWillFinish = this->fieldTelem.GetObject("where we will finish");

  auto path = this->anav->where_are_we_going();

  whereAreWeGoing->SetPoses(path);
  if (path.size() > 0) {
    whereWeWillFinish->SetPose(path[path.size() - 1]);
  }

  frc::SmartDashboard::PutData("field", &this->fieldTelem);

  frc::SmartDashboard::PutData("auton", &this->autoChooser);

  if (frc::SmartDashboard::GetBoolean("NAVX/Reset", false)) {
    this->navx.Reset();
    this->navx.SetAngleAdjustment(0);
    frc::SmartDashboard::PutBoolean("NAVX/Reset", false);
  }

  if (frc::SmartDashboard::GetBoolean("Capstan/Reset", false)) {
    this->control->m_capstanHomingStep = ss::Control::CapstanHomingStep::STOPPED;
    frc::SmartDashboard::PutBoolean("Capstan/Reset", false);
  }

  units::second_t matchTime = frc::DriverStation::GetMatchTime();
  frc::SmartDashboard::PutNumber("Match Time", (double)matchTime);

  auto voltage = frc::RobotController::GetBatteryVoltage();
  frc::SmartDashboard::PutNumber("Battery Voltage", (double)voltage);

  auto alliance = frc::DriverStation::GetAlliance();

  std::string hexColor = "#000000";

  if (alliance.has_value()) {
    if (*alliance == frc::DriverStation::kRed) {
      hexColor = "#FF0000";
    } else {
      hexColor = "#0000FF";
    }
  }

  frc::SmartDashboard::PutString("Alliance Color", hexColor);
}

void Robot::AutonomousInit() {
  elastic::SelectTab("Autonomous");
  const auto& auton = this->autoChooser.GetSelected();
  this->guidance->set_field_position(auton.first);
  this->anav->set_path(auton.second);
  this->anav->begin_navigation();
}
void Robot::AutonomousPeriodic() {
  this->guidance->update_guidance();
  this->anav->update_navigation();
  frc::SmartDashboard::PutNumber("desired drive x", this->anav->get_desired_drive().x);
  frc::SmartDashboard::PutNumber("desired drive y", this->anav->get_desired_drive().y);
  this->control->update_control(*this->anav);
}

void Robot::TeleopInit() {
  elastic::SelectTab("Teleoperated");
  this->orchestra->Stop();
  this->tnav->begin_navigation();
  this->anav->begin_navigation();
}
void Robot::TeleopPeriodic() {
  this->tnav->update_navigation();
  this->anav->update_navigation();
  frc::SmartDashboard::PutNumber("desired drive x", this->anav->get_desired_drive().x);
  frc::SmartDashboard::PutNumber("desired drive y", this->anav->get_desired_drive().y);
  this->control->update_control(*this->tnav);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {
  const auto& auton = this->autoChooser.GetSelected();
  this->guidance->set_field_position(auton.first);
  this->anav->set_path(auton.second);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

Robot::~Robot() {
  delete this->fl;
  delete this->fr;
  delete this->bl;
  delete this->br;

  delete this->fld;
  delete this->flt;
  delete this->frd;
  delete this->frt;
  delete this->bld;
  delete this->blt;
  delete this->brd;
  delete this->brt;

  delete this->outtake;

  delete this->orchestra;

  delete this->drive;

  delete this->guidance;
  delete this->tnav;
  delete this->anav;
  delete this->control;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

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

static float smoothstep(float x) {
  return 3.0f*x*x - 2.0f*x*x*x;
}

void Robot::RobotInit() {
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
    {}
  });

  this->autoChooser.AddOption("test auto 2", {
    glm::vec2(-1.0, 0.0),
    {}
  });

  this->guidance = new ss::Guidance(*this->drive, *this->outtake, glm::vec2(0.0), this->navx);
  this->tnav = new ss::TeleopNavigation(this->joystick, *this->guidance);
  using PathNode = ss::AutonNavigation::PathNode;
  this->anav = new ss::AutonNavigation(*this->guidance, {});
  this->control = new ss::Control(*this->drive, *this->outtake);
}
void Robot::RobotPeriodic() {
  this->guidance->update_guidance();
  frc::SmartDashboard::PutData("swerve", this->drive);

  this->fieldTelem.SetRobotPose(frc::Pose2d(units::meter_t(-this->guidance->info()->fieldPosition.y), units::meter_t(-this->guidance->info()->fieldPosition.x), frc::Rotation2d(units::radian_t(-this->guidance->info()->fieldAngle))));

  frc::SmartDashboard::PutData("field", &this->fieldTelem);

  frc::SmartDashboard::PutData("auton", &this->autoChooser);
}

void Robot::AutonomousInit() {
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

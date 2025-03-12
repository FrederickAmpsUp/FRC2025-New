// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <ctre/phoenix6/Orchestra.hpp>
#include <swerve.hpp>
#include <iostream>
#include <navigation.hpp>
#include <control.hpp>
#include <guidance.hpp>
#include <frc/shuffleboard/Shuffleboard.h>
#include <studica/AHRS.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <tuple>

class Robot : public frc::TimedRobot {
public:
    Robot() : joystick(0), navx(studica::AHRS::NavXComType::kMXP_SPI) {}

    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

    ~Robot();
private:
    ss::SwerveModule *fl, *fr, *bl, *br;
    ctre::phoenix6::hardware::TalonFX *fld, *flt, *frd, *frt, *bld, *blt, *brd, *brt;
    ctre::phoenix6::hardware::CANcoder *fle, *fre, *ble, *bre;
    ctre::phoenix::motorcontrol::can::TalonSRX *outtake;
    ctre::phoenix6::Orchestra *orchestra;
    ss::SwerveDrive *drive;
    frc::Joystick joystick;

    studica::AHRS navx;

    frc::Field2d fieldTelem;

    frc::SendableChooser<std::pair<glm::vec2, std::vector<ss::AutonNavigation::PathNode>>> autoChooser;

    ss::Control *control;
    ss::TeleopNavigation *tnav;
    ss::AutonNavigation *anav;
    ss::Guidance *guidance;
};

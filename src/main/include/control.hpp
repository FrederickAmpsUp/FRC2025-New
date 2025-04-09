#pragma once

#include <swerve.hpp>
#include <navigation.hpp>
#include <rev/SparkMax.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <frc/Servo.h>
#define M_PI 3.1415926535

namespace ss {

class Control {
public:
    Control(ss::SwerveDrive& swerve, ctre::phoenix::motorcontrol::can::TalonSRX& outtake) : m_swerve(swerve), m_algaeIntakeWheel(13, rev::spark::SparkLowLevel::MotorType::kBrushless), m_outtakeWheel(14, rev::spark::SparkLowLevel::MotorType::kBrushless), m_algaeIntakeActuator(15), m_elevator(16, rev::spark::SparkLowLevel::MotorType::kBrushless), m_outtakeActuator(outtake), m_capstan(18), m_climber(19), m_coralIntake(0) {
        std::cout << "control init" << std::endl;
        
        this->m_algaeIntakeActuator.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
    
        this->m_algaeIntakeActuator.Config_kP(0, 1.0);
        this->m_algaeIntakeActuator.Config_kI(0, 0.0);
        this->m_algaeIntakeActuator.Config_kD(0, 0.5);

        this->m_algaeIntakeActuator.SetSelectedSensorPosition(0);
        this->m_algaeIntakeActuator.SetSensorPhase(true);

        rev::spark::SparkBaseConfig config = {};

        config.closedLoop.P(0.1);
        config.closedLoop.I(0.0);
        config.closedLoop.D(0.0);

        config.Inverted(false);

        this->m_elevator.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters);

        this->m_outtakeActuator.Config_kP(0, 0.7);
        this->m_outtakeActuator.Config_kI(0, 0.0);
        this->m_outtakeActuator.Config_kD(0, 0.1);

        this->m_outtakeActuator.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0);
        this->m_outtakeActuator.SetSensorPhase(false);

        ctre::phoenix6::configs::Slot0Configs capConf = {};

        capConf.kP = 5.0;
        capConf.kI = 0.5;
        capConf.kD = 0.1;

        this->m_capstan.GetConfigurator().Apply(capConf);
    }

    template<typename Nav>
    void update_control(const Nav& nav) {
        glm::vec2 drive = nav.get_desired_drive();
        float turn = nav.get_desired_turn();

        this->m_swerve.set(drive, turn);

        this->m_algaeIntakeWheel.Set(nav.get_desired_algae_intake_power());
        this->m_outtakeWheel.Set(nav.get_desired_outtake_power());

        float intakeAngle = (nav.get_desired_algae_intake_angle() / (2.0*M_PI) + c_algaeIntakeActuatorOffset) * 4096;
        float intakePosition = this->m_algaeIntakeActuator.GetSelectedSensorPosition();

        if (intakePosition > 2048) intakeAngle += 4096;

        this->m_algaeIntakeActuator.Set(ctre::phoenix::motorcontrol::ControlMode::Position, intakeAngle);
        frc::SmartDashboard::PutNumber("alg pos", this->m_algaeIntakeActuator.GetSelectedSensorPosition());
        frc::SmartDashboard::PutNumber("alg pos setp", (nav.get_desired_algae_intake_angle() / (2.0*M_PI) + c_algaeIntakeActuatorOffset) * 4096);

        this->m_elevator.GetClosedLoopController().SetReference(nav.get_desired_elevator_height() * c_elevRevsPerM, rev::spark::SparkLowLevel::ControlType::kPosition);
        float elevPos = this->m_elevator.GetEncoder().GetPosition() / c_elevRevsPerM;
        float elevVel = this->m_elevator.GetEncoder().GetVelocity() / 60.0f / c_elevRevsPerM;

        float elevTimeLeft = 0.0f;
        if (fabsf(elevPos - nav.get_desired_elevator_height()) > 0.01) {
            elevTimeLeft = (nav.get_desired_elevator_height() - elevPos) / elevVel;
        }

        this->m_outtakeActuator.Set(ctre::phoenix::motorcontrol::ControlMode::Position, nav.get_desired_outtake_angle() / (2.0 * M_PI) * -4096);

        if (this->m_capstanHomingStep == CapstanHomingStep::STOPPED) {
            this->m_capstanHomingStep = CapstanHomingStep::RUN_POSITIVE;
            this->m_capstanIterationsStopped = 0;
        } else if (this->m_capstanHomingStep == CapstanHomingStep::RUN_POSITIVE) {
            auto capCtr = ctre::phoenix6::controls::DutyCycleOut(0.05);
            this->m_capstan.SetControl(capCtr);

            if (fabsf((float)this->m_capstan.GetVelocity().GetValue()) < 0.5) {
                this->m_capstanIterationsStopped++;
            }

            if (this->m_capstanIterationsStopped > 3) {
                this->m_capstanIterationsStopped = 0;
                this->m_capstanHomingStep = CapstanHomingStep::RUN_NEGATIVE;

                this->m_capstanMaxPos = (float)this->m_capstan.GetPosition().GetValue();
            }
        } else if (this->m_capstanHomingStep == CapstanHomingStep::RUN_NEGATIVE) {
            auto capCtr = ctre::phoenix6::controls::DutyCycleOut(-0.05);
            this->m_capstan.SetControl(capCtr);

            if (fabsf((float)this->m_capstan.GetVelocity().GetValue()) < 0.5) {
                this->m_capstanIterationsStopped++;
            }

            if (this->m_capstanIterationsStopped > 3) {
                auto capCtr = ctre::phoenix6::controls::DutyCycleOut(0.0);
                this->m_capstan.SetControl(capCtr);
                this->m_capstanIterationsStopped = 0;
                this->m_capstanHomingStep = CapstanHomingStep::DONE;

                this->m_capstanMinPos = (float)this->m_capstan.GetPosition().GetValue();

                std::cout << "Capstan homing complete, min: " << this->m_capstanMinPos << ", max: " << this->m_capstanMaxPos << std::endl;
            }
        } else if (this->m_capstanHomingStep == CapstanHomingStep::DONE) {
            float targ = glm::mix(this->m_capstanMinPos, this->m_capstanMaxPos, glm::clamp(nav.get_desired_capstan_angle() * 3.0f, 0.0f, 1.0f));
            float curr = (float)this->m_capstan.GetPosition().GetValue();

            float itersLeft = elevTimeLeft * 20.0f;

            float lerpPos = glm::mix(curr, targ, 1.0 / itersLeft);

            if (elevTimeLeft < 0.1f) {
                auto capCtr = ctre::phoenix6::controls::PositionVoltage(units::angle::turn_t(glm::mix(targ, curr, nav.get_desired_capstan_slowness())));
                this->m_capstan.SetControl(capCtr);
            } else {
                auto capCtr = ctre::phoenix6::controls::PositionVoltage(units::angle::turn_t(lerpPos));
                this->m_capstan.SetControl(capCtr);
            }

            this->m_climber.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, nav.get_desired_climber_power());
        }

        this->m_coralIntake.Set(nav.get_desired_coral_intake_speed() * 0.5 + 0.5);
    }

    ~Control() {
        std::cout << "control deinit" << std::endl;
    }
private:
    ss::SwerveDrive& m_swerve;
    rev::spark::SparkMax m_algaeIntakeWheel, m_outtakeWheel, m_elevator;
    ctre::phoenix::motorcontrol::can::TalonSRX m_algaeIntakeActuator, & m_outtakeActuator;
    ctre::phoenix::motorcontrol::can::VictorSPX m_climber;
    ctre::phoenix6::hardware::TalonFX m_capstan;
    frc::Servo m_coralIntake;
public:
    enum class CapstanHomingStep {
        STOPPED, RUN_POSITIVE, RUN_NEGATIVE, DONE
    } m_capstanHomingStep = CapstanHomingStep::STOPPED;
private:
    float m_capstanMaxPos;
    float m_capstanMinPos;

    int m_capstanIterationsStopped = 0;

    const float c_algaeIntakeActuatorOffset = -812 / 2048;

    const float c_elevRevsPerM = 180.5555;
};
} // end namespace ss
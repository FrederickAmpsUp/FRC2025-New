#pragma once

#include <glm/glm.hpp>
#include <frc/joystick.h>
#include <guidance.hpp>

namespace ss {

class Navigation {
public:
    Navigation(frc::Joystick& joy, ss::Guidance& guidance) : m_joystick(joy), m_guidance(guidance) {}

    void update_navigation();

    glm::vec2 get_desired_drive() { return this->m_desiredDrive; }
    float get_desired_turn() { return this->m_desiredTurn; }
private:
    glm::vec2 m_desiredDrive = glm::vec2(0.0); // m/s
    float m_desiredTurn; // radians/s

    const float c_maxDriveSpeed = 5.0; // m/s
    const float c_minDriveSpeed = 0.5; // m/s
    const float c_maxTurnSpeed = 2.0 * M_PI; // radians/s
    const float c_minTurnSpeed = 0.5 * M_PI; // radians/s

    ss::Guidance& m_guidance;

    frc::Joystick& m_joystick;
};

} // end namespace ss
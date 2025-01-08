#include <control.hpp>

namespace ss {

void Control::update_control() {
    glm::vec2 drive = this->m_nav.get_desired_drive();
    float turn = this->m_nav.get_desired_turn();

    this->m_swerve.set(drive, turn);
}
} // end namespace ss
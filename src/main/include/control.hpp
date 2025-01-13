#pragma once

#include <swerve.hpp>
#include <navigation.hpp>

namespace ss {

class Control {
public:
    Control(ss::SwerveDrive& swerve) : m_swerve(swerve) {}

    template<typename Nav>
    void update_control(const Nav& nav) {
        glm::vec2 drive = nav.get_desired_drive();
        float turn = nav.get_desired_turn();

        this->m_swerve.set(drive, turn);
    }
private:
    ss::SwerveDrive& m_swerve;
};
} // end namespace ss
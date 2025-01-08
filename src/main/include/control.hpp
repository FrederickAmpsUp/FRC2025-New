#pragma once

#include <swerve.hpp>
#include <navigation.hpp>

namespace ss {

class Control {
public:
    Control(ss::SwerveDrive& swerve, ss::Navigation& navigation) : m_swerve(swerve), m_nav(navigation) {}

    void update_control();
private:
    ss::SwerveDrive& m_swerve;
    ss::Navigation& m_nav;
};
} // end namespace ss
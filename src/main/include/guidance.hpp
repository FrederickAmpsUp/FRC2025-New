#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <swerve.hpp>
#include <studica/AHRS.h>

namespace ss {

class Guidance {
public:
    Guidance(ss::SwerveDrive& drive, glm::vec2 frameStartPosition);

    struct Info {
        /**
         * Velocity relative to the field (bird's eye) in m/s
        */
        glm::vec2 fieldVelocity;
        /**
         * Angular velocity (positive = clockwise) in radians/s
        */
        float angularVelocity;

        /**
         * Position relative to the field (bird's eye) in meters
        */
        glm::vec2 fieldPosition;
        /**
         * Angle relative to the field (0 = away from DS, positive = clockwise) in radians
        */
        float fieldAngle;
    };

    void update_guidance();

    std::shared_ptr<Info> info() { return this->m_info; }
private:
    std::shared_ptr<Info> m_info;
    ss::SwerveDrive& m_drive;
    studica::AHRS m_navx;
    std::vector<glm::vec2> m_modulePositions;
};
}
/** 
 *     This file is part of dqrobot.
 *  
 *     dqrobot is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dqrobot is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dqrobot. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file examples/example1.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#include <array>
#include "SerialManipulator.hpp"

int main() {
    std::array<std::array<float, 6>, 5> dh {
        std::array<float, 6>{0,       M_PI,      -M_PI_2,       0,      0,   0},
        std::array<float, 6>{0.1735,  0,         0,       -0.1182,  0,      -0.264},
        std::array<float, 6>{0.027,   0.1629,   0.055,    0,      0,      0},
        std::array<float, 6>{-M_PI_2,     M_PI,      -M_PI_2,       M_PI_2,    -M_PI_2,     M_PI},
        std::array<float, 6>{0,       0,      0,       0,      0,   0}
    };

    std::array<std::array<float, 6>, 4> limits {
        std::array<float, 6>{-150, -10, -10, -260, -90, -350},
        std::array<float, 6>{150, 110, 150, 260, 100, 350},
        std::array<float, 6>{-33, -32.1, -59.40, -60.50, -60.50, -90.80},
        std::array<float, 6>{33, 32.1, 59.40, 60.50, 60.50, 90.80}
    };
    for (int i=0; i<6; ++i) {{
        limits[0][i] = limits[0][i] / 180 * M_PI;
        limits[1][i] = limits[0][i] / 180 * M_PI;
        limits[2][i] = limits[0][i] / 180 * M_PI;
        limits[3][i] = limits[0][i] / 180 * M_PI;
    }}

    std::array<float, 6> joint_positions {0,0,0,0,0,0};

    dqkinematics::SerialManipulator<float, 6> robot(dh, limits, joint_positions);

    dqpose::Posef x_init = robot.end_pose();
    dqpose::Tranf t_init = x_init.translation();
    dqpose::Rotf r_init = x_init.rotation();
    float radius = 0.01;
    float rad_speed = 0.001;
    size_t i = 0;

    while (true)
    {   
        dqpose::Tranf offset = t_init + dqpose::Tranf(-0.02, radius * cos(rad_speed*i), radius * sin(rad_speed*i) + 0.05);
        ++i;
        dqpose::Posef xd = dqpose::Posef(r_init, offset);

        robot.update(xd);

        std::cout << robot.end_pose() << ">>" << xd << "\n";
    }

}             
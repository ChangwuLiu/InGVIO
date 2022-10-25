/**  This File is part of InGVIO, an invariant filter for mono/stereo visual-
 *    inertial-raw GNSS navigation. 
 *    
 *    Copyright (C) 2022  Changwu Liu (cwliu529@163.com,
 *                                     lcw18@mails.tsinghua.edu.cn (valid until 2023))
 *    
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *    
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// The TicToc Class is inspired by such realization in GVINS.
// see https://github.com/HKUST-Aerial-Robotics/GVINS

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace ingvio
{
    class TicToc
    {
    public:
        TicToc()
        {
            tic();
        }
        
        void tic()
        {
            start = std::chrono::system_clock::now();
        }
        
        double toc()
        {
            end = std::chrono::system_clock::now();
            
            std::chrono::duration<double> elapsed_seconds = end - start;
            
            return elapsed_seconds.count() * 1000;
        }
        
    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
    };
}

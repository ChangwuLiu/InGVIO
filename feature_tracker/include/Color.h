/**   This File is part of Feature Tracker
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

#pragma once

#include <iostream>

namespace color
{
    inline std::ostream& setBlue(std::ostream& s)
    {
        s << "\033[0;1;34m";
        return s;
    }
    
    inline std::ostream& setRed(std::ostream& s)
    {
        s << "\033[0;1;31m";
        return s;
    }
    
    inline std::ostream& setGreen(std::ostream& s)
    {
        s << "\033[0;1;32m";
        return s;
    }
    
    inline std::ostream& setYellow(std::ostream& s)
    {
        s << "\033[0;1;33m";
        return s;
    }
    
    inline std::ostream& setWhite(std::ostream& s)
    {
        s << "\033[0;1;37m";
        return s;
    }
    
    inline std::ostream& resetColor(std::ostream& s)
    {
        s << "\033[0m";
        return s;
    }
}

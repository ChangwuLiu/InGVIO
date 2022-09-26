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

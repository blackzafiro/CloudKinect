 /*
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2017 UNAM, Mex.
  *
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of UNAM nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  */

#include <iostream>

#ifndef __TERM_COLOR_PRINT_H_
#define __TERM_COLOR_PRINT_H_

namespace PrettyPrint {

    /** Color end */
    const std::string CEND = "\e[0m";

	// There could be more colors, but they depend on the terminal
	// http://misc.flogisoft.com/bash/tip_colors_and_formatting
    
    /** Foreground colors */
    enum Foreground {
        Black = 30, Red, Green, Yellow, Blue, Magenta, Cyan, LightGray, Default = 39,
        DarkGray = 90, LightRed, LightGreen, LightYellow, LightBlue, LightMagenta,
		LightCyan, White
    };
    
    /** Background colors */
    enum Background {
        BGrey = 40, BRed, BGreen, BYellow, BBlue, BMagenta, BCyan, BLightGray, BDefault = 49,
		BDarkGray = 90, BLightRed, BLightGreen, BLightYellow, BLightBlue, BLightMagenta,
		BLightCyan, BWhite
    };
    
    /** Attributes */
    enum Attribute {
        ADefault = 0, Bold, Dim, Slanted, Underlined, Blink, Reverse=7, Hidden
    };
    
    /** Wrapper for cout and cerr that prints values in color. */
    class ColorPrinter : public std::ostream {
    private:
        std::ostream& stm;
        std::string cini;
        
    public:
        /**
         * This object is a wrapper for cout or cerr, which surrounds values
         * with code for printting in color in term.
         */
        ColorPrinter(std::ostream& stm, Foreground color, Background bcolor=BDefault, Attribute=ADefault);
        
        /** Pases value to the stream and keeps working with Pretty Print. */
        template <typename T>
        ColorPrinter& operator<< ( T value) {
            this->stm << this->cini << value;
            return *this;
        }
        
        /** Meant to be used with std::endl */
        ColorPrinter& operator<<(std::ostream& (*endl)(std::ostream&));
    };
}

#endif

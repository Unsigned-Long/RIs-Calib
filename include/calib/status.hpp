// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIS_CALIB_STATUS_HPP
#define RIS_CALIB_STATUS_HPP

#include "exception"
#include "string"
#include "util/enum_cast.hpp"

namespace ns_ris {
    struct Status : std::exception {
        enum class Flag {
            FINE, WARNING, ERROR, CRITICAL
        };
    public:
        Flag flag;
        std::string what;

        Status(Flag flag, std::string what) : flag(flag), what(std::move(what)) {}

        Status() : flag(Flag::FINE), what() {}

        friend std::ostream &operator<<(std::ostream &os, const Status &status) {
            os << "[" << EnumCast::enumToString(status.flag) << "]-[" << status.what << "]";
            return os;
        }
    };
}

#endif //RIS_CALIB_STATUS_HPP

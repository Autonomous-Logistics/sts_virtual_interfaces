/* Copyright (C) 2018-2020 Nick Fiege - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

#include <sts_core_utils/sts_core/utils/betterEnums.h>
#include <sts_virtual_interface_msgs.h>

namespace sts_health_monitoring_types{

enum HEALTHRESULT{NOTIMPLEMENTED, ACK, NACK};

typedef std::function<sts_health_monitoring_types::HEALTHRESULT(void)> callbackFct;


// from: https://stackoverflow.com/questions/18335861/why-is-enum-class-preferred-over-plain-enum
// The basic advantage of using enum class over normal enums is that you may have same
// enum variables for 2 different enums and still can resolve them

/*
enum class Severity : unsigned int
{
    SAFE = 0x00,
    NON_CRITICAL_INFO = 0x01,
    NON_CRITICAL_WARNING = 0x02,
    WARNING = 0x03,
    ERROR = 0x04,
    CRITICAL_ERROR = 0x05
};*/

/// fgn: we dont use a normal enum class, we use a custom enum macro now.
/// works like a charm x3 =)

DECLARE_ENUM_WITH_TYPE(Severity, uint32_t,
   SAFE = 0x00,
   NON_CRITICAL_INFO = 0x01,
   NON_CRITICAL_WARNING = 0x02,
   WARNING = 0x03,
   ERROR = 0x04,
   CRITICAL_ERROR = 0x05
);


DECLARE_ENUM_WITH_TYPE(HealthReportType, uint32_t,
    NONE = 0x000,
    OTHER = 0x001,
    UNKNOWN = 0x002,

    COMMUNICATION_ERROR = 0x100,
    COMMUNICATION_MISMATCH = 0x101,
    COMMUNICATION_MISSING = 0x1FF,

    ACCURACY_ERROR = 0x200,
    ACCURACY_MISMATCH = 0x201,

    RANGE_ERROR = 0x300,
    RANGE_MISMATCH = 0x301,

    INPUT_ERROR = 0x400,
    INPUT_FILESYSTEM = 0x401,
    INPUT_ROS = 0x402,
    INPUT_MISSING = 0x4FF,

    OUTPUT_ERROR = 0x500,
    OUTPUT_FILESYSTEM = 0x501,
    OUTPUT_ROS = 0x502,

    USER_ERROR = 0x600,
    USER_TIMEOUT = 0x601,
    USER_INTERRUPT = 0x602,

    OPERATION_RANGE_ERROR = 0xE00,
    OPERATION_RANGE_ENVIRONMENT_UNSUITABLE = 0xE01,

    DANGER = 0xF00,
    DANGER_OBSTACLE = 0xF01,
    DANGER_MALFUNCTION = 0xF02
);


/// defines:
///     - the type of the source of the problem (who is probably responsible?!)
///     - the type of the target of the problem (who is the likely victim?!)
DECLARE_ENUM_WITH_TYPE(Participant, uint32_t,
    NONE = 0x00,
    OTHER = 0x01,
    UNKNOWN = 0x02,

    TOPIC = 0x10,
    NODE = 0x40,
    GROUP = 0x70,

    SYSTEM = 0xF0
);


} //namespace

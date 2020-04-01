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

#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthReport.h>
#include <std_msgs/UInt32.h>

using namespace sts_core::sts_interfaces;
using namespace sts_core::sts_interfaces::sts_health_monitor;

HealthReport::HealthReport(std::string humanMsg, std::string machineMsg,
                           sts_health_monitoring_types::Severity severity,
                           sts_health_monitoring_types::HealthReportType type) : Report(humanMsg, machineMsg)
{
    this->initialize(severity, type);
}

std::string HealthReport::toString()
{
    std::ostringstream stringStream;
    stringStream <<  sts_core::sts_interfaces::Report::toString();
    stringStream << " - severity: " << this->severity_ << std::endl;
    stringStream << " - type: " << this->type_ << std::endl;
    stringStream << " - source: " << this->source_ << " [" << this->sourceType_ << "] " << std::endl;
    stringStream << " - target: " << this->target_ << " [" << this->targetType_ << "] " << std::endl;
    std::string str = stringStream.str();
    return str;
}

void HealthReport::convertToRosMsg(sts_virtual_interface_msgs::HealthReport& r)
{
    Report::convertToRosMsg(r);
    r.severity = (uint32_t)this->severity_;
    r.severity_str = ~this->severity_;
    r.type = (uint32_t)this->type_;
    r.type_str = ~this->type_;

    r.source_type = (uint32_t)this->sourceType_;
    r.source_type_str = ~this->sourceType_;
    r.source = this->source_;

    r.target_type = (uint32_t) this->targetType_;
    r.target_type_str = ~this->targetType_;
    r.target = this->target_;
}


void HealthReport::initialize(sts_health_monitoring_types::Severity severity, sts_health_monitoring_types::HealthReportType type)
{
    this->severity_ = severity; this->type_ = type;
}

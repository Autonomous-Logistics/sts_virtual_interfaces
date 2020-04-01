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

#include <iostream>
#include <string>
#include <functional>
#include <memory>

#include <sts_virtual_interface_msgs.h>
#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthMonitorTypes.h>



namespace sts_core {
namespace sts_interfaces {
namespace sts_health_monitor {


class HealthReport : public sts_core::sts_interfaces::Report
{
public:
    /// constructors
    HealthReport(std::string humanMsg, std::string machineMsg,
                 sts_health_monitoring_types::Severity severity,
                 sts_health_monitoring_types::HealthReportType type);

    virtual std::string toString();

    /// public api
    template <class T>
    void setSource(sts_health_monitoring_types::Participant sourceType, T source)
    { this->source_ = std::string(source); this->sourceType_ = sourceType;}
    template <class T>
    void setTarget(sts_health_monitoring_types::Participant targetType, T target)
    { this->target_ = std::string(target); this->targetType_ = targetType;}

    void convertToRosMsg(sts_virtual_interface_msgs::HealthReport& r);

private:
    void initialize(sts_health_monitoring_types::Severity severity, sts_health_monitoring_types::HealthReportType type);

public:

private:
    sts_health_monitoring_types::Severity severity_;
    sts_health_monitoring_types::HealthReportType type_ = sts_health_monitoring_types::HealthReportType::NONE;

    sts_health_monitoring_types::Participant sourceType_ = sts_health_monitoring_types::Participant::NONE;
    std::string source_;
    sts_health_monitoring_types::Participant targetType_ = sts_health_monitoring_types::Participant::NONE;
    std::string target_;
    //std::vector<sts_health_monitoring_types::Action> possible_actions_;

};
typedef std::shared_ptr<HealthReport> HealthReportPtr;


} /* namespace sts_health_monitor */
} /* namespace sts_interfaces */
} /* namespace sts_core */

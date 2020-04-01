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

#include <ros/ros.h>
#include <memory>

#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthMonitorTypes.h>
#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthReport.h>


namespace sts_core {
namespace sts_interfaces {

namespace sts_health_monitor {

class StsHealthReportFactory
{
public:
    /// public factory api
    template <class T1, class T2>
    static HealthReportPtr create(T1& humanMsg, T2& machineMsg,
                                  sts_health_monitoring_types::Severity severity,
                                  sts_health_monitoring_types::HealthReportType type)
    { return StsHealthReportFactory::create_(humanMsg, machineMsg, severity, type); }

    static HealthReportPtr create(sts_health_monitoring_types::Severity severity,
                                  sts_health_monitoring_types::HealthReportType type)
    { return StsHealthReportFactory::create("", "", severity, type); }


private:
    /// factory implementation
    static HealthReportPtr create_(std::string humanMsg, std::string machineMsg,
                                   sts_health_monitoring_types::Severity severity,
                                   sts_health_monitoring_types::HealthReportType type);
};


} /* namespace sts_health_monitor */
} /* namespace sts_interfaces */
} /* namespace sts_core */

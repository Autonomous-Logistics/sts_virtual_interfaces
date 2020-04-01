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

#include <sts_core_utils/sts_core/sts_base/sts_core_node.h>

#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthMonitor.h>
#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthReportFactory.h>
#include <sts_virtual_interface_msgs.h>


namespace sts_core {
namespace sts_interfaces {

class StsHealthMonitorInterface : public virtual sts_core::sts_base::StsCoreNode
{
public:
    StsHealthMonitorInterface(ros::NodeHandle* nodeHandlePtr);
    ~StsHealthMonitorInterface();

protected:
    /// Interface provides callback functions which will be set at the hm_
    /// these have to be overrided by user code (inheritance) to function properly
    virtual sts_health_monitoring_types::HEALTHRESULT healthCheck() {return sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED;}
    virtual sts_health_monitoring_types::HEALTHRESULT healthReset() {return sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED;}

    template <class T>
    void pushReport(T r){
        this->hm_->pushReport(r);
    }

private:
    sts_core::sts_interfaces::sts_health_monitor::StsHealthMonitor* hm_;

};

} /* namespace */
} /* namespace */

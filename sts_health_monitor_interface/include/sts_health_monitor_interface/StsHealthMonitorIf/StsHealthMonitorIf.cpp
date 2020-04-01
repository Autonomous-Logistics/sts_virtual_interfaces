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

#include <sts_health_monitor_interface/StsHealthMonitorIf/StsHealthMonitorIf.h>

using namespace sts_core::sts_interfaces;

StsHealthMonitorInterface::StsHealthMonitorInterface(ros::NodeHandle* nodeHandlePtr){
    if (!this->initialize(nodeHandlePtr))
        this->setNOK();
    /// create hm_ instance
    this->hm_ = new sts_core::sts_interfaces::sts_health_monitor::StsHealthMonitor(this->getNodeHandle());
    /// bind relevant functions of this interface to hm_, our functions are the glue between hm_ and user (virtual)
    /// we hope that a user will implement /override those when they inherit us, or lets hope that they set those if
    /// the instantiate hmthemselfs
    this->hm_->setHealthCheckCallback(std::bind(&StsHealthMonitorInterface::healthCheck, this));
    this->hm_->setHealthResetCallback(std::bind(&StsHealthMonitorInterface::healthReset, this));
}
StsHealthMonitorInterface::~StsHealthMonitorInterface(){
    if(this->hm_ != nullptr)
        delete this->hm_;
}


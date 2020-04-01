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

#include <sts_virtual_interface_msgs.h>

#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthMonitorTypes.h>
#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthReportFactory.h>


namespace sts_core {
namespace sts_interfaces {

namespace sts_health_monitor {

class StsHealthMonitor
{
public:
    /// default constructor
    StsHealthMonitor(ros::NodeHandle* nodeHandlePtr);
    /// if a user wants to instantiate this hm_ on it's own, he needs a mechanism to override those service callbacks to use it properly
    StsHealthMonitor(ros::NodeHandle* nodeHandlePtr,
                     sts_health_monitoring_types::callbackFct healthCheckCb,
                     sts_health_monitoring_types::callbackFct healthResetCb);
    /// default destructor
    ~StsHealthMonitor();
public:
    /// public callback setter api
    void setHealthCheckCallback(sts_health_monitoring_types::callbackFct cb){this->userHealthCheckCallback_ = cb;}
    void setHealthResetCallback(sts_health_monitoring_types::callbackFct cb){this->userHealthResetCallback_ = cb;}

    template<class T>
    void pushReport(T* r){
        this->pushReport_(r->getBasePtr());
    }
    template<class T>
    void pushReport(T& r){
        T* tp = &r; this->pushReport_(tp->getBasePtr());
    }
    template<class T>
    void pushReport(std::shared_ptr<T> r){
        T* tp = r.get(); this->pushReport_(tp->getBasePtr());
    }

private:
    void initialize(ros::NodeHandle* nodeHandlePtr);
    /// default subscriber callbacks for ros interfaces
    bool healthCheckCallbackFunc(sts_virtual_interface_msgs::HealthCheck::Request  &request,
                                 sts_virtual_interface_msgs::HealthCheck::Response &response);
    bool healthResetCallbackFunc(sts_virtual_interface_msgs::HealthReset::Request  &request,
                                 sts_virtual_interface_msgs::HealthReset::Response &response);
    /// callback for checking if we are ok, user callbacks will be checked and called here
    sts_health_monitoring_types::HEALTHRESULT healthCheck();
    sts_health_monitoring_types::HEALTHRESULT healthReset();

    void pushReport_(Report* r);

private:
    ros::NodeHandle* nodeHandlePtr;
    ros::ServiceServer healthCheckServiceServer_;
    ros::ServiceServer healthResetServiceServer_;
    ros::Publisher healthReportPublisher_;

    sts_health_monitoring_types::callbackFct userHealthCheckCallback_ = NULL;
    sts_health_monitoring_types::callbackFct userHealthResetCallback_ = NULL;
};


} /* namespace sts_health_monitor */
} /* namespace sts_interfaces */
} /* namespace sts_core */

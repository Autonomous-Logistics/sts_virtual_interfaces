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

#include <sts_health_monitor_interface/StsHealthMonitor/StsHealthMonitor.h>

using namespace sts_core::sts_interfaces::sts_health_monitor;



StsHealthMonitor::StsHealthMonitor(ros::NodeHandle* nodeHandlePtr){
    this->initialize(nodeHandlePtr);
}
/// if a user wants to instantiate this hm_ on it's own, he needs a mechanism to override those service callbacks to use it properly
StsHealthMonitor::StsHealthMonitor(ros::NodeHandle* nodeHandlePtr,
                 sts_health_monitoring_types::callbackFct healthCheckCb, sts_health_monitoring_types::callbackFct healthResetCb){
    this->initialize(nodeHandlePtr);
    this->setHealthCheckCallback(healthCheckCb);
    this->setHealthResetCallback(healthResetCb);
}

StsHealthMonitor::~StsHealthMonitor(){

}

void StsHealthMonitor::initialize(ros::NodeHandle* nodeHandlePtr){
    this->nodeHandlePtr = nodeHandlePtr;
    this->healthCheckServiceServer_ = this->nodeHandlePtr->advertiseService("HealthCheck", &StsHealthMonitor::healthCheckCallbackFunc, this);
    this->healthResetServiceServer_ = this->nodeHandlePtr->advertiseService("HealthReset", &StsHealthMonitor::healthResetCallbackFunc, this);
    this->healthReportPublisher_ = this->nodeHandlePtr->advertise<sts_virtual_interface_msgs::HealthReport>("/HealthReports", 3, false);
    /// publishers are not ready instantaneously, so we wait a little bit
    usleep(1000*200);
}

bool StsHealthMonitor::healthCheckCallbackFunc(sts_virtual_interface_msgs::HealthCheck::Request  &request,
                                               sts_virtual_interface_msgs::HealthCheck::Response &response){
    sts_health_monitoring_types::HEALTHRESULT r = this->healthCheck();
    bool res = true;
    if( r == sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED || r == sts_health_monitoring_types::HEALTHRESULT::NACK )
        res = false;
    response.success = res;
    return true;
}

bool StsHealthMonitor::healthResetCallbackFunc(sts_virtual_interface_msgs::HealthReset::Request  &request,
                                               sts_virtual_interface_msgs::HealthReset::Response &response){
    sts_health_monitoring_types::HEALTHRESULT r = this->healthReset();
    bool res = true;
    if( r == sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED || r == sts_health_monitoring_types::HEALTHRESULT::NACK )
        res = false;
    response.success = res;
    return true;
}

sts_health_monitoring_types::HEALTHRESULT StsHealthMonitor::healthCheck(){
    if(this->userHealthCheckCallback_)
        return this->userHealthCheckCallback_();
    else
        return sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED;
}
sts_health_monitoring_types::HEALTHRESULT StsHealthMonitor::healthReset(){
    if(this->userHealthResetCallback_)
        return this->userHealthResetCallback_();
    else
        return sts_health_monitoring_types::HEALTHRESULT::NOTIMPLEMENTED;
}

/// template push report specializations
/*
template<>
void StsHealthMonitor::pushReport(sts_core::sts_interfaces::sts_health_monitor::StsHealthMonitor:: r){
}
*/

/// base push report implementation
void StsHealthMonitor::pushReport_(sts_core::sts_interfaces::Report *r){
    /// set more stuff inside the report
    std::string sender_nodename = this->nodeHandlePtr->getNamespace();
    r->setSender(sender_nodename);
    /// set and fill our message
    sts_virtual_interface_msgs::HealthReport reportMsg;
    r->convertToRosMsg(reportMsg);
    /// we want to publish that thing
    this->healthReportPublisher_.publish(reportMsg);
}

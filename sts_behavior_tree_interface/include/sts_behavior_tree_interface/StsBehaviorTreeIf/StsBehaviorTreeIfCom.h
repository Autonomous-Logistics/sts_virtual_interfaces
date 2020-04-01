/* Copyright (C) 2018-2020 Sebastian Mueller, Nick Fiege - All Rights Reserved
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
#include <string>
#include <chrono>
#include <thread>

#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIfTypes.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <actionlib/server/simple_action_server.h>

#include <sts_core_utils/sts_core/sts_io/sts_io.h>
#include <sts_virtual_interface_msgs.h>
#include <sts_virtual_interface_msgs/BTAction.h>
#include <sts_virtual_interface_msgs/BTService.h>


//##################################
//####### Communication BASE #######
//##################################



class StsBehaviorTreeIfComBase
{
public:
    enum COMAPIRETURN { NOTIMPLEMENTED, ACK, NACK };

    StsBehaviorTreeIfComBase() : userCallback(NULL) {}
    ~StsBehaviorTreeIfComBase() {}

    virtual void initialize(bool b, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb);

    void setDefaultResult(bool b) {this->defaultResult_ = b;}
    bool getDefaultResult() {return this->defaultResult_;}
    void setCallbackFct(sts_behavior_tree_if_types::callbackFct cb) {this->userCallback = cb;}


    /// internal virtuals different for each COM type
    virtual COMAPIRETURN sendAsync(bool value);
    virtual void setFrequency(double frequency);
    virtual double getFrequency();

    virtual void reset(){}


protected:

    void setIdentifier(std::string id) {this->identifier = id;}
    std::string getIdentifier() {return this->identifier;}

    std::string normalizeToRosName(const std::string& name){
        std::string nn(name);
        ///remove leading '/'
        if(nn[0] == '/') nn = nn.substr(1);
        ///add namespace 
        std::string ns = sts_core::sts_interfaces::BEHAVIOR_TREE_NAMESPACE;
        nn = ns + "/" + nn;
        return nn;
    }

    void setNodeHandle(ros::NodeHandle* nodeHandlePtr) {this->nodeHandlePtr = nodeHandlePtr;}
    ros::NodeHandle* getNodeHandle() {return this->nodeHandlePtr;}

    sts_behavior_tree_if_types::callbackFct getCallbackFct() {return this->userCallback;}

private:
    bool defaultResult_;
    std::string identifier;
    ros::NodeHandle* nodeHandlePtr;
    sts_behavior_tree_if_types::callbackFct userCallback;
    double frequency_;
};

typedef std::shared_ptr<StsBehaviorTreeIfComBase> btIfComPtr;

//####################################
//############# TOPIC ################
//####################################

class StsBehaviorTreeIfComTopic : public StsBehaviorTreeIfComBase
{
public:
    StsBehaviorTreeIfComTopic();
    ~StsBehaviorTreeIfComTopic();
    
    void initialize(bool b, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb);

private:
    void bt_publish(const ros::TimerEvent& e);
    void setFrequency(double frequency);

private:
    ros::Publisher pub_;
    ros::Timer pubTimer_;
    sts_behavior_tree_if_types::StsBehaviorTreeResult lastResult_;
};

//####################################
//############  FLAG  ################
//####################################

class StsBehaviorTreeIfComFlag : public StsBehaviorTreeIfComBase
{
public:
    StsBehaviorTreeIfComFlag();
    ~StsBehaviorTreeIfComFlag();

    void initialize(bool b, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb);
    StsBehaviorTreeIfComBase::COMAPIRETURN sendAsync(bool value);

private:
    ros::Publisher pub_;
    sts_behavior_tree_if_types::StsBehaviorTreeResult lastResult_;
};

//##################################
//############# SERVICE ############
//##################################

class StsBehaviorTreeIfComService: public StsBehaviorTreeIfComBase
{
public:
    StsBehaviorTreeIfComService();
    ~StsBehaviorTreeIfComService();

    void initialize(bool b, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb);


private:
    bool bt_serviceCallback(sts_virtual_interface_msgs::BTService::Request &req,
                            sts_virtual_interface_msgs::BTService::Response &res);

private:
    ros::ServiceServer service_;
};

//#################################
//############# ACTION ############
//#################################

class StsBehaviorTreeIfComAction: public StsBehaviorTreeIfComBase
{    
public:
    StsBehaviorTreeIfComAction();
    ~StsBehaviorTreeIfComAction();

    void initialize(bool b, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb);
    virtual void reset();

private:
    void bt_actionCallback(const sts_virtual_interface_msgs::BTGoalConstPtr &goal);
    void bt_preemptCallback();
    void preempt(sts_virtual_interface_msgs::BTResult &btActionResult);

private:
    actionlib::SimpleActionServer<sts_virtual_interface_msgs::BTAction>* asPtr;
    bool running = false;
    bool preemptRequested;
};

//##################################
//############# FACTORY ############
//##################################

class StsBehaviorTreeIfComFactory
{
public:
    static btIfComPtr createRosCom(std::string identifier, sts_behavior_tree_if_types::COMTYPE type, ros::NodeHandle* nhPtr, bool errorDefault, sts_behavior_tree_if_types::callbackFct userCallback);

};

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

#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIfCom.h>

//##################################
//####### Communication BASE #######
//##################################

void StsBehaviorTreeIfComBase::initialize(bool defaultResult, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb)
{
    this->setDefaultResult(defaultResult);
    this->setIdentifier(id);
    this->setNodeHandle(nodeHandlePtr);
    this->setCallbackFct(cb);

    this->setFrequency(1.0); //Set default frequency to 1 Hz
}

/// internal virtuals different for each COM type
StsBehaviorTreeIfComBase::COMAPIRETURN StsBehaviorTreeIfComBase::sendAsync(bool value){return StsBehaviorTreeIfComBase::NOTIMPLEMENTED;}
void StsBehaviorTreeIfComBase::setFrequency(double f){this->frequency_ = f;}
double StsBehaviorTreeIfComBase::getFrequency(){return this->frequency_;}


//####################################
//############# TOPIC ################
//####################################

StsBehaviorTreeIfComTopic::StsBehaviorTreeIfComTopic(){}
StsBehaviorTreeIfComTopic::~StsBehaviorTreeIfComTopic(){}

void StsBehaviorTreeIfComTopic::initialize(bool defaultResult, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb)
{
	//call base initialize first
    StsBehaviorTreeIfComBase::initialize(defaultResult, id, nodeHandlePtr, cb);
		
    //do your own topic related initialization

    //set default result
    this->lastResult_ = sts_behavior_tree_if_types::StsBehaviorTreeResult();
    this->lastResult_.setResult(this->getDefaultResult());

    ///set publisher >> [fgn] this is latched!
    this->pub_ = this->getNodeHandle()->advertise<std_msgs::Bool>(this->normalizeToRosName(this->getIdentifier()), 1, true);
    this->pubTimer_ = this->getNodeHandle()->createTimer(ros::Duration(1/this->getFrequency()), &StsBehaviorTreeIfComTopic::bt_publish, this);

    ///[fgn] we have to publish out status at least ONCE
    std_msgs::Bool message;
    message.data = defaultResult;
    this->pub_.publish(message);

    return;
}

void StsBehaviorTreeIfComTopic::setFrequency(double frequency)
{
    //Set frequency attribute first
    StsBehaviorTreeIfComBase::setFrequency(frequency);
    if(frequency != 0.0)
        this->pubTimer_.setPeriod(ros::Duration(1/this->getFrequency()), true);

    return;
}

void StsBehaviorTreeIfComTopic::bt_publish(const ros::TimerEvent& e)
{
    std_msgs::Bool message;
	//if callback was declared by user, call it
    if(this->getCallbackFct() != NULL)
    {
        sts_behavior_tree_if_types::StsBehaviorTreeParam result = sts_behavior_tree_if_types::StsBehaviorTreeParam();
        this->getCallbackFct()(&result);

        //if new result values publish
        if (!this->lastResult_.isEqual(result.getResultObj()))
        {
            //Set latest result
            this->lastResult_ = sts_behavior_tree_if_types::StsBehaviorTreeResult(result.getResultObj()); //Copy constructor
            //publish
            message.data = this->lastResult_.getResult();
            this->pub_.publish(message);
        }
    }
    return;
}


//####################################
//############  FLAG  ################
//####################################
StsBehaviorTreeIfComFlag::StsBehaviorTreeIfComFlag(){}
StsBehaviorTreeIfComFlag::~StsBehaviorTreeIfComFlag(){}

void StsBehaviorTreeIfComFlag::initialize(bool defaultResult, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb)
{
    //call base initialize first
    StsBehaviorTreeIfComBase::initialize(defaultResult, id, nodeHandlePtr, cb);
    //do your own topic related initialization
    //set default result
    this->lastResult_ = sts_behavior_tree_if_types::StsBehaviorTreeResult();
    this->lastResult_.setResult(this->getDefaultResult());
    ///set publisher >> [fgn] this is latched!
    this->pub_ = this->getNodeHandle()->advertise<std_msgs::Bool>(this->normalizeToRosName(this->getIdentifier()), 1, true);
    ///[fgn] we have to publish out status at least ONCE
    std_msgs::Bool result;
    result.data = defaultResult;
    this->pub_.publish(result);
    return;
}

StsBehaviorTreeIfComBase::COMAPIRETURN StsBehaviorTreeIfComFlag::sendAsync(bool value)
{
    std_msgs::Bool message;
    /// sendAsync has no callback, we just send this val to the tree
    sts_behavior_tree_if_types::StsBehaviorTreeResult r = sts_behavior_tree_if_types::StsBehaviorTreeResult();
    r.setResult(value);
    if (!this->lastResult_.isEqual(&r))
    {
        //Set latest result
        this->lastResult_ = r;
        //publish
        message.data = this->lastResult_.getResult();
        this->pub_.publish(message);
    }
    return StsBehaviorTreeIfComBase::ACK;
}


//####################################
//############# Service ##############
//####################################

StsBehaviorTreeIfComService::StsBehaviorTreeIfComService()
{}

StsBehaviorTreeIfComService::~StsBehaviorTreeIfComService()
{}

void StsBehaviorTreeIfComService::initialize(bool defaultResult, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb)
{
    //call base initialize first
    StsBehaviorTreeIfComBase::initialize(defaultResult, id, nodeHandlePtr, cb);

    //do your own topic related initialization
    this->service_ = this->getNodeHandle()->advertiseService(this->normalizeToRosName(this->getIdentifier()), &StsBehaviorTreeIfComService::bt_serviceCallback, this);

    return;
}

bool StsBehaviorTreeIfComService::bt_serviceCallback(sts_virtual_interface_msgs::BTService::Request &req,
                                               sts_virtual_interface_msgs::BTService::Response &res)
{
    //set default result
    res.result = this->getDefaultResult();

    //if callback was declared by user, call it
    if (this->getCallbackFct() != NULL)
    {
        sts_behavior_tree_if_types::StsBehaviorTreeParam result = sts_behavior_tree_if_types::StsBehaviorTreeParam();
        this->getCallbackFct()(&result);
        res.result = result.getResultObj()->getResult();
    }

    return true;
}

//####################################
//############# Action ###############
//####################################

StsBehaviorTreeIfComAction::StsBehaviorTreeIfComAction() : asPtr(NULL)
{}

StsBehaviorTreeIfComAction::~StsBehaviorTreeIfComAction()
{
    if (this->asPtr != NULL)
        delete this->asPtr;
}

void StsBehaviorTreeIfComAction::initialize(bool defaultResult, std::string id, ros::NodeHandle* nodeHandlePtr, sts_behavior_tree_if_types::callbackFct cb)
{
    //call base initialize first
    StsBehaviorTreeIfComBase::initialize(defaultResult, id, nodeHandlePtr, cb);

    //do your own topic related initialization
    this->preemptRequested = false;

    this->asPtr = new actionlib::SimpleActionServer<sts_virtual_interface_msgs::BTAction>(*(this->getNodeHandle()),this->normalizeToRosName(this->getIdentifier()), boost::bind(&StsBehaviorTreeIfComAction::bt_actionCallback, this, _1), false);
    this->asPtr->registerPreemptCallback(std::bind(&StsBehaviorTreeIfComAction::bt_preemptCallback, this));
    this->asPtr->start();

    return;
}

void StsBehaviorTreeIfComAction::bt_preemptCallback()
{
    this->preemptRequested = true;
    return;
}

void StsBehaviorTreeIfComAction::bt_actionCallback(const sts_virtual_interface_msgs::BTGoalConstPtr &goal)
{
    this->running = true;
    sts_virtual_interface_msgs::BTResult btActionResult_;

    //set action default return value
    if (getDefaultResult() == false)
        btActionResult_.status = 2;
    else
        btActionResult_.status = 1;

    //If userCallback registered
    if(this->getCallbackFct() != NULL)
    {
        //Set result args
        sts_behavior_tree_if_types::StsBehaviorTreeParam result = sts_behavior_tree_if_types::StsBehaviorTreeParam();
        result.getArgsObj()->setPreemptRequestReference(&this->preemptRequested);

        //Call userCallback
        this->getCallbackFct()(&result);

        /// check if the actionServer has a pending preempt request from an external source
        /*if (this->asPtr->isPreemptRequested() )
            this->preempt(btActionResult_);*/
        /// check if we have a pending preempt request from any source
        if(this->preemptRequested)
        {
            /// this flag will be set either by user (async) or by the actionServer via bt_preemptCallback() injection
            this->preempt(btActionResult_);
        }

        //else we returned with valid return value
        else
        {
            btActionResult_.status = result.getResultObj()->getResult();
            this->asPtr->setSucceeded(btActionResult_);
        }
    }
    else
    {
        /// from: http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
        /// uint8 REJECTED = 5 # The goal was rejected by the action server without being processed,
        //this->asPtr->setRejected();
        /// because we have allready defined a returnvalue before, well use that instead
        this->asPtr->setSucceeded(btActionResult_);
    }
    this->running = false;
    return;
}


void StsBehaviorTreeIfComAction::preempt(sts_virtual_interface_msgs::BTResult &btActionResult_)
{
    btActionResult_.status = 4;
    this->asPtr->setPreempted(btActionResult_);
    this->preemptRequested = false;
}

void StsBehaviorTreeIfComAction::reset()
{
    /// do not preempt actively, instead set the preempt flag of our action server to let the user callback "think" the action was preempted normally
    /// blocking:
    unsigned int timeout = 0;
    while(this->running == true)
    {
        /// do stuff
        this->bt_preemptCallback();
        /// check timeout
        if(timeout >= 100 )
        {
            sts_core::StsIO::PRINT_ERROR("ACTION RESET TIMEOUT OCCURED [sts_virtual_interfaces]");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout++;
    }
    /// nonblocking:
    //this->bt_preemptCallback();
}


/* ******************************************************* *
 * ****************** F A C T O R Y ********************** *
 * ******************************************************* */

btIfComPtr StsBehaviorTreeIfComFactory::createRosCom(std::string identifier,  sts_behavior_tree_if_types::COMTYPE comType, ros::NodeHandle* nodeHandlePtr, bool errorDefault, sts_behavior_tree_if_types::callbackFct userCallback)
{

    if (comType == sts_behavior_tree_if_types::TOPIC)
	{
		//create new COM object
		StsBehaviorTreeIfComTopic com = StsBehaviorTreeIfComTopic();
		//create sharedPtr
		std::shared_ptr<StsBehaviorTreeIfComTopic> sharedPtr = std::make_shared<StsBehaviorTreeIfComTopic>(com);
		//do initialization of the COM object here on the sharedPtr because of referencing problems with stack (ros::timer)
		sharedPtr->initialize(errorDefault, identifier, nodeHandlePtr, userCallback );

		return sharedPtr;
    }
    else if (comType == sts_behavior_tree_if_types::FLAG)
    {
        //create new COM object
        StsBehaviorTreeIfComFlag com = StsBehaviorTreeIfComFlag();
        //create sharedPtr
        std::shared_ptr<StsBehaviorTreeIfComFlag> sharedPtr = std::make_shared<StsBehaviorTreeIfComFlag>(com);
        //do initialization of the COM object here on the sharedPtr because of referencing problems with stack (ros::timer)
        sharedPtr->initialize(errorDefault, identifier, nodeHandlePtr, userCallback );

        return sharedPtr;
    }
    else if (comType == sts_behavior_tree_if_types::SERVICE)
    {
        //create new COM object
        StsBehaviorTreeIfComService com = StsBehaviorTreeIfComService();
        //create sharedPtr
        std::shared_ptr<StsBehaviorTreeIfComService> sharedPtr = std::make_shared<StsBehaviorTreeIfComService>(com);
        //do initialization of the COM object here on the sharedPtr because of referencing problems with stack (ros::timer)
        sharedPtr->initialize(errorDefault, identifier, nodeHandlePtr, userCallback );

        return sharedPtr;
    }
    else if (comType == sts_behavior_tree_if_types::ACTION)
    {
        //create new COM object
        StsBehaviorTreeIfComAction com = StsBehaviorTreeIfComAction();
        //create sharedPtr
        std::shared_ptr<StsBehaviorTreeIfComAction> sharedPtr = std::make_shared<StsBehaviorTreeIfComAction>(com);
        //do initialization of the COM object here on the sharedPtr because of referencing problems with stack (ros::timer)
        sharedPtr->initialize(errorDefault, identifier, nodeHandlePtr, userCallback );

        return sharedPtr;
    }
    else
        return NULL;
}

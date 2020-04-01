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

#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIf.h>


sts_core::sts_interfaces::StsBehaviorTreeInterface::StsBehaviorTreeInterface(ros::NodeHandle* nodeHandlePtr)
{
    //Important Base Class init
    if (!this->initialize(nodeHandlePtr))
    {
        this->setNOK();
    }
}

sts_core::sts_interfaces::StsBehaviorTreeInterface::~StsBehaviorTreeInterface() {}

//##############################
//## Managing ros com objects ##
//##############################
bool sts_core::sts_interfaces::StsBehaviorTreeInterface::createBtIfCom(std::string identifier, sts_behavior_tree_if_types::COMTYPE type, bool errorDefault, sts_behavior_tree_if_types::callbackFct userCallback)
{
    btIfComPtr comPtr = StsBehaviorTreeIfComFactory::createRosCom(identifier,type, this->getNodeHandle(), errorDefault, userCallback);
    this->btComMap.insert(std::pair<std::string, btIfComPtr>(identifier, comPtr));
    return true;
}

bool sts_core::sts_interfaces::StsBehaviorTreeInterface::createBtIfCom(std::string identifier, sts_behavior_tree_if_types::COMTYPE type, bool errorDefault)
{
    btIfComPtr comPtr = StsBehaviorTreeIfComFactory::createRosCom(identifier,type, this->getNodeHandle(), errorDefault, NULL);
    this->btComMap.insert(std::pair<std::string, btIfComPtr>(identifier, comPtr));
    return true;
}

bool sts_core::sts_interfaces::StsBehaviorTreeInterface::deleteBtIfCom(std::string id)
{
    this->btComMap.erase(id);
    return true;
}

bool sts_core::sts_interfaces::StsBehaviorTreeInterface::resetBtIfCom()
{ return this->resetBtIfCom_(std::string(""));}
bool sts_core::sts_interfaces::StsBehaviorTreeInterface::resetBtIfCom(std::string id)
{ return this->resetBtIfCom_(id);}
bool sts_core::sts_interfaces::StsBehaviorTreeInterface::resetBtIfCom_(std::string id)
{
    if(id.compare("") == 0)
    {
        /// in case we dont have an id, reset all
        std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.begin();
        for (;iter != this->btComMap.end(); iter++)
        {
            btIfComPtr sharedPtr = iter->second;
            sharedPtr->reset();
        }
    }
    else
    {
        /// in case we have an id, reset only this one
        std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
        if (iter != this->btComMap.end() )
        {
            btIfComPtr sharedPtr = iter->second;
            sharedPtr->reset();
        }

    }
    return true;
}


//##############################
//####### Set / Get API ########
//##############################
void sts_core::sts_interfaces::StsBehaviorTreeInterface::setFrequency(std::string id, double frequency)
{
    std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
    if (iter != this->btComMap.end() )
    {
        btIfComPtr sharedPtr = iter->second;
        sharedPtr->setFrequency(frequency);
    }
    return;
}
bool sts_core::sts_interfaces::StsBehaviorTreeInterface::sendAsync(std::string id, bool value)
{
    bool success = false;
    std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
    if (iter != this->btComMap.end() )
    {
        btIfComPtr sharedPtr = iter->second;
        StsBehaviorTreeIfComBase::COMAPIRETURN comReturn = sharedPtr->sendAsync(value);
        if(comReturn == StsBehaviorTreeIfComBase::COMAPIRETURN::ACK)
            success = true;
        else if(comReturn == StsBehaviorTreeIfComBase::COMAPIRETURN::NOTIMPLEMENTED)
            STS_WARNING("COM returned NOTIMPLEMENTED, sendAsync() can only be called on FLAG types!");
    }
    return success;
}


void sts_core::sts_interfaces::StsBehaviorTreeInterface::setDefaultBtIfComResult(std::string id, bool b)
{
    std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
    if (iter != this->btComMap.end() )
    {
        btIfComPtr sharedPtr = iter->second;
        sharedPtr->setDefaultResult(b);
    }
    return;
}

 bool sts_core::sts_interfaces::StsBehaviorTreeInterface::getDefaultBtIfComResult(std::string id)
{
     bool res = false;
     std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
     if (iter != this->btComMap.end() )
     {
         btIfComPtr sharedPtr = iter->second;
         res = sharedPtr->getDefaultResult();
     }
     return res;
}

void sts_core::sts_interfaces::StsBehaviorTreeInterface::setCallbackFct(std::string id, sts_behavior_tree_if_types::callbackFct cb)
{
    std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>>::iterator iter = this->btComMap.find(id);
    if (iter != this->btComMap.end() )
    {
        btIfComPtr sharedPtr = iter->second;
        sharedPtr->setCallbackFct(cb);
    }
    return;
}

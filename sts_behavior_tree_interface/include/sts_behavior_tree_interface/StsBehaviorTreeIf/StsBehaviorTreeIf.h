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

#include <sts_core_utils/sts_core/sts_base/sts_core_node.h>
#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIfTypes.h>
#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIfCom.h>

namespace sts_core {
namespace sts_interfaces {

class StsBehaviorTreeInterface : public virtual sts_core::sts_base::StsCoreNode
{
public:
    StsBehaviorTreeInterface(ros::NodeHandle* nodeHandlePtr);
    ~StsBehaviorTreeInterface();

    //####################################
    //############# USER API #############
    //####################################
protected:
    bool createBtIfCom(std::string id, sts_behavior_tree_if_types::COMTYPE type, bool errorDefault, sts_behavior_tree_if_types::callbackFct userCallback);
    bool createBtIfCom(std::string id, sts_behavior_tree_if_types::COMTYPE type, bool errorDefault);

    bool resetBtIfCom();
    bool resetBtIfCom(std::string id);


    bool deleteBtIfCom(std::string id);

    void setFrequency(std::string id, double frequency);
    bool sendAsync(std::string id, bool value);

    void setDefaultBtIfComResult(std::string id, bool b);
    bool getDefaultBtIfComResult(std::string id);

    void setCallbackFct(std::string id, sts_behavior_tree_if_types::callbackFct cb);

    //Attributes:
protected:
    std::map<std::string, std::shared_ptr<StsBehaviorTreeIfComBase>> btComMap;
private:
    bool resetBtIfCom_(std::string id);
};

} /* namespace */
} /* namespace */

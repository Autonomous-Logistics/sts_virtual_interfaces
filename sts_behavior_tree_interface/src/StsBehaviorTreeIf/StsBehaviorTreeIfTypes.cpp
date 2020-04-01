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

#include <sts_behavior_tree_interface/StsBehaviorTreeIf/StsBehaviorTreeIfTypes.h>

//###################################
//######### sts_bt_args #############
//###################################
sts_behavior_tree_if_types::StsBehaviorTreeArguments::StsBehaviorTreeArguments() : preemptRequest(NULL)
{}

sts_behavior_tree_if_types::StsBehaviorTreeArguments::~StsBehaviorTreeArguments()
{}

void sts_behavior_tree_if_types::StsBehaviorTreeArguments::setPreemptRequestReference(bool* reference)
{
    preemptRequest = reference;
}

bool& sts_behavior_tree_if_types::StsBehaviorTreeArguments::getPreemptRequestReference()
{
    return *this->preemptRequest;
}

//###################################
//######### sts_bt_result ###########
//###################################
sts_behavior_tree_if_types::StsBehaviorTreeResult::StsBehaviorTreeResult()
{
    this->setResult(false);
}

sts_behavior_tree_if_types::StsBehaviorTreeResult::StsBehaviorTreeResult(const StsBehaviorTreeResult *obj)
{
    this->result_ = obj->result_;
}

bool sts_behavior_tree_if_types::StsBehaviorTreeResult::isEqual(const StsBehaviorTreeResult *obj)
{
    if (this->result_ == obj->result_)
        return true;
    else
        return false;
}

//###################################
//######### sts_bt_param ############
//###################################
sts_behavior_tree_if_types::StsBehaviorTreeParam::StsBehaviorTreeParam() : argsObj(StsBehaviorTreeArguments()), resultObj(StsBehaviorTreeResult())
{}

sts_behavior_tree_if_types::StsBehaviorTreeParam::~StsBehaviorTreeParam()
{}

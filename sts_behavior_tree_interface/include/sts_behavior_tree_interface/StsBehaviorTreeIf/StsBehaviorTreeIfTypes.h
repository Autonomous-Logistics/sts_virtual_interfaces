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

#include <functional>

#include <sts_virtual_interface_msgs/BTAction.h>


class StsBehaviorTreeIfComAction; //Forward declaration for friend declaration

namespace sts_behavior_tree_if_types{

enum COMTYPE{TOPIC, FLAG, SERVICE, ACTION};

//###################################
//######### sts_bt_args #############
//###################################
class StsBehaviorTreeArguments
{
friend class ::StsBehaviorTreeIfComAction;

public:
    StsBehaviorTreeArguments();
    ~StsBehaviorTreeArguments();

    bool& getPreemptRequestReference();

protected:
    void setPreemptRequestReference(bool* reference);

private:
    bool* preemptRequest;
};

//###################################
//######### sts_bt_result ###########
//###################################
class StsBehaviorTreeResult
{
public:
    StsBehaviorTreeResult();
    StsBehaviorTreeResult(const StsBehaviorTreeResult* obj);

    ~StsBehaviorTreeResult() {}

    bool getResult(){return this->result_;}

    void setResult(bool b){this->result_ = b;}

    bool isEqual(const StsBehaviorTreeResult* obj);

private:
    bool result_;
};

//###################################
//######### sts_bt_param ############
//###################################
class StsBehaviorTreeParam
{
 public:
    StsBehaviorTreeParam();
    ~StsBehaviorTreeParam();

    StsBehaviorTreeArguments* getArgsObj(){return &this->argsObj;}
    StsBehaviorTreeResult* getResultObj(){return &this->resultObj;}

private:
    StsBehaviorTreeArguments argsObj;
    StsBehaviorTreeResult  resultObj;
};

typedef std::function<void(sts_behavior_tree_if_types::StsBehaviorTreeParam*)> callbackFct;

} //namespace

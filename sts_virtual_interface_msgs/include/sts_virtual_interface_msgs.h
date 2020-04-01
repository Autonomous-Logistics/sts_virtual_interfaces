#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <string>
#include <memory>

#include <boost/preprocessor.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/lexical_cast.hpp>


//interface _msgs specific local includes
#include <sts_virtual_interface_msgs/HealthCheck.h>
#include <sts_virtual_interface_msgs/HealthReset.h>
#include <sts_virtual_interface_msgs/HealthReport.h>

/// ******************************************************************************************************************

namespace sts_core {
namespace sts_interfaces {

///[fgn] default namespace for all behavior tree specific identifiers (topicnames etc)
const static std::string BEHAVIOR_TREE_NAMESPACE_KEY = "/sts_behavior_tree_namespace";
const static std::string BEHAVIOR_TREE_NAMESPACE = "/sts_behavior_tree";

class Report
{
public:
    template <class T1, class T2>
    Report(T1 humanMsg, T2 machineMsg){this->initialize(std::string(humanMsg), std::string(machineMsg));}
    //virtual ~Report() = default;
    //Report(const Report&) = default;
    //Report(Report&&) = default;

    friend std::ostream& operator<<(std::ostream &out, Report& r)
    { out << r.toString(); return out; }
    friend std::ostream& operator<<(std::ostream &out, Report* r)
    { out << r->toString(); return out; }
    virtual std::string toString(){
        std::ostringstream stringStream;
        stringStream << "Report: " << std::endl;
        stringStream << " - stamp: " << this->stamp_ << std::endl;
        stringStream << " - uuid: " << this->uuid_ << std::endl;
        stringStream << " - machine_message: " << this->machine_message_ << std::endl;
        stringStream << " - human_message: " << this->human_message_ << std::endl;
        std::string str = stringStream.str();
        return str;
    }

    Report* getBasePtr(){
        return this;
    }

    virtual sts_virtual_interface_msgs::HealthReport convertToRosMsg(){
        sts_virtual_interface_msgs::HealthReport r;
        this->convertToRosMsg(r);
        return r;
    }
    virtual void convertToRosMsg(sts_virtual_interface_msgs::HealthReport& r){
        r.stamp = this->stamp_;
        r.uuid = this->uuid_;
        r.sender = this->sender_;
        r.machine_message = this->machine_message_;
        r.human_message = this->human_message_;
    }

    void setSender(std::string sender){
        this->sender_ = sender;
    }


protected:
    virtual void initialize(std::string humanMsg, std::string machineMsg){
        this->stamp_ = ros::Time::now();
        this->uuid_ = this->generateUuid();
        this->human_message_ = humanMsg; this->machine_message_ = machineMsg;
    }

private:
    std::string generateUuid(){
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        return boost::lexical_cast<std::string>(uuid);
    };

public:
protected:
    ros::Time stamp_;
    std::string uuid_;
    std::string sender_;

    std::string human_message_;
    std::string machine_message_;
private:

};
/// consider:
/// forward declaration of class for shared ptr typedef
/// https://stackoverflow.com/questions/28386185/cant-use-stdunique-ptrt-with-t-being-a-forward-declaration
//class Report;
typedef std::shared_ptr<Report> ReportPtr;


} // end of namepsace sts_interfaces
} // end of namepsace sts_core

/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vector>
#include <string>

#include "rtt/RTT.hpp"
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "velma_core_cs_ve_body_msgs/Command.h"
#include "velma_core_cs_ve_body_msgs/StatusSC.h"
#include "velma_core_ve_body_re_body_msgs/Command.h"

using namespace RTT;

namespace velma_core_ve_body_types {

class BypassComponent: public RTT::TaskContext {
public:
    explicit BypassComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:

    velma_core_cs_ve_body_msgs::Command cmd_in_;
    RTT::InputPort<velma_core_cs_ve_body_msgs::Command > port_cmd_in_;

    velma_core_ve_body_re_body_msgs::Command cmd_out_;
    RTT::OutputPort<velma_core_ve_body_re_body_msgs::Command > port_cmd_out_;

    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;


    int diag_;
};

BypassComponent::BypassComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    port_cmd_in_("command_INPORT"),
    port_cmd_out_("command_OUTPORT")
{
    this->ports()->addPort(port_cmd_in_);
    this->ports()->addPort(port_cmd_out_);
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

    this->addOperation("getDiag", &BypassComponent::getDiag, this, RTT::ClientThread);
}

std::string BypassComponent::getDiag() {
// this method may not be RT-safe
    int diag = diag_;

    if (diag == 1) {
        return "could not read commands";
    }
    return "";
}

bool BypassComponent::configureHook() {
    return true;
}

bool BypassComponent::startHook() {
    return true;
}

void BypassComponent::stopHook() {
}

void BypassComponent::updateHook() {

    if (port_cmd_in_.read(cmd_in_) != RTT::NewData) {
        Logger::In in("BypassComponent::updateHook");
        Logger::log() << Logger::Error << "could not read data on port "
            << port_cmd_in_.getName() << Logger::endl;
        error();
        diag_ = 1;
        return;
    }

    diag_ = 0;

    cmd_out_ = velma_core_ve_body_re_body_msgs::Command();

//    cmd_out_.rTact = cmd_in_.rTact;
//    cmd_out_.rTact_valid = cmd_in_.rTact_valid;

    cmd_out_.tMotor = cmd_in_.tMotor;
    cmd_out_.tMotor_valid = cmd_in_.tMotor_valid;

    cmd_out_.hpMotor = cmd_in_.hpMotor;
    cmd_out_.hpMotor_valid = cmd_in_.hpMotor_valid;

    cmd_out_.htMotor = cmd_in_.htMotor;
    cmd_out_.htMotor_valid = cmd_in_.htMotor_valid;

    cmd_out_.lArm = cmd_in_.lArm;
    cmd_out_.lArm_valid = cmd_in_.lArm_valid;

    cmd_out_.rArm = cmd_in_.rArm;
    cmd_out_.rArm_valid = cmd_in_.rArm_valid;

//    cmd_out_.lHand = cmd_in_.lHand;
//    cmd_out_.lHand_valid = cmd_in_.lHand_valid;

//    cmd_out_.rHand = cmd_in_.rHand;
//    cmd_out_.rHand_valid = cmd_in_.rHand_valid;

    port_cmd_out_.write(cmd_out_);

    // no error
    sc_out_.safe_behavior = false;
    sc_out_.error = false;
    sc_out_.fault_type = 0;
    sc_out_.faulty_module_id = 0;

    port_sc_out_.write(sc_out_);
}

}   //namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::BypassComponent)


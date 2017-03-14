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

#include <string>

#include "rtt/RTT.hpp"
#include <rtt/Component.hpp>

using namespace RTT;

namespace velma_core_ve_body_types {

class PacketCounterComponent: public RTT::TaskContext {
public:
    explicit PacketCounterComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:

    uint32_t packet_counter_;
    RTT::OutputPort<uint32_t > port_packet_counter_out_;
};

PacketCounterComponent::PacketCounterComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    port_packet_counter_out_("packet_counter_OUTPORT"),
    packet_counter_(1)
{
    this->ports()->addPort(port_packet_counter_out_);
    this->addOperation("getDiag", &PacketCounterComponent::getDiag, this, RTT::ClientThread);
}

std::string PacketCounterComponent::getDiag() {
// this method may not be RT-safe
    std::stringstream ss;
    ss << packet_counter_;
    return ss.str();
}

bool PacketCounterComponent::configureHook() {
    return true;
}

bool PacketCounterComponent::startHook() {
    return true;
}

void PacketCounterComponent::stopHook() {
}

void PacketCounterComponent::updateHook() {
    ++packet_counter_;
    port_packet_counter_out_.write(packet_counter_);
}

}   // namespace velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::PacketCounterComponent)


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

#include "velma_core_ve_head/master.h"

namespace velma_core_ve_head_types {

class BehaviorTransparent : public BehaviorBase {

public:

    BehaviorTransparent() :
        BehaviorBase("behavior_velma_core_ve_head_transparent", "transparent")
    {
    }

    virtual bool checkErrorCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                ErrorCausePtr result) const
    {
        return false;
    }

    virtual bool checkStopCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components) const
    {
        return false;
    }
};

class StateTransparent : public StateBase {
public:
    StateTransparent() :
        StateBase("state_velma_core_ve_head_transparent", "transparent", "behavior_velma_core_ve_head_transparent")
    {
    }

    virtual bool checkInitialCondition(
                const boost::shared_ptr<InputData >& in_data,
                const std::vector<RTT::TaskContext*> &components,
                const std::string& prev_state_name,
                bool in_error) const
    {
        if (prev_state_name == "state_velma_core_ve_head_transparent") {
            return false;
        }

        return true;
    }
};

};  // namespace velma_core_ve_head_types

REGISTER_BEHAVIOR( velma_core_ve_head_types::BehaviorTransparent );

REGISTER_STATE( velma_core_ve_head_types::StateTransparent );


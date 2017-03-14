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

#include <sstream>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/base/PortInterface.hpp>
#include <kuka_lwr_fri/friComm.h>

#include "velma_core_cs_ve_body_msgs/Command.h"

#include "eigen_conversions/eigen_msg.h"

#include "velma_core_ve_body_re_body_msgs/Command.h"
#include "velma_core_ve_body_re_body_msgs/Status.h"

#include "velma_core_cs_ve_body_msgs/StatusSC.h"

#include <math.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <sys/time.h>

using namespace RTT;

static const int DIAG_R_ARM_INVALID = 0;
static const int DIAG_L_ARM_INVALID = 1;
static const int DIAG_T_MOTOR_INVALID = 2;
static const int DIAG_HP_MOTOR_INVALID = 3;
static const int DIAG_HT_MOTOR_INVALID = 4;
static const int DIAG_R_ARM_MONITOR_MODE = 5;
static const int DIAG_L_ARM_MONITOR_MODE = 6;
static const int DIAG_R_ARM_ERROR = 7;
static const int DIAG_L_ARM_ERROR = 8;

static int setBit(int bitfield, int bit_num, bool value) {
    if (value) {
        // set
        bitfield |= (1 << bit_num);
    }
    else {
        // clear
        bitfield &= ~(1 << bit_num);
    }
    return bitfield;
}

static bool getBit(int bitfield, int bit_num) {
    return ((bitfield & (1 << bit_num)) != 0)?true:false;
}

namespace velma_core_ve_body_types {

class SafeComponent: public RTT::TaskContext {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit SafeComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

private:
    bool isLwrOk(const velma_core_ve_body_re_body_msgs::StatusArmFriRobot& friRobot, const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf) const;

    const int arm_joints_count_;

    void calculateArmDampingTorque(const boost::array<double, 7> &joint_velocity,
        const std::vector<double> &damping_factors, boost::array<double, 7> &joint_torque_command);

    void calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command);

    bool isNaN(double d) const;
    bool isInLim(double d, double lo_lim, double hi_lim) const;

    // ROS parameters
	std::vector<double> l_arm_damping_factors_;
	std::vector<double> r_arm_damping_factors_;
	double torso_damping_factor_;

    std::vector<double> arm_q_limits_lo_;
    std::vector<double> arm_q_limits_hi_;
    std::vector<double> arm_dq_limits_;
    std::vector<double> arm_t_limits_;

    // port data
    velma_core_ve_body_re_body_msgs::Command cmd_out_;
    RTT::OutputPort<velma_core_ve_body_re_body_msgs::Command > port_cmd_out_;

    velma_core_ve_body_re_body_msgs::Status status_in_;
    RTT::InputPort<velma_core_ve_body_re_body_msgs::Status > port_status_in_;

    RTT::OutputPort<int >   diag_out_;

    velma_core_cs_ve_body_msgs::StatusSC sc_out_;
    RTT::OutputPort<velma_core_cs_ve_body_msgs::StatusSC> port_sc_out_;

    tFriIntfState       rArm_fri_state_;
    tFriRobotState      rArm_robot_state_;
    tFriIntfState       lArm_fri_state_;
    tFriRobotState      lArm_robot_state_;

    bool emergency_;

    int no_hw_error_counter_;

    bool allHwOk_;
    bool hwStatusValid_;
    bool readCmdData_;
    bool cmdValid_;


    uint32_t packet_counter_;
    ros::Time wall_time_prev_;

    struct timeval time_prev_;
    int diag_;

};

SafeComponent::SafeComponent(const std::string &name) :
    TaskContext(name, PreOperational),
    arm_joints_count_(7),
    diag_(0),
    torso_damping_factor_(-1)   // initialize with invalid value, should be later set to >= 0
{
//    this->ports()->addPort("command_INPORT", port_command_in_);
//    this->ports()->addPort("status_test_OUTPORT", port_status_test_out_);

    this->ports()->addPort("status_INPORT", port_status_in_);
    this->ports()->addPort("cmd_OUTPORT", port_cmd_out_);
    this->ports()->addPort("sc_OUTPORT", port_sc_out_);

    addProperty("l_arm_damping_factors", l_arm_damping_factors_);
    addProperty("r_arm_damping_factors", r_arm_damping_factors_);
    addProperty("torso_damping_factor", torso_damping_factor_);

    addProperty("arm_q_limits_lo", arm_q_limits_lo_);
    addProperty("arm_q_limits_hi", arm_q_limits_hi_);
    addProperty("arm_dq_limits", arm_dq_limits_);
    addProperty("arm_t_limits", arm_t_limits_);

    this->addOperation("getDiag", &SafeComponent::getDiag, this, RTT::ClientThread);
}

std::string SafeComponent::getDiag() {
// this method may not be RT-safe
    int diag = diag_;
    std::string result;
    if (getBit(diag, DIAG_R_ARM_INVALID)) {
        result += "R_ARM_INVALID ";
    }

    if (getBit(diag, DIAG_L_ARM_INVALID)) {
        result += "L_ARM_INVALID ";
    }

    if (getBit(diag, DIAG_T_MOTOR_INVALID)) {
        result += "T_MOTOR_INVALID ";
    }

    if (getBit(diag, DIAG_HP_MOTOR_INVALID)) {
        result += "HP_MOTOR_INVALID ";
    }

    if (getBit(diag, DIAG_HT_MOTOR_INVALID)) {
        result += "HT_MOTOR_INVALID ";
    }

    if (getBit(diag, DIAG_R_ARM_MONITOR_MODE)) {
        result += "R_ARM_MONITOR_MODE ";
    }

    if (getBit(diag, DIAG_L_ARM_MONITOR_MODE)) {
        result += "L_ARM_MONITOR_MODE ";
    }

    if (getBit(diag, DIAG_R_ARM_ERROR)) {
        result += "R_ARM_ERROR ";
    }

    if (getBit(diag, DIAG_L_ARM_ERROR)) {
        result += "L_ARM_ERROR ";
    }
    return result;
}

bool SafeComponent::configureHook() {
    Logger::In in("SafeComponent::configureHook");

    if (l_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter l_arm_damping_factors is set to illegal value (wrong vector size: " <<
            l_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    if (r_arm_damping_factors_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter r_arm_damping_factors is set to illegal value (wrong vector size: " <<
            r_arm_damping_factors_.size() << ")" << Logger::endl;
        return false;
    }

    for (int i = 0; i < arm_joints_count_; ++i) {
        if (l_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter l_arm_damping_factors[" << i << "] is set to illegal value: " <<
                l_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
        if (r_arm_damping_factors_[i] < 0) {
            Logger::log() << Logger::Error <<
                "parameter r_arm_damping_factors[" << i << "] is set to illegal value: " <<
                r_arm_damping_factors_[i] << Logger::endl;
            return false;
        }
    }

    if (torso_damping_factor_ < 0) {
        Logger::log() << Logger::Error <<
            "parameter torso_damping_factor is set to illegal value: " <<
            torso_damping_factor_ << Logger::endl;
        return false;
    }

    if (arm_q_limits_lo_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_lo is set to illegal value (wrong vector size: " <<
            arm_q_limits_lo_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_q_limits_hi_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_q_limits_hi is set to illegal value (wrong vector size: " <<
            arm_q_limits_hi_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_dq_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_dq_limits is set to illegal value (wrong vector size: " <<
            arm_dq_limits_.size() << ")" << Logger::endl;
        return false;
    }

    if (arm_t_limits_.size() != arm_joints_count_) {
        Logger::log() << Logger::Error <<
            "parameter arm_t_limits is set to illegal value (wrong vector size: " <<
            arm_t_limits_.size() << ")" << Logger::endl;
        return false;
    }

    return true;
}

bool SafeComponent::startHook() {
    return true;
}

void SafeComponent::stopHook() {
}

void SafeComponent::calculateArmDampingTorque(const boost::array<double, 7> &joint_velocity,
    const std::vector<double> &damping_factors, boost::array<double, 7> &joint_torque_command)
{
    for (int i = 0; i < arm_joints_count_; ++i) {
        joint_torque_command[i] = -damping_factors[i] * joint_velocity[i];
    }
}

void SafeComponent::calculateTorsoDampingTorque(double motor_velocity, double &motor_current_command)
{
    const double torso_gear = 158.0;
    const double torso_trans_mult = 20000.0 * torso_gear / (M_PI * 2.0);
    const double torso_motor_constant = 0.00105;
    double joint_velocity = motor_velocity / torso_trans_mult;
    double motor_torque_command = -torso_damping_factor_ * joint_velocity;
    motor_current_command = motor_torque_command / torso_gear / torso_motor_constant;
}

bool SafeComponent::isLwrOk(const velma_core_ve_body_re_body_msgs::StatusArmFriRobot& friRobot, const velma_core_ve_body_re_body_msgs::StatusArmFriIntf& friIntf) const {
    if (friRobot.power != 0x7F                           // error
        || friRobot.error != 0                           // error
        || friRobot.warning != 0                         // TODO: check if this is error
        || friRobot.control != FRI_CTRL_JNT_IMP          // error
        || friIntf.quality <= FRI_QUALITY_UNACCEPTABLE)    // error
    {
        return false;
    }
    return true;
}

void SafeComponent::updateHook() {
    cmd_out_ = velma_core_ve_body_re_body_msgs::Command();

    //
    // read HW status
    //
    bool rArm_valid_prev = status_in_.rArm_valid;
    bool lArm_valid_prev = status_in_.lArm_valid;

    if (port_status_in_.read(status_in_) != RTT::NewData) {
        status_in_ = velma_core_ve_body_re_body_msgs::Status();
    }


    // as FRI components are not synchronized, their communication status should
    // be checked in two last cycles
    bool rArm_valid = rArm_valid_prev || status_in_.rArm_valid;
    bool lArm_valid = lArm_valid_prev || status_in_.lArm_valid;

    // check FRI and LWR state
    // as FRI components may not be synchronized
    bool tMotor_valid = status_in_.tMotor_valid;
    bool hpMotor_valid = status_in_.hpMotor_valid;
    bool htMotor_valid = status_in_.htMotor_valid;

/*
// TODO
    if (port_rArm_fri_state_in_.read(rArm_fri_state_) == RTT::NewData && port_rArm_robot_state_in_.read(rArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(rArm_fri_state_, rArm_robot_state_) ) {
            rArm_valid = false;
        }
    }
    if (port_lArm_fri_state_in_.read(lArm_fri_state_) == RTT::NewData && port_lArm_robot_state_in_.read(lArm_robot_state_) == RTT::NewData) {
        if ( !isLwrOk(lArm_fri_state_, lArm_robot_state_) ) {
            lArm_valid = false;
        }
    }

    if ( !isStatusValid(status_in_.rArm, id_faulty_submodule) ) {
        rArm_valid = false;
    }
    if ( !isStatusValid(status_in_.lArm, id_faulty_submodule) ) {
        lArm_valid = false;
    }

    if ( !status_ports_in_.isValid("rHand")) {
        rHand_valid = false;
    }

    if ( !isStatusValid(status_in_.rHand, id_faulty_submodule) ) {
        rHand_valid = false;
    }

    if ( !status_ports_in_.isValid("lHand")) {
        lHand_valid = false;
    }

    if ( !isStatusValid(status_in_.lHand, id_faulty_submodule) ) {
        lHand_valid = false;
    }

    if ( !status_ports_in_.isValid("tMotor")) {
        tMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.tMotor, id_faulty_submodule) ) {
        tMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("hpMotor")) {
        hpMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.hpMotor, id_faulty_submodule) ) {
        hpMotor_valid = false;
    }

    if ( !status_ports_in_.isValid("htMotor")) {
        htMotor_valid = false;
    }

    if ( !isStatusValid(status_in_.htMotor, id_faulty_submodule) ) {
        htMotor_valid = false;
    }
*/
    allHwOk_ =  rArm_valid      && lArm_valid &&
                tMotor_valid    && hpMotor_valid &&
                htMotor_valid;
/*
    //
    // read commands
    //
    readCmdData_ = (port_command_in_.read(cmd_in_) == NewData);
    cmdValid_ = false;
    if (!readCmdData_) {
        setFault(status_sc_out_, StatusSC::FAULT_COMM_UP, 0, 0);
    }
    else if (!isCommandValid(cmd_in_, id_faulty_module, id_faulty_submodule)) {
        setFault(status_sc_out_, StatusSC::FAULT_NAN_COMMAND, id_faulty_module, id_faulty_submodule);
    }
    else if (StatusSC::STATE_HW_DOWN != state_ && cmd_in_.test != packet_counter_) {
        setFault(status_sc_out_, StatusSC::FAULT_COMM_PACKET_LOST, 0, 0);
    }
    else {
        cmdValid_ = true;
    }
*/

    //
    // write HW commands to available devices
    //
/*
            cmd_out_.rArmFri.data
            cmd_out_.lArmFri.data
        cmd_out_.rArm.t
        cmd_out_.lArm.t
        cmd_out_.tMotor.i

        status_in_.rArm.dq
        status_in_.lArm.dq
        status_in_.tMotor.dq
        status_in_.hpMotor.q
        status_in_.htMotor.q
        status_in_.rArmFriRobot
        status_in_.rArmFriIntf.state

        status_in_.lArmFriRobot
        status_in_.lArmFriIntf.state
*/
    // generate safe outputs for all operational devices
    if (rArm_valid) {
        calculateArmDampingTorque(status_in_.rArm.dq, r_arm_damping_factors_, cmd_out_.rArm.t);
    }
    cmd_out_.rArm_valid = rArm_valid;

    if (lArm_valid) {
        calculateArmDampingTorque(status_in_.lArm.dq, l_arm_damping_factors_, cmd_out_.lArm.t);
    }
    cmd_out_.lArm_valid = lArm_valid;

    if (tMotor_valid) {
        calculateTorsoDampingTorque(status_in_.tMotor.dq, cmd_out_.tMotor.i);
        cmd_out_.tMotor.q = 0;
        cmd_out_.tMotor.dq = 0;
    }
    cmd_out_.tMotor_valid = tMotor_valid;

    if (hpMotor_valid) {
        cmd_out_.hpMotor.i = 0;
        cmd_out_.hpMotor.q = status_in_.hpMotor.q;
        cmd_out_.hpMotor.dq = 0;
    }
    cmd_out_.hpMotor_valid = hpMotor_valid;

    if (htMotor_valid) {
        cmd_out_.htMotor.i = 0;
        cmd_out_.htMotor.q = status_in_.htMotor.q;
        cmd_out_.htMotor.dq = 0;
    }
    cmd_out_.htMotor_valid = htMotor_valid;

    // KUKA LWR fri commands
    cmd_out_.rArmFri_valid = false;
    if (rArm_valid) {
        bool lwrOk = isLwrOk(status_in_.rArmFriRobot, status_in_.rArmFriIntf);

        // try to switch LWR mode
        if (lwrOk && status_in_.rArmFriIntf.state == FRI_STATE_MON) {
            cmd_out_.rArmFri.data = 1;
            cmd_out_.rArmFri_valid = true;
        }

        // diagnostics
        diag_ = setBit(diag_, DIAG_R_ARM_ERROR, !lwrOk);
        diag_ = setBit(diag_, DIAG_R_ARM_MONITOR_MODE, status_in_.rArmFriIntf.state == FRI_STATE_MON);
    }

    cmd_out_.lArmFri_valid = false;
    if (lArm_valid) {
        bool lwrOk = isLwrOk(status_in_.lArmFriRobot, status_in_.lArmFriIntf);

        // try to switch LWR mode
        if (lwrOk && status_in_.lArmFriIntf.state == FRI_STATE_MON) {
            cmd_out_.lArmFri.data = 1;
            cmd_out_.lArmFri_valid = true;
        }

        // diagnostics
        diag_ = setBit(diag_, DIAG_L_ARM_ERROR, !lwrOk);
        diag_ = setBit(diag_, DIAG_L_ARM_MONITOR_MODE, status_in_.lArmFriIntf.state == FRI_STATE_MON);
    }

    //
    // write commands
    //
    port_cmd_out_.write(cmd_out_);

    //
    // write diagnostics information for this subsystem
    //
    diag_ = setBit(diag_, DIAG_R_ARM_INVALID, !rArm_valid);
    diag_ = setBit(diag_, DIAG_L_ARM_INVALID, !lArm_valid);
    diag_ = setBit(diag_, DIAG_T_MOTOR_INVALID, !tMotor_valid);
    diag_ = setBit(diag_, DIAG_HP_MOTOR_INVALID, !hpMotor_valid);
    diag_ = setBit(diag_, DIAG_HT_MOTOR_INVALID, !htMotor_valid);

    //
    // write diagnostics information for other subsystems
    //
    sc_out_.safe_behavior = true;

    sc_out_.fault_type = 0;

    setBit(sc_out_.fault_type, velma_core_cs_ve_body_msgs::StatusSC::FAULT_COMM_HW,
        getBit(diag_, DIAG_R_ARM_INVALID) || getBit(diag_, DIAG_L_ARM_INVALID) ||
        getBit(diag_, DIAG_T_MOTOR_INVALID) || getBit(diag_, DIAG_HP_MOTOR_INVALID) ||
        getBit(diag_, DIAG_HT_MOTOR_INVALID));


    setBit(sc_out_.fault_type, velma_core_cs_ve_body_msgs::StatusSC::FAULT_HW_STATE,
        getBit(diag_, DIAG_R_ARM_ERROR) || getBit(diag_, DIAG_L_ARM_ERROR) ||
        getBit(diag_, DIAG_R_ARM_MONITOR_MODE) || getBit(diag_, DIAG_L_ARM_MONITOR_MODE));

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_R_ARM,
        getBit(diag_, DIAG_R_ARM_INVALID) || getBit(diag_, DIAG_R_ARM_ERROR) ||
        getBit(diag_, DIAG_R_ARM_MONITOR_MODE));

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_L_ARM,
        getBit(diag_, DIAG_L_ARM_INVALID) || getBit(diag_, DIAG_L_ARM_ERROR) ||
        getBit(diag_, DIAG_L_ARM_MONITOR_MODE));

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_T_MOTOR,
        getBit(diag_, DIAG_T_MOTOR_INVALID));

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HP_MOTOR,
        getBit(diag_, DIAG_HP_MOTOR_INVALID));

    setBit(sc_out_.faulty_module_id, velma_core_cs_ve_body_msgs::StatusSC::MODULE_HT_MOTOR,
        getBit(diag_, DIAG_HT_MOTOR_INVALID));

    sc_out_.error = (sc_out_.fault_type != 0);

// TODO: diagnostics for:
//MODULE_R_HAND
//MODULE_L_HAND
//MODULE_R_FT
//MODULE_L_FT
//MODULE_R_TACTILE
//MODULE_L_OPTOFORCE

    // write diagnostic data
    port_sc_out_.write(sc_out_);
}

}   // velma_core_ve_body_types

ORO_LIST_COMPONENT_TYPE(velma_core_ve_body_types::SafeComponent)


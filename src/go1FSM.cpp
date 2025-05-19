#include <chrono>

#include "go1_cpp_cmake/go1FSM.h"

// Constructor
go1FSM::go1FSM(double state_hz, 
                double mpc_hz,
                std::unique_ptr<lowLevelDataReader> data_interface,
                std::unique_ptr<go1StateEstimator> estimator,
                std::unique_ptr<lowLevelCommandSender> command_sender)
    : data_interface_(std::move(data_interface)),
        command_sender_(std::move(command_sender)),
        estimator_(std::move(estimator)),
        state_(),
        mpc_(),
        current_(go1FiniteState::Passive),
        mpc_counter_(0),
        mpc_interval_(static_cast<int>(mpc_hz / state_hz) - 1)
{
    if (dynamic_cast<mujocoDataReader*>(data_interface_.get())) {
        state_.thresh = MUJOCO_CONTACT_THRESH;
    } else {
        state_.thresh = UNITREE_SDK_CONTACT_THRESH;
    }
}

// Destructor
go1FSM::~go1FSM() {
    command_sender_->setMotorsToDamping(true);
}

void go1FSM::requestStartup() {
    want_shutdown_ = false;
    want_walk_ = false;
    current_ = go1FiniteState::Startup;
}

void go1FSM::requestWalk(bool enable) {
    want_walk_ = enable;
}

void go1FSM::requestShutdown() {
    want_shutdown_ = true;
    want_walk_ = false;
}

go1FSM::go1FiniteState go1FSM::getFiniteState() const {
    return current_;
}

const char* go1FSM::go1FiniteState2Str() {
    go1FiniteState finiteState = getFiniteState();
    switch(finiteState) {
        case go1FiniteState::Passive: return "Passive";
        case go1FiniteState::Startup: return "Startup";
        case go1FiniteState::Locomotion: return "Locomotion";
        case go1FiniteState::Shutdown: return "Shutdown";
        default: return "Unknown, am I cooked chat?";
    }
}

void go1FSM::setDesiredVel(const Eigen::Vector3d &lin_vel_cmd, double &yaw_cmd) {
    state_.root_lin_vel_d = lin_vel_cmd;
    state_.root_ang_vel_d(2) = yaw_cmd;
}

void go1FSM::setDesiredPos() {
    state_.root_pos_d += state_.root_lin_vel_d * DT_CTRL;
    auto rpy = state_.root_rpy_d;
    double new_yaw = rpy(2) + state_.root_ang_vel_d(2) * DT_CTRL;
    new_yaw = std::atan2(std::sin(new_yaw), std::cos(new_yaw));
    rpy(2) = new_yaw;
    state_.root_rpy_d = rpy; 
}

void go1FSM::step() {
    just_transitioned_to_locomotion_ = false;
    data_interface_->pullSensorData(state_);
    estimator_->estimateState(state_);

    // First, check if want to shutdown
    if (want_shutdown_) current_ = go1FiniteState::Shutdown;

    // Check finite states for corresponding actions
    switch (current_) {
        case go1FiniteState::Passive:
            command_sender_->setMotorsToDamping(true);
            state_.joint_pos_d = state_.joint_pos;
            state_.joint_vel_d = state_.joint_vel;
            state_.joint_torques_swing.setZero();
            state_.joint_torques.setZero();
            state_.joint_torques_stacked.setZero();
            break;

        case go1FiniteState::Startup:
            command_sender_->setMotorsToDamping(false);
            state_.squat_flag = true;
            state_.computeStartupPD();
            
            if (state_.isStartupComplete()) {
                current_ = go1FiniteState::Locomotion;
                Eigen::Vector3d pos_ctrl = USE_EST_FOR_CONTROL ? state_.root_pos_est : state_.root_pos;
                state_.root_pos_d << pos_ctrl(0), pos_ctrl(1), WALK_HEIGHT; // for sim, treadmill XML requires 1 + WALK_HEIGHT meters
                just_transitioned_to_locomotion_ = true;
                state_.squat_flag = false;
            }
            break;

        case go1FiniteState::Locomotion:
            if (want_walk_) {
                state_.setWalkingMode(want_walk_);

            } else if (!want_walk_ && 
                       (state_.swing_phase == SWING_PHASE_MAX/2 || state_.swing_phase == SWING_PHASE_MAX)) {
                state_.setWalkingMode(want_walk_);
                state_.foot_forces_swing.setZero();
            }

            state_.updateLocomotionPlan();
            break;

        case go1FiniteState::Shutdown:
            state_.squat_flag = true;
            state_.computeShutdownPD();

            if (state_.isShutdownComplete()) {
                current_ = go1FiniteState::Passive;
                Eigen::Vector3d pos_ctrl = USE_EST_FOR_CONTROL ? state_.root_pos_est : state_.root_pos;
                state_.root_pos_d << pos_ctrl(0), pos_ctrl(1), 0.0; // for sim, treadmill XML requires 1 meters
                state_.squat_flag = false;
            }
            break;
    }

    if (current_ == go1FiniteState::Locomotion) {
        // Iterate mpc_counter and only solve if at 10th tick (50 Hz)
        if (mpc_counter_++ >= mpc_interval_ || just_transitioned_to_locomotion_) {
            mpc_counter_ = 0;
            mpc_.solveMPCForState(state_);
        }
        
        state_.convertForcesToTorques();
    }

    command_sender_->setCommand(state_);
}
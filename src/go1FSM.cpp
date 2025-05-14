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
        current_(go1FiniteState::Startup),
        mpc_counter_(0),
        mpc_interval_(static_cast<int>(std::round(state_hz / mpc_hz)))
{
    mpc_thread_running_ = true;
    snapshot_ready_ = false;
    snapshot_grf_populated_ = false;
    mpc_thread_ = std::thread(&go1FSM::mpcLoop, this);
}

// Destructor
go1FSM::~go1FSM() {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        mpc_thread_running_ = false;
        snapshot_ready_ = true;
        cv_.notify_one();
    }

    if (mpc_thread_.joinable()) mpc_thread_.join();
}

void go1FSM::requestStartup() {
    std::lock_guard<std::mutex> lock(mtx_);
    want_shutdown_ = false;
    want_walk_ = false;
    current_ = go1FiniteState::Startup;
}

void go1FSM::requestWalk(bool enable) {
    std::lock_guard<std::mutex> lock(mtx_);
    want_walk_ = enable;
}

void go1FSM::requestShutdown() {
    std::lock_guard<std::mutex> lock(mtx_);
    want_shutdown_ = true;
    want_walk_ = false;
}

go1FSM::go1FiniteState go1FSM::getFiniteState() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return current_;
}

const char* go1FSM::go1FiniteState2Str() {
    go1FiniteState finiteState = getFiniteState();
    switch(finiteState) {
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

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (want_shutdown_) current_ = go1FiniteState::Shutdown;
    }

    switch (current_) {
        case go1FiniteState::Startup:
            state_.computeStartupPD();
            
            if (state_.isStartupComplete()) {
                std::lock_guard<std::mutex> lock(mtx_);
                current_ = go1FiniteState::Locomotion;
                state_.root_pos_d << 0, 0, WALK_HEIGHT; // careful for treadmill & hardware here
                just_transitioned_to_locomotion_ = true;
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
            state_.computeShutdownPD();
            break;
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (snapshot_grf_populated_ && current_ == go1FiniteState::Locomotion) {
            state_.retrieveGRF(pending_snapshot_.grf_forces);
            snapshot_grf_populated_ = false;
            state_.convertForcesToTorques();
        }
    }

    if (current_ == go1FiniteState::Locomotion && (mpc_counter_++ >= mpc_interval_ || just_transitioned_to_locomotion_)) {
        mpc_counter_ = 0;
        std::lock_guard<std::mutex> lock(mtx_);
        pending_snapshot_ = state_.getSnapshot();
        snapshot_ready_ = true;
        cv_.notify_one();
    }

    command_sender_->sendCommand(state_);

}

void go1FSM::mpcLoop() {
    std::unique_lock<std::mutex> lock(mtx_);
    while (mpc_thread_running_) {
        cv_.wait(lock, [this]() { return snapshot_ready_; });
        if (!mpc_thread_running_) break;

        snapshot_ready_ = false;
        snapshot_grf_populated_ = false;
        lock.unlock();

        mpc_.solveMPCFromSnapshot(pending_snapshot_);

        lock.lock();
        snapshot_grf_populated_ = true;
    }
}
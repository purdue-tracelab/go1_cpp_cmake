#include <chrono>

#include "go1_cpp_cmake/go1FSM.h"

go1FSM::go1FSM(go1State &state) : state_(state), mpc_() {}

go1FSM::~go1FSM() {
    stop();
    if (thr_.joinable()) thr_.join();
}

void go1FSM::start() {
    {
        std::lock_guard lk(mtx_);
        if (running_) return;
        running_ = true;
        current_ = go1FiniteState::Startup;
    }
    thr_ = std::thread(&go1FSM::runLoop, this);
}

void go1FSM::stop() {
    {
        std::lock_guard lk(mtx_);
        running_ = false;
        want_walk_ = false;
    }
    cv_.notify_all();
}

void go1FSM::requestWalk(bool enable) {
    {
        std::lock_guard lk(mtx_);
        want_walk_ = enable;
    }
    cv_.notify_all();
}

void go1FSM::requestShutdown() {
    {
        std::lock_guard lk(mtx_);
        want_shutdown_ = true;
        want_walk_ = false;
    }
    cv_.notify_all();
}

go1FiniteState go1FSM::getFiniteState() const {
    std::lock_guard lk(mtx_);
    return current_;
}

void go1FSM::runLoop() {
    std::unique_lock lk(mtx_);
    while (running_) {
        switch (current_) {
            case go1FiniteState::Startup: handleStartup(lk); break;
            case go1FiniteState::Locomotion: handleLocomotion(lk); break;
            case go1FiniteState::Shutdown: handleShutdown(lk); break;
        }
    }
}

void go1FSM::handleStartup(std::unique_lock<std::mutex>& lk) {
    lk.unlock();

    const auto dt = std::chrono::milliseconds(2); // DT_CTRL = 0.002 s -> 500 Hz
    auto next_tick = std::chrono::steady_clock::now();

    while (running_ && !state_.isStartupComplete()) {
        {
            std::lock_guard lk2(mtx_);
            if (want_shutdown_) break;
        }

        next_tick += dt;
        auto now = std::chrono::steady_clock::now();

        if (next_tick > now)
            std::this_thread::sleep_for(next_tick - now);
    }

    lk.lock();

    if (want_shutdown_) {
        current_ = go1FiniteState::Shutdown;
        return;
    }

    if (!running_) return;
    current_ = go1FiniteState::Locomotion;
    state_.root_pos_d << 0, 0, WALK_HEIGHT; // careful for treadmill & hardware here
}

void go1FSM::handleLocomotion(std::unique_lock<std::mutex>& lk) {
    lk.unlock();

    const auto tick_dt = std::chrono::milliseconds(2);
    const auto mpc_dt = std::chrono::milliseconds(20);

    auto next_tick = std::chrono::steady_clock::now();
    auto next_mpc = next_tick;

    while (1) {
        auto now = std::chrono::steady_clock::now();

        {
            std::lock_guard lk2(mtx_);
            if (!running_ || want_shutdown_) {
                current_ = go1FiniteState::Shutdown;
                break;
            }

            if (want_walk_)
                state_.setWalkingMode(true);
            else {
                if (state_.swing_phase == SWING_PHASE_MAX/2 || state_.swing_phase == SWING_PHASE_MAX) {
                    state_.setWalkingMode(false);
                    state_.foot_forces_swing.setZero();
                }
            }
        }

        if (now >= next_mpc) {
            auto snap = state_.getSnapshot();
            mpc_.solveMPCFromSnapshot(snap);
            state_.setGRFForces(snap.grf_forces);
            next_mpc += mpc_dt;
        }

        next_tick += tick_dt;
        auto sleep_dur = next_tick - std::chrono::steady_clock::now();
        if (sleep_dur > std::chrono::milliseconds::zero())
            std::this_thread::sleep_for(sleep_dur);
    }

    lk.lock();
}

void go1FSM::handleShutdown(std::unique_lock<std::mutex>& lk) {
    lk.unlock();

    const auto dt = std::chrono::milliseconds(2);
    auto next_tick = std::chrono::steady_clock::now();

    while(!state_.isShutdownComplete()) {
        next_tick += dt;
        auto now = std::chrono::steady_clock::now();

        if (next_tick > now)
            std::this_thread::sleep_for(next_tick - now);
    }

    lk.lock();
    running_ = false;
}
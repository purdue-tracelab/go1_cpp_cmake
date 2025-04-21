#ifndef GO1_FSM_H
#define GO1_FSM_H

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "go1State.h"
#include "go1StanceMPC.h"

enum class go1FiniteState {
    Startup,
    Locomotion,
    Shutdown
};

class go1FSM {
    public:
        explicit go1FSM(go1State &state);
        ~go1FSM(); // destructor?

        void start();
        void stop();
        void requestWalk(bool enable = true);
        void requestShutdown();
        go1FiniteState getFiniteState() const;

    private:
        void runLoop();

        // explicit mode handlers
        void handleStartup(std::unique_lock<std::mutex>& lk);
        void handleLocomotion(std::unique_lock<std::mutex>& lk);
        void handleShutdown(std::unique_lock<std::mutex>& lk);

        go1State &state_;
        go1MPC mpc_;

        std::thread thr_;
        mutable std::mutex mtx_;
        std::condition_variable cv_;

        bool running_{false};
        bool want_walk_{false};
        bool want_shutdown_{false};
        go1FiniteState current_{go1FiniteState::Startup};

};

#endif // GO1_FSM_H
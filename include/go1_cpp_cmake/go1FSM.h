#ifndef GO1_FSM_H
#define GO1_FSM_H

#include <thread>
#include <atomic>
#include <condition_variable>
#include <memory>

#include "go1State.h"
#include "go1StanceMPC.h"
#include "go1StateEstimator.h"
#include "go1DataInterface.h"

class go1FSM {
    public:
        enum class go1FiniteState { Passive, Startup, Locomotion, Shutdown };

        go1FSM(double state_hz, 
                double mpc_hz, 
                std::unique_ptr<lowLevelDataReader> data_interface, 
                std::unique_ptr<go1StateEstimator> estimator,
                std::unique_ptr<lowLevelCommandSender> command_sender);
        ~go1FSM();

        void step();
        void requestStartup();
        void requestWalk(bool enable = true);
        void requestShutdown();
        go1FiniteState getFiniteState() const;
        const char* go1FiniteState2Str();
        const go1State& getState() const { return state_; }
        Eigen::VectorXd getMeasurement() const { return estimator_->getMeasurement(); }
        Eigen::VectorXd getPrediction() const { return estimator_->getPrediction(); }
        Eigen::VectorXd getPostFitResidual() const { return estimator_->getPostFitResidual(); }
        Eigen::VectorXd getPostFitPrediction() const {return estimator_->getPostFitPrediction(); }
        void setDesiredPos();
        void setDesiredVel(const Eigen::Vector3d &lin_vel_cmd, double &yaw_cmd);

        void collectInitialState() {
            data_interface_->pullSensorData(state_);
            estimator_->collectInitialState(state_);
        }

    private:
        // Core controller objects
        std::unique_ptr<lowLevelDataReader> data_interface_;
        std::unique_ptr<lowLevelCommandSender> command_sender_;
        std::unique_ptr<go1StateEstimator> estimator_;
        go1State state_;
        go1MPC mpc_;
        
        // FSM state management
        go1FiniteState current_{go1FiniteState::Startup};
        int mpc_counter_, mpc_interval_;
        bool want_walk_{false};
        bool want_shutdown_{false};
        bool just_transitioned_to_locomotion_{false};
};

#endif // GO1_FSM_H
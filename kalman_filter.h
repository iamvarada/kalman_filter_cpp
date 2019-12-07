/// Basic implementation of a Kalman filter in C++
/// Developer : Krishna Varadarajan

#pragma once

#include <Eigen/Dense>
#include <iostream>

/// Class for a basic kalman filter implementation
class BasicKalmanFilter {
    public : 
        /// Constructor w/intiializer list
        BasicKalmanFilter(
            const Eigen::MatrixXd &A, 
            const Eigen::MatrixXd &C,
            const Eigen::MatrixXd &Q,  
            const Eigen::MatrixXd &R,
            Eigen::MatrixXd &P
        );

        /// Initialize the filter zero initial states
        void initialize_filter();

        /// Overload the initialize_filter function with non-zero initial states
        /// \param[in] t_init start time of the filter (if needs to be set other than zero(0))
        /// \param[in] init_states initial states of the filter
        void initialize_filter(double t_init, const Eigen::VectorXd &init_states);

        /// Predict step for the filter when a measurement is received
        void predict_filter_step();

        /// Update step for the filter when a measurement is received
        /// \param[in] : y incoming measurement
        void update_filter_step(const Eigen::VectorXd &y);

        /// Return the latest estimated-state value
        /// \return latest value of estimated state 
        const Eigen::VectorXd get_latest_estimated_state() const&;

        /// Return current time
        /// \return current process time
        const double get_current_time() const&;

    private :
        /// time-step
        double dt_; 

        /// Initial time
        double t_initial_;

        /// Current time
        double t_current_;

        /// Latest time at which predication step was performed
        double last_predict_step_time;

        /// Latest time at which update step was performed
        double last_update_step_time;

        /// System equation : X' = A.X ; Y = C.X 
        /// \A : system dynamics matrix
        /// \C : output matrix
        /// \Q : process noise covariance
        /// \R : measurement noise covariance
        /// \P_init_ : initial estimate error covariance
        /// \P_ : estimate error covariance
        /// \K_gain_ : Kalman gain
        const Eigen::MatrixXd A_;
        const Eigen::MatrixXd C_;
        const Eigen::MatrixXd Q_;
        const Eigen::MatrixXd R_;
        Eigen::MatrixXd P_init_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd K_gain_;

        /// System dimensions
        const std::size_t num_states_;
        const std::size_t num_outputs_;

        /// Estimates states : priori, posteriori
        Eigen::VectorXd x_hat_priori_;
        Eigen::VectorXd x_hat_posteriori_;

        /// Flag to check if the filter is initialized
        bool filter_initialized_;
};  

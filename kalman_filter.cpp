/// Basic implementation of a Kalman filter in C++
/// Developer : Krishna Varadarajan

#include "kalman_filter.h"

BasicKalmanFilter::BasicKalmanFilter (
    const Eigen::MatrixXd &A, 
    const Eigen::MatrixXd &C,
    const Eigen::MatrixXd &Q,  
    const Eigen::MatrixXd &R,
    Eigen::MatrixXd &P,
    const double dt
    ) : A_(A), C_(C), Q_(Q), R_(R), P_init_(P), num_states_(A.rows()), num_outputs_(C.rows()), x_hat_posteriori_(num_states_), x_hat_priori_(num_states_), filter_initialized_(false), dt_(dt) {}

void BasicKalmanFilter::initialize_filter() {
    x_hat_posteriori_.setZero();
    x_hat_priori_.setZero();
    t_initial_ = 0.0;
    t_current_ = t_initial_;
    last_predict_step_time = t_initial_;
    last_update_step_time = t_initial_;
    filter_initialized_ = true;
}

void BasicKalmanFilter::initialize_filter(double t_init, const Eigen::VectorXd &init_states) {
    x_hat_priori_ = init_states;
    P_ = P_init_;
    t_initial_ = t_init;
    t_current_ = t_initial_;
    last_predict_step_time = t_initial_;
    last_update_step_time = t_initial_;
    filter_initialized_ = true;
}

void BasicKalmanFilter::predict_filter_step(){
    x_hat_priori_ = A_*x_hat_posteriori_;
    P_ = A_*P_*A_.transpose() + Q_; 

    t_current_ += dt_;
    last_predict_step_time = t_current_;
}

void BasicKalmanFilter::update_filter_step(const Eigen::VectorXd &y) {
    if (!filter_initialized_) {
        std::cerr << "\n Filter not initialized!";
    }
    K_gain_ = P_*C_.transpose()*(C_*P_*C_.transpose()+R_).inverse();
    x_hat_posteriori_ = x_hat_priori_ + K_gain_*(y - C_*x_hat_priori_);
    P_ = (Eigen::MatrixXd::Identity(num_states_, num_states_) - K_gain_*C_)*P_;

    t_current_ += dt_;
    last_update_step_time = t_current_;
}

const Eigen::VectorXd BasicKalmanFilter::get_latest_estimated_state() const& {
    if (last_update_step_time > last_predict_step_time) {
        return x_hat_posteriori_;
    } else {
        return x_hat_priori_;
    }
}

const double BasicKalmanFilter::get_current_time() const& {
    return t_current_;
}

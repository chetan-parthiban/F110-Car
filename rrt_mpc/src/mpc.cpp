# include "mpc/mpc.h"

// Standard library
#include <cmath>

// Eigen
#include <unsupported/Eigen/MatrixFunctions>

using namespace rrt_mpc;

// Constructor
MPC::MPC(ros::NodeHandle &nh): nh_(nh){
    // Load Car params
    nh_.getParam("wheelbase", wheelbase_);
    nh_.getParam("car_width", car_width_);
    nh_.getParam("max_steering_angle", max_steering_angle_);
    nh_.getParam("max_accel", max_accel_);
    nh_.getParam("max_speed", max_speed_);
    nh_.getParam("friction_coeff", friction_coeff_);
    nh_.getParam("height_cg", h_cg_);
    nh_.getParam("l_cg2rear", l_r_);
    nh_.getParam("l_cg2front", l_f_);
    nh_.getParam("C_S_front", cs_f_);
    nh_.getParam("C_S_rear", cs_r_);
    nh_.getParam("mass", mass_);
    nh_.getParam("moment_inertia", I_z_);
    nh_.getParam("kin_vel_threshold", kin_vel_threshold_);

    // Load MPC params
    nh_.getParam("pred_horizon", N_);
    nh_.getParam("num_dare_iters", num_dare_iters_);
    nh_.getParam("print_osqp_output", print_osqp_output_);
    nh_.getParam("corridor_width_multiplier", corridor_width_multiplier_);
    nh_.getParam("vel_min", vel_min_);
    nh_.getParam("vel_max", vel_max_);
    nh_.getParam("accel_min", accel_min_);
    nh_.getParam("accel_max", accel_max_);

    // Load Q cost matrix params
    nh_.getParam("x_pos_cost", x_pos_cost_);
    nh_.getParam("y_pos_cost", y_pos_cost_);
    nh_.getParam("orientation_cost", orientation_cost_);
    nh_.getParam("velocity_cost", velocity_cost_);
    nh_.getParam("orientation_vel_cost", orientation_vel_cost_);
    nh_.getParam("slip_angle_cost", slip_angle_cost_);

    // Load R cost matrix params
    nh_.getParam("accel_cost", accel_cost_);
    nh_.getParam("steer_cost", steer_cost_);

    // Create Cost Matrices
    Q_.diagonal() << x_pos_cost_, y_pos_cost_, orientation_cost_, velocity_cost_, orientation_vel_cost_, slip_angle_cost_;    
    R_.diagonal() << accel_cost_, steer_cost_;
    hessian_matrix_ = blkdiag(repdiag(Q_, N_+1), repdiag(R_, N_));
    gradient_vector_ = Eigen::VectorXd::Zero(hessian_matrix_.rows());

    // Load Constraints
    xmin_.w() = vel_min_;
    xmax_.w() = vel_max_;    
    umin_ << accel_min_, -max_steering_angle_;
    umax_ << accel_max_, max_steering_angle_;
    
    Eigen::Matrix<double, 1, 1> bf_l = -OsqpEigen::INFTY*Eigen::Matrix<double, 1, 1>::Ones(); // Terminal set {x | bf_l Af*x <= bf_u}
    Eigen::Matrix<double, 1, 1> bf_u = Eigen::Matrix<double, 1, 1>::Ones(); // Terminal set {x | Af*x <= bf}
    
    G_eq_xI_ = repdiag(-Eigen::Matrix<double, nx, nx>::Identity(), N_+1);
    G_eq_ = Eigen::MatrixXd::Zero(nx*(N_+1), nx*(N_+1) + nu*N_);
    
    Eigen::MatrixXd Af = Eigen::Matrix<double, 1, nx>::Zero(); // Terminal set {x | Af*x <= bf}
    Eigen::MatrixXd Au = Eigen::Matrix<double, nu, nu>::Identity();
    G_in_ = blkdiag(repdiag(Ax_, N_), Af, repdiag(Au, N_));
    
    Eigen::VectorXd umin_rep = umin_.replicate(N_, 1);
    lower_bound_ = Eigen::VectorXd::Zero(nx + 2*nx*N_ + bf_u.size() + nu*N_);
    lower_bound_.segment(nx*(N_+1) + nx*N_, bf_l.size()) = bf_l; // Af constraint
    lower_bound_.tail(nu*N_) = umin_rep; // Au constraint
    
    Eigen::VectorXd umax_rep = umax_.replicate(N_, 1);
    upper_bound_ = Eigen::VectorXd::Zero(nx + 2*nx*N_ + bf_u.size() + nu*N_);
    upper_bound_.segment(nx*(N_+1) + nx*N_, bf_u.size()) = bf_u; // Af constraint
    upper_bound_.tail(nu*N_) = umax_rep; // Au constraint
    
    constraint_matrix_ = Eigen::MatrixXd::Zero(G_eq_.rows() + G_in_.rows(), G_in_.cols());
    constraint_matrix_.bottomRows(G_in_.rows()) = G_in_;
    
    // Initialize Solution Variables
    QP_solution_ = Eigen::VectorXd::Zero(hessian_matrix_.rows());
    x_star_ = Eigen::VectorXd::Zero(G_eq_.rows());
    u_star_ = Eigen::VectorXd::Zero(nu*N_);
}

// Main Entrypoint
Eigen::Matrix<double, nu, 1> MPC::carMPC(
    const Eigen::Matrix<double, nx, 1>  &x,
    const Eigen::Matrix<double, nx, 1>  &x_r,
    const double &dt){
    x_ = x;
    x_r_ = x_r;
    dt_ = dt;
    kinematicsOrDynamics(); // Pick if we are using kinematic model or dynamic model

    // Update costs and constraints
    double corroridor_width = car_width_*corridor_width_multiplier_;
    updateCorridorConstraintParams(x, x_r, corroridor_width);
    Eigen::Matrix<double, nx, nx>  P = calcTerminalCost();
    
    updateConstraintMatrix();
    updateHessian(P);
    updateGradientVector(P);
    
    hessian_matrix_sparse_ = hessian_matrix_.sparseView();
    constraint_matrix_sparse_ = constraint_matrix_.sparseView();
    
    OsqpEigen::Solver solver;

    // Set solver settings
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(print_osqp_output_);
    solver.data()->setNumberOfVariables(hessian_matrix_.rows());
    solver.data()->setNumberOfConstraints(constraint_matrix_.rows());
    if(!solver.data()->setHessianMatrix(hessian_matrix_sparse_))

    // Return error and previous u if solving fails
    if(!solver.data()->setHessianMatrix(hessian_matrix_sparse_)){
        ROS_WARN("Error in setting hessian matrix");
        return u_v_des_;
    }
    if(!solver.data()->setGradient(gradient_vector_)){
        ROS_WARN("Error in setting gradient vector");
        return u_v_des_;
    }
    if(!solver.data()->setLinearConstraintsMatrix(constraint_matrix_sparse_)){
        ROS_WARN("Error in setting constraint matrix");
        return u_v_des_;
    }
    if(!solver.data()->setLowerBound(lower_bound_)){
        ROS_WARN("Error in setting lower bound");
        return u_v_des_;
    }
    if(!solver.data()->setUpperBound(upper_bound_)){
        ROS_WARN("Error in setting upper bound");
        return u_v_des_;
    }
    if(!solver.initSolver()){
        ROS_WARN("Error in initializing solver");
        return u_v_des_;
    }
    if(!solver.solve()){
        ROS_WARN("Did not solve using prev_u");
        return u_v_des_;
    }
    
    // Return final solution
    QP_solution_ = solver.getSolution();
    x_star_ = QP_solution_.head(nx*(N_+1));
    u_star_ = QP_solution_.tail(nu*N_);
    
    u_accel_ = u_star_.head<nu>();
    accel2Vdes();
    
    return u_v_des_;
}

// Update Dynamics and Constraints
void MPC::setDynamicsMatrices(
    const Eigen::Matrix<double, nx, 1> &x_lin,
    const Eigen::Matrix<double, nu, 1> &u_lin)
{
    double yaw_lin     = x_lin(2);
    double v_lin       = x_lin(3);
    double yaw_dot_lin = x_lin(4);
    double slip_lin    = x_lin(5);

    double accel_lin       = u_lin.x();
    double steer_angle_lin = u_lin.y();

    Eigen::Matrix<double, nx, 1> cont_dyn, affine_const_cont;
    Eigen::Matrix<double, nx, nx> Ac = Eigen::Matrix<double, nx, nx>::Zero();
    Eigen::Matrix<double, nx, nu> Bc = Eigen::Matrix<double, nx, nu>::Zero();

    if (!is_st_dyn_)
    {
        double s_y  = std::sin(yaw_lin);
        double c_y  = std::cos(yaw_lin);
        double c_st = std::cos(steer_angle_lin);
        double t_st = std::tan(steer_angle_lin);

        Ac(0, 2) = -v_lin*s_y;
        Ac(0, 3) = c_y;
        Ac(1, 2) = v_lin*c_y;
        Ac(1, 3) = s_y;
        Ac(2, 3) = t_st/wheelbase_;
    
        Bc(2, 1) = v_lin/(c_st*c_st*wheelbase_);
        Bc(3, 0) = 1;
        
        cont_dyn << v_lin * c_y,
                         v_lin * s_y,
                         v_lin * t_st,
                         accel_lin,
                         0,
                         0;


    }
    else
    {
        double g = 9.81;
        double rear_val  = g*l_r_ - accel_lin*h_cg_;
        double front_val = g*l_f_ + accel_lin*h_cg_;
        double s_ys  = std::sin(yaw_lin + slip_lin);
        double c_ys  = std::cos(yaw_lin + slip_lin);

        cont_dyn << v_lin * c_ys,
                    v_lin * s_ys,
                    yaw_dot_lin,
                    accel_lin,
                    (friction_coeff_ * mass_ / (I_z_ * wheelbase_)) *
                        (l_f_ * cs_f_ * steer_angle_lin * (rear_val) +
                        slip_lin * (l_r_ * cs_r_ * (front_val) - l_f_ * cs_f_ * (rear_val)) -
                        (yaw_dot_lin/v_lin) * (std::pow(l_f_, 2) * cs_f_ * (rear_val) + std::pow(l_r_, 2) * cs_r_ * (front_val))),
                    (friction_coeff_ / (v_lin * (l_r_ + l_f_))) *
                        (cs_f_ * steer_angle_lin * rear_val - slip_lin * (cs_r_ * front_val + cs_f_ * rear_val) +
                        (yaw_dot_lin/v_lin) * (cs_r_ * l_r_ * front_val - cs_f_ * l_f_ * rear_val)) - yaw_dot_lin;


        double yawd_v = (friction_coeff_ * mass_ / (I_z_ * wheelbase_))
                        * (std::pow(l_f_, 2) * cs_f_ * (rear_val)
                        + std::pow(l_r_, 2) * cs_r_ * (front_val))
                        * yaw_dot_lin / std::pow(v_lin, 2);

        double yawd_yd = -(friction_coeff_ * mass_ / (I_z_ * wheelbase_))
                        * (std::pow(l_f_, 2) * cs_f_ * (rear_val)
                        + std::pow(l_r_, 2) * cs_r_ * (front_val))
                        / v_lin;

        double yawd_sl = (friction_coeff_ * mass_ / (I_z_ * wheelbase_))
                        * (l_r_ * cs_r_ * (front_val)
                        - l_f_ * cs_f_ * (rear_val));

        double yawd_a = (friction_coeff_ * mass_ / (I_z_ * wheelbase_))
                        * (-l_f_*cs_f_*h_cg_*steer_angle_lin
                        + l_r_*cs_r_*h_cg_*slip_lin
                        + l_f_*cs_f_*h_cg_*slip_lin
                        - (yaw_dot_lin/v_lin)
                        *(-std::pow(l_f_,2)*cs_f_*h_cg_)
                        + std::pow(l_r_,2)*cs_r_*h_cg_);

        double yawd_st = (friction_coeff_ * mass_/(I_z_ * wheelbase_)) * (l_f_ * cs_f_ * rear_val);
        
        double slip_v = -(friction_coeff_ / (l_r_ + l_f_))
                        * (cs_f_ * steer_angle_lin * rear_val - slip_lin
                        * (cs_r_ * front_val + cs_f_ * rear_val))/std::pow(v_lin,2)
                        -2*(friction_coeff_ / (l_r_ + l_f_))
                        * (cs_r_ * l_r_ * front_val - cs_f_ * l_f_ * rear_val)
                        * yaw_dot_lin/std::pow(v_lin,3);

        double slip_yd = (friction_coeff_ / (std::pow(v_lin,2) * (l_r_ + l_f_)))
                        * (cs_r_ * l_r_ * front_val - cs_f_ * l_f_ * rear_val) - 1;

        double slip_sl = -(friction_coeff_ / (v_lin * (l_r_ + l_f_)))*(cs_r_ * front_val + cs_f_ * rear_val);

        double slip_a = (friction_coeff_ / (v_lin * (l_r_ + l_f_)))
                        * (-cs_f_*h_cg_*steer_angle_lin - (cs_r_*h_cg_ - cs_f_*h_cg_)*slip_lin
                        + (cs_r_*h_cg_*l_r_ + cs_f_*h_cg_*l_f_)*(yaw_dot_lin/v_lin));

        double slip_st = (friction_coeff_ / (v_lin * (l_r_ + l_f_))) * (cs_f_ * rear_val);
        
        Ac <<   0, 0, -v_lin*s_ys, c_ys,     0,    -v_lin*s_ys,
                0, 0,  v_lin*c_ys, s_ys,     0,     v_lin*c_ys,
                0, 0,       0,      0,       1,         0,
                0, 0,       0,      0,       0,         0,
                0, 0,       0,    yawd_v,  yawd_yd,   yawd_sl,
                0, 0,       0,    slip_v,  slip_yd,   slip_sl;

        Bc <<     0,     0,
                  0,     0,
                  0,     0,
                  1,     0,
                yawd_a, yawd_st,
                slip_a, slip_st;
    }

    // MATLAB c2d with zero-order hold
    Eigen::Matrix<double, 2*nx, 2*nx> aux, M;
    aux.setZero();
    aux.topLeftCorner<nx, nx>() = Ac;
    aux.topRightCorner<nx, nx>() = Eigen::Matrix<double, nx, nx>::Identity();
    M = (aux*dt_).exp();
    Eigen::Matrix<double, nx, nx> M_top_right = M.topRightCorner<nx, nx>();
    affine_const_cont = cont_dyn - (Ac*x_lin + Bc*u_lin);

    A_ = (Ac*dt_).exp();
    B_ = M_top_right*Bc;
    affine_const_ = M_top_right*affine_const_cont;
}

void MPC::updateCorridorConstraintParams(
    const Eigen::Matrix<double, nx, 1>  &x,
    const Eigen::Matrix<double, nx, 1>  &x_r,
    const double &width)
{
    using namespace Eigen;
    VectorXd goal_line = x_r - x;
    Vector2d position;
    position << x.x(), x.y();

    goaline_norm_ << -goal_line.y(), goal_line.x();
    goaline_norm_.normalize();
    double cur_dist = goaline_norm_.dot(position);

    corridor_upper_ = cur_dist + width;
    corridor_lower_ = cur_dist - width;
}

void MPC::updateBounds(){
    lower_bound_.head<nx>() = -x_;
    upper_bound_.head<nx>() = -x_;

    xmin_.x() = corridor_lower_;
    Eigen::VectorXd xmin_rep = xmin_.replicate(N_, 1);
    lower_bound_.segment(nx*(N_+1), nx*N_) = xmin_rep; // Ax constraint

    xmax_.x() = corridor_upper_;
    Eigen::VectorXd xmax_rep = xmax_.replicate(N_, 1);
    upper_bound_.segment(nx*(N_+1), nx*N_) = xmax_rep; // Ax constraint
}

void MPC::updateInequalityConstraints(){
    Ax_.block<1, 2>(0, 0) = goaline_norm_.transpose();
    G_in_.block(0, 0, nx*N_, nx*N_) = repdiag(Ax_, N_);
}

void MPC::updateEqualityConstraints(){
    using namespace Eigen;
    MatrixXd G_eq_xA1 = Eigen::MatrixXd::Zero(nx*N_, nx*N_);
    MatrixXd G_eq_u1  = Eigen::MatrixXd::Zero(nx*N_, nu*N_);
    
    // Linearize around the previous solution
    for (size_t i = 0; i < N_; i++)
    {
        int x_idx = i*nx;
        int u_idx = i*nu;
        
        Matrix<double, nx, 1> x_lin = x_star_.segment<nx>(x_idx);
        Matrix<double, nu, 1> u_lin = u_star_.segment<nu>(u_idx);
        setDynamicsMatrices(x_lin, u_lin);

        G_eq_xA1.block<nx, nx>(x_idx, x_idx) = A_;
        G_eq_u1.block<nx, nu>(x_idx, u_idx) = B_;
        lower_bound_.segment<nx>(nx+x_idx) = -affine_const_;
        upper_bound_.segment<nx>(nx+x_idx) = -affine_const_;
    }
    
    // Add nx_ row of zeros on top for initial condition
    MatrixXd G_eq_xA2 = MatrixXd::Zero(G_eq_xA1.rows()+nx, G_eq_xA1.cols());
    G_eq_xA2.bottomRows(G_eq_xA1.rows()) = G_eq_xA1;
    
    // Extend the columns by N colums of zeros
    MatrixXd G_eq_xA(G_eq_xA2.rows(), G_eq_xA2.cols()+nx);
    G_eq_xA.leftCols(G_eq_xA2.cols()) = G_eq_xA2;
    G_eq_xA.rightCols(N_).setZero();
    MatrixXd G_eq_x = G_eq_xI_ + G_eq_xA;
    
    
    // Add nx_ row of zeros on top for initial condition
    MatrixXd G_eq_u = MatrixXd::Zero(G_eq_u1.rows() + nx, G_eq_u1.cols());
    G_eq_u.bottomRows(G_eq_u1.rows()) = G_eq_u1;

    G_eq_.leftCols(G_eq_x.cols()) = G_eq_x;
    G_eq_.rightCols(G_eq_u.cols()) = G_eq_u;
}

void MPC::updateConstraintMatrix(){
    updateBounds();
    updateInequalityConstraints();
    updateEqualityConstraints();
    constraint_matrix_.topRows(G_eq_.rows()) = G_eq_;
}

void MPC::updateHessian(const Eigen::Matrix<double, nx, nx>  &P){
    hessian_matrix_.block<nx, nx>(N_*nx, N_*nx) = P;
}

void MPC::updateGradientVector(const Eigen::Matrix<double, nx, nx>  &P){
    Eigen::Matrix<double, nx, 1>  Qx_r = -Q_*x_r_;
    Eigen::Matrix<double, nx, 1>  Px_r = -P*x_r_;
    gradient_vector_.head(nx*N_) = Qx_r.replicate(N_, 1);
    gradient_vector_.segment<nx>(nx*N_) = Px_r;
}

// Calculate Terminal Cost
Eigen::Matrix<double, nx, nx> MPC::dare(){
    Eigen::Matrix<double, nx, nx> At, P;
    Eigen::Matrix<double, nu, nx> Bt;
    At = A_.transpose();
    Bt = B_.transpose();
    P = Q_;
    double prev_d;
    double d = INFINITY;
    for (size_t i = 0;
         i < num_dare_iters_ && d >= 0.001 && abs(d-prev_d) >= 1e-6;
         i++)
    {
        prev_d = d;
        Eigen::Matrix<double, nx, nx> Pp = P;
        P = Q_ + At*P*A_ - At*P*B_*(Bt*P*B_+R_).inverse()*Bt*P*A_;
        d = (P - Pp).array().abs().sum();
    }
    
    return P;
}

Eigen::Matrix<double, nx, nx> MPC::calcTerminalCost(){
    // This function is to add alternate ways to calculate the terminal cost aside from dare
    setDynamicsMatrices(x_, u_accel_); // Set dynamics to linearize around current state    
    return dare();
}

void MPC::accel2Vdes(){
    u_v_des_ = u_accel_;
    double alpha = 2*max_accel_/max_speed_;
    double v_des = u_accel_.x()/alpha + x_(3);
    v_des = std::min(std::max(v_des, vel_min_), vel_max_); // clip v_des
    u_v_des_.x() = v_des;
}

void MPC::kinematicsOrDynamics(){
    if ((!is_st_dyn_) && (x_.w() > kin_vel_threshold_)){
        is_st_dyn_ = true;
    }
    if(is_st_dyn_ && (x_.w() < kin_vel_threshold_*0.5)){
        is_st_dyn_ = false;
    }
}

// Get Problem Info
int MPC::getNumberStates() {return nx;}
int MPC::getNumberInputs() {return nu;}

// Get MPC Solutions
Eigen::Matrix<double, nu, 1> MPC::getU() {return u_v_des_;}
Eigen::VectorXd MPC::getUStar() {return u_star_;}
Eigen::VectorXd MPC::getXStar() {return x_star_;}
Eigen::VectorXd MPC::getQPSolution() {return QP_solution_;}
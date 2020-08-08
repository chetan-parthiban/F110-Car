// ROS libraries
#include <ros/ros.h>

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/SparseCore>

// Osqp Eigen
#include <OsqpEigen/OsqpEigen.h>

// Good progammig practice to have globals in a namespace
// namespace named after package
namespace rrt_mpc
{
    constexpr int nx = 6; // number of states
    constexpr int nu = 2; // number of inputs
}

using namespace rrt_mpc;

class MPC
{
private:
    ros::NodeHandle nh_;

    // Car params
    double wheelbase_;
    double car_width_;
    double max_steering_angle_;
    double max_accel_;
    double max_speed_;
    double friction_coeff_;
    double h_cg_;
    double l_r_;
    double l_f_;
    double cs_f_;
    double cs_r_; 
    double mass_;
    double I_z_;
    double kin_vel_threshold_; // velocity threshold used to switch from dynamic model or kinematic model
    bool   is_st_dyn_ = false;  // Use dynamics when true, use kinematics when false

    // MPC params
    int N_; // prediction horizon
    double dt_;
    int num_dare_iters_;
    bool print_osqp_output_;
    double vel_min_;
    double vel_max_;
    double accel_min_;
    double accel_max_;
    double corridor_width_multiplier_;
    double corridor_upper_;
    double corridor_lower_;
    Eigen::Vector2d goaline_norm_;

    // Cost params
    double x_pos_cost_;
    double y_pos_cost_;
    double orientation_cost_;
    double velocity_cost_;
    double orientation_vel_cost_;
    double slip_angle_cost_;
    double accel_cost_;
    double steer_cost_;

    // State Vector and Reference Vector
    Eigen::Matrix<double, nx, 1>  x_; // current state
    Eigen::Matrix<double, nx, 1>  x_r_; // current reference

    // Transition Matrices
    Eigen::Matrix<double, nx, nx> A_ = Eigen::Matrix<double, nx, nx>::Zero();
    Eigen::Matrix<double, nx, nu> B_ = Eigen::Matrix<double, nx, nu>::Zero();
    Eigen::Matrix<double, nx, 1> affine_const_ = Eigen::Matrix<double, nx, 1>::Zero();

    // Cost Matrices
    Eigen::Matrix<double, nx, nx> Q_ = Eigen::Matrix<double, nx, nx>::Zero();
    Eigen::Matrix<double, nu, nu> R_ = Eigen::Matrix<double, nu, nu>::Zero();
    
    // Constraint Matrices
    Eigen::Matrix<double, nx, nx> Ax_ = Eigen::Matrix<double, nx, nx>::Identity();
    Eigen::Matrix<double, nx, 1> xmin_ = -OsqpEigen::INFTY*Eigen::Matrix<double, nx, 1>::Ones();
    Eigen::Matrix<double, nx, 1> xmax_ = OsqpEigen::INFTY*Eigen::Matrix<double, nx, 1>::Ones();
    Eigen::Matrix<double, nu, 1> umin_ = Eigen::Matrix<double, nu, 1>::Zero();
    Eigen::Matrix<double, nu, 1> umax_ = Eigen::Matrix<double, nu, 1>::Zero();

    // Full QP Matrices
    Eigen::SparseMatrix<double> hessian_matrix_sparse_, constraint_matrix_sparse_;
    Eigen::MatrixXd hessian_matrix_, G_in_, G_eq_xI_, G_eq_, constraint_matrix_;
    Eigen::VectorXd gradient_vector_, lower_bound_, upper_bound_;

    // MPC Solutions
    Eigen::VectorXd QP_solution_;
    Eigen::VectorXd x_star_;
    Eigen::VectorXd u_star_;
    Eigen::Matrix<double, nu, 1> u_accel_ = Eigen::Matrix<double, nu, 1>::Zero(); // [accel; steer_angle]
    Eigen::Matrix<double, nu, 1> u_v_des_ = Eigen::Matrix<double, nu, 1>::Zero();

    // Update Dynamics and constraints
    void updateCorridorConstraintParams(
        const Eigen::Matrix<double, nx, 1>  &x,
        const Eigen::Matrix<double, nx, 1>  &x_r,
        const double &width);
    void setDynamicsMatrices(
        const Eigen::Matrix<double, nx, 1> &x_lin,
        const Eigen::Matrix<double, nu, 1> &u_lin);
    void updateBounds();
    void updateInequalityConstraints();
    void updateEqualityConstraints();
    void updateConstraintMatrix();
    void updateHessian(const Eigen::Matrix<double, nx, nx>  &P);
    void updateGradientVector(const Eigen::Matrix<double, nx, nx>  &P);

    // Calculate terminal cost
    Eigen::Matrix<double, nx, nx> dare();
    Eigen::Matrix<double, nx, nx> calcTerminalCost();

    void accel2Vdes();
    void kinematicsOrDynamics();

public:
    // Constructor
    MPC() = default;
    MPC(ros::NodeHandle &nh);
    // Outside Entry Point
    Eigen::Matrix<double, nu, 1> carMPC(
        const Eigen::Matrix<double, nx, 1>  &x,
        const Eigen::Matrix<double, nx, 1>  &x_r,
        const double &dt);
    // Get Problem Info
    int getNumberStates();
    int getNumberInputs();
    // Get Solution
    Eigen::Matrix<double, nu, 1> getU();
    Eigen::VectorXd getUStar();
    Eigen::VectorXd getXStar();
    Eigen::VectorXd getQPSolution();
};

// Block Diagonal Matrix Utils
// Repeats matrix A in a diagonal N number of times
template <typename Derived>
Eigen::MatrixXd repdiag(const Eigen::MatrixBase<Derived>& A, const int &n){
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(A.rows() * n, A.cols() * n);
    for (int i = 0; i < n; ++i)
    {
        bdm.block(i * A.rows(), i * A.cols(), A.rows(), A.cols()) = A;
    }

    return bdm;
}

// Block Matrix from A and B
template <typename Derived>
Eigen::MatrixXd blkdiag(const Eigen::MatrixBase<Derived>& A, const Eigen::MatrixBase<Derived>& B){
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(A.rows() + B.rows(), A.cols() + B.cols());
    bdm.topLeftCorner(A.rows(), A.cols()) = A;
    bdm.bottomRightCorner(B.rows(), B.cols()) = B;
    return bdm;
}

// Block Matrix from A, B, and C
template <typename Derived>
Eigen::MatrixXd blkdiag(
    const Eigen::MatrixBase<Derived>& A,
    const Eigen::MatrixBase<Derived>& B,
    const Eigen::MatrixBase<Derived>& C){
    Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(
        A.rows() + B.rows() + C.rows(),
        A.cols() + B.cols() + C.cols());

    bdm.topLeftCorner(A.rows(), A.cols()) = A;
    bdm.block(A.rows(), A.cols(), B.rows(), B.cols()) = B;
    bdm.bottomRightCorner(C.rows(), C.cols()) = C;
    return bdm;
}
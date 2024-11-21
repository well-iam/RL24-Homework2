#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    /*
    std::cout << "INSIDE CONTROLLER\n";
    std::cout << ddqd << "\n";
    std::cout << robot_->getJsim() << "\n\n";
    */
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    //implementation of Op.Space Controller 4a)
 
// calculate gain matrices (matrici diagonali)
    Eigen::Matrix<double,6,6> Kp, Kd;
    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();  
    Kp.block(2,2,1,1) = 5*Kp.block(2,2,1,1);
    Kd.block(2,2,1,1) = 2*Kd.block(2,2,1,1);
 
    // read current cartesian state 
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    Eigen::Matrix<double,7,7> M = robot_->getJsim();
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
 
 
    // position (desired + current)
    Eigen::Vector3d p_d(_desPos.p.data);
    Eigen::Vector3d p_e(robot_->getEEFrame().p.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    R_d = matrixOrthonormalization(R_d);
    R_e = matrixOrthonormalization(R_e);
 
    // velocity (desired + current)
    Eigen::Vector3d dot_p_d(_desVel.vel.data);                                 
    Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);
    Eigen::Vector3d omega_d(_desVel.rot.data);
    Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);
 
    // acceleration (desired)
    Eigen::Matrix<double,6,1> dot_dot_x_d;
    Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
    Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);
 
    // compute linear errors (position + velocity)
    Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
    Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);
 
    // compute orientation errors (position + velocity)                                        
    Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                        omega_e,
                                                                        R_d,
                                                                        R_e);
    // calculate errors                                                                   
    Eigen::Matrix<double,6,1> x_tilde;                                                 
    Eigen::Matrix<double,6,1> dot_x_tilde;
    x_tilde << e_p, e_o;
    dot_x_tilde << dot_e_p, dot_e_o;
    dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;
 
    //Define J_dot*q_dot
    Eigen::Matrix<double,6,1> Jdotqdot = robot_->getEEJacDotqDot();
 
    //Control Law
    Eigen::Matrix<double,7,1> y;
    Eigen::Matrix<double,7,1> n;
 
    y << Jpinv * (dot_dot_x_d - Jdotqdot + Kd*dot_x_tilde + Kp*x_tilde);
    n << robot_->getCoriolis() + robot_->getGravity();
 
    
    return M * y + n;
}


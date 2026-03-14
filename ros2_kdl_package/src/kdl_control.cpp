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
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}

Eigen::VectorXd KDLController::velocity_ctrl_null(KDL::JntArray &q_min,
                                                  KDL::JntArray &q_max,
                                                  double lambda,
                                                  KDL::JntArray &_qd,
                                                  double Kp,
                                                  Eigen::VectorXd &pos_error)
{
    unsigned int nj = robot_->getNrJnts();
    Eigen::VectorXd joint_velocity_cmd(nj);
    Eigen::VectorXd q0_dot(nj);

    //computation of q0_dot

    for(int i=0; i<nj; i++){
        double span = q_max(i) - q_min(i);
        q0_dot(i) = (1/lambda)*((span*span)/((q_max(i)-_qd(i))*(_qd(i)-q_min(i))));
        //printf("%lf\t",q0_dot(i));
    }
    //printf("\n\n");

    //computation of Jacobian Pseudoinverse
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    
    //DEBUG
    /*
    printf("J\n");
    for (int i = 0; i < J.rows(); ++i) {
    for (int j = 0; j < J.cols(); ++j)
        printf("%f ", J(i, j));
    printf("\n");
    }
    printf("J_pinv\n");
    for (int i = 0; i < J_pinv.rows(); ++i) {
    for (int j = 0; j < J_pinv.cols(); ++j)
        printf("%f ", J_pinv(i, j));
    printf("\n");
    }
    */

    
    //CONTROL LAW
    Eigen::VectorXd Kpe = (Kp*pos_error);
    printf("task calculated 1\n");
    //printf("Kpe: Rows: %d, Cols: %d\n", Kpe.rows(), Kpe.cols());

    Eigen::VectorXd task = J_pinv * Kpe;
    //printf("task calculated 2\n");
    
    //printf("Rows: %d, Cols: %d\n", task.rows(), task.cols());

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(nj, nj);
    //printf("identyty matrix defined\n");
    Eigen::MatrixXd N = I - J_pinv * J;
    //printf("N matrix calculated\n");
    Eigen::VectorXd null = N * q0_dot;
    //printf("null task calculated\n");
    
    joint_velocity_cmd = task + null;
    //printf("joint_velocity_cmd: Rows: %d, Cols: %d\n", joint_velocity_cmd.rows(), joint_velocity_cmd.cols());

    //printf("command calculated\n");
    
    //joint_velocity_cmd.setZero();
    return joint_velocity_cmd;
}

Eigen::VectorXd KDLController::vision_ctrl(KDL::JntArray &q_min,
                                                  KDL::JntArray &q_max,
                                                  double lambda,
                                                  KDL::JntArray &_qd,
                                                  double Kp,
                                                  Eigen::Vector3d &aruco_pos)
{
    unsigned int nj = robot_->getNrJnts();
    Eigen::VectorXd joint_velocity_cmd = Eigen::VectorXd::Zero(nj);
    Eigen::Vector3d sd(0.0, 0.0, 1.0); //desired feature velocity
    Eigen::Vector3d cPo;
    cPo = aruco_pos;


    Eigen::Matrix<double,3,3> Rc;
    Rc = toEigen(robot_->getEEFrame().M);//assumiamo che la matrice di rotazione siano approssimabili
    Eigen::MatrixXd K(nj,nj);
    K = 3*Kp*K.Identity(nj,nj);

    Eigen::Matrix<double,6,6> R =Eigen::Matrix<double,6,6>::Zero();

    R.block<3, 3>(0, 0) = Rc;//.transpose();
    R.block<3, 3>(3, 3) = Rc;//.transpose();

    Eigen::Vector3d s;
    for (int i=0; i<3; i++){
        s(i) = cPo(i)/cPo.norm();
    }
    
    
    /*RCLCPP_INFO(rclcpp::get_logger("KDLController"),
                "vector s: %f %f %f",
                s(0),s(1),s(2));
    */
    Eigen::Matrix<double,3,3> L1;
    L1 = -1/cPo.norm()* (Eigen::Matrix3d::Identity() - s*s.transpose());

    Eigen::Matrix3d S_skew = Eigen::Matrix3d::Zero();
    S_skew <<     0, -s.z(),  s.y(),
                 s.z(),      0, -s.x(),
                -s.y(),  s.x(),      0;


    Eigen::Matrix<double,3,3> L2;
    L2 = S_skew;

    Eigen::Matrix<double,3,6> L;

    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = L2;
    
    L = L*R;

    Eigen::MatrixXd J;
    J = robot_->getEEJacobian().data;
    Eigen::MatrixXd Jc; 
    Jc  = J; //assumiamo che i due jacobiani siano uguali

    
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(nj,nj);

    Eigen::MatrixXd JntLimits_ (nj,2);
    JntLimits_ = robot_->getJntLimits();

    Eigen::VectorXd q(nj);
    q  = robot_->getJntValues();

    Eigen::VectorXd q0_dot(nj);

    for(int i=0; i<nj; i++){
        double span = q_max(i) - q_min(i);
        q0_dot(i) = (1/lambda)*((span*span)/((q_max(i)-q(i))*(q(i)-q_min(i))));
        //printf("%lf\t",q0_dot(i));
    }

    Eigen::MatrixXd N (nj,nj);

    N = I - pseudoinverse(J)*J;
    
    Eigen::MatrixXd J_pinv = pseudoinverse(L*J);
 
    Eigen::VectorXd null = N * q0_dot;

    Eigen::VectorXd task  = K*J_pinv*sd;

    joint_velocity_cmd = task + null;


    return joint_velocity_cmd;
}


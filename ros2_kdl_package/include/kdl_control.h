#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _KppS,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    Eigen::VectorXd velocity_ctrl_null(KDL::JntArray &q_min,
                            KDL::JntArray &q_max,
                            double lambda,
                            KDL::JntArray &_qd,
                            double Kp,
                            Eigen::VectorXd &pos_error);
    
    Eigen::VectorXd vision_ctrl(KDL::JntArray &q_min,
                                KDL::JntArray &q_max,
                                double lambda,
                                KDL::JntArray &_qd,
                                double Kp,
                                Eigen::Vector3d &aruco_pos);

                           

private:

    KDLRobot* robot_;

};

#endif

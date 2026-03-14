// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/kdl_feedback.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        using KdlFeedback = ros2_kdl_package::action::KdlFeedback;
        using GoalHandleKdlFeedback = rclcpp_action::ServerGoalHandle<KdlFeedback>;

        Iiwa_pub_sub()
        : Node("ros2_kdl_node"), 
        node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // declare cmd_interface parameter (position, velocity)
            declare_parameter("cmd_interface", "position"); // default to "position"
            get_parameter("cmd_interface", cmd_interface_);

            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());

            this->declare_parameter("traj_duration", 1.5); // default to 1.5s
            this->declare_parameter("acc_duration", 0.5); // default to 0.5s
            this->declare_parameter("total_time", 0.5); // default to 0.5s
            this->declare_parameter("trajectory_len", 150); // default to 150
            this->declare_parameter("Kp", 5.0); // default to 5
            this->declare_parameter("lambda", 5.0); // default to 5
            this->declare_parameter("ctrl", "velocity_ctrl_null");
            this->declare_parameter("action_server_mode", false);
            
            this->declare_parameter("end_position", std::vector<double>{0.5,0.0,0.2});
            std::vector<double> end_position_array;
            this->get_parameter("end_position", end_position_array);
            
            this->get_parameter("ctrl", velocity_ctrl_type);
            RCLCPP_INFO(get_logger(),"Current velocity control is: '%s'", velocity_ctrl_type.c_str());

            bool action_server_mode;

            this->get_parameter("action_server_mode",action_server_mode);
            RCLCPP_INFO(get_logger(),"ACTION SERVER MODE IS: '%d'", action_server_mode);

            /*
            RCLCPP_INFO(get_logger(),"end position 1 is: '%lf'", end_position_array[0]);
            RCLCPP_INFO(get_logger(),"end position 2 is: '%lf'", end_position_array[1]);
            RCLCPP_INFO(get_logger(),"end position 3 is: '%lf'", end_position_array[2]);
            */

            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort"))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            // declare traj_type parameter (linear, circular)
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
            if (!(traj_type_ == "linear" || traj_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected traj type is not valid!"); return;
            }

            // declare s_type parameter (trapezoidal, cubic)
            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
            if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
            }

            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 

            RCLCPP_INFO(get_logger(),"DEBUG 1");

            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "iiwa/robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }

            RCLCPP_INFO(get_logger(),"DEBUG 2");

            auto parameter = parameters_client->get_parameters({"robot_description"});

            RCLCPP_INFO(get_logger(),"DEBUG 3");

            // create KDLrobot structure
            KDL::Tree robot_tree;
            
            RCLCPP_INFO(get_logger(),"DEBUG 4");

            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            RCLCPP_INFO(get_logger(),"DEBUG 5");

            // Create joint array
            
            unsigned int nj = robot_->getNrJnts();
            RCLCPP_INFO(get_logger(),"number of joints is '%d'", nj);
            q_min.resize(nj);
            q_max.resize(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            RCLCPP_INFO(get_logger(),"DEBUG 4");

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/iiwa/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));
            // ---------- Inspection interfaces ----------

            // Publish inspection active flag
            inspection_active_pub_ =
                this->create_publisher<std_msgs::msg::Bool>(
                    "/inspection_active", 10);

            // Subscribe to inspection result from CV node
            inspection_result_sub_ =
                this->create_subscription<std_msgs::msg::String>(
                    "/inspection_result", 10,
                    [this](const std_msgs::msg::String::SharedPtr msg)
                    {
                        inspection_result_ = msg->data;
                        inspection_received_ = true;

                        RCLCPP_INFO(this->get_logger(),
                            "Inspection result received: %s",
                            inspection_result_.c_str());
                    }
                );

            
            if(velocity_ctrl_type == "vision_ctrl"){
                // Subscriber to aruco marker position
                arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));
            }

            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }

            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            // Compute EE frame
            init_cart_pose_ = robot_->getEEFrame();
            // std::cout << "The initial EE pose is: " << std::endl;  
            // std::cout << init_cart_pose_ <<std::endl;

            // Compute IK
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            // std::cout << "The inverse kinematics returned: " <<std::endl; 
            // std::cout << q.data <<std::endl;

            // Initialize controller
            //KDLControl controller_(*robot_);

            // EE's trajectory initial position (just an offset)
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

            // EE's trajectory end position (just opposite y)
            Eigen::Vector3d end_position; end_position << end_position_array[0], -end_position_array[1], end_position_array[2];


            // Plan trajectory
            double traj_duration = 2;
            double acc_duration = 1,
            traj_radius = 0.15;

            this->get_parameter("traj_duration",traj_duration);  
            RCLCPP_INFO(get_logger(),"Current trajectory duration is: '%lf'", traj_duration);

            this->get_parameter("acc_duration",acc_duration);  
            RCLCPP_INFO(get_logger(),"Current acceleration duration is: '%lf'", acc_duration);

            // Retrieve the first trajectory point
            if(traj_type_ == "linear"){
                planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } 
            else if(traj_type_ == "circular")
            {
                planner_ = KDLPlanner(traj_duration, init_position, traj_radius, acc_duration);
                if(s_type_ == "trapezoidal")
                {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                }else if(s_type_ == "cubic")
                {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }
            // // Retrieve the first trajectory point
            // trajectory_point p = planner_.compute_trajectory(t);

            // compute errors
            // Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            //std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/iiwa_arm_controller/commands", 10);
                
                if(action_server_mode==false){

                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                }else{
                    this->action_server_ = rclcpp_action::create_server<KdlFeedback>(this,"kdl_feedback",
                    std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                    std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
                    );
                }
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa/velocity_controller/commands", 10);    

                if(action_server_mode==false){
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                }else{
                    this->action_server_ = rclcpp_action::create_server<KdlFeedback>(this,"kdl_feedback",
                    std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                    std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
                    );
                }
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);

                if(action_server_mode==false){
                   
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                                std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                }else{
                    this->action_server_ = rclcpp_action::create_server<KdlFeedback>(this,"kdl_feedback",
                    std::bind(&Iiwa_pub_sub::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&Iiwa_pub_sub::handle_cancel, this, std::placeholders::_1),
                    std::bind(&Iiwa_pub_sub::handle_accepted, this, std::placeholders::_1)
                    );
                }
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            if (action_server_mode == false){
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Waiting for action client to send request ...");
            }
        }

    private:
        rclcpp_action::Server<KdlFeedback>::SharedPtr action_server_;
        // ---------- Inspection synchronization ----------
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr inspection_active_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inspection_result_sub_;

        bool inspection_received_ = false;
        std::string inspection_result_;


        //Action Server Callbacks
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                                std::shared_ptr<const KdlFeedback::Goal> goal)
        {
            if (!goal->start) return rclcpp_action::GoalResponse::REJECT;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleKdlFeedback>)
        {
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleKdlFeedback> goal_handle)
        {
            std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
        }

        //Action server routine
/*
        void execute(const std::shared_ptr<GoalHandleKdlFeedback> goal_handle)
        {
        (void) goal_handle;  // suppress unused warning
        RCLCPP_INFO(this->get_logger(), "Hello world");
        }
*/

        void execute(const std::shared_ptr<GoalHandleKdlFeedback> goal_handle)
        {
        auto error_feedback = std::make_shared<KdlFeedback::Feedback>();
        auto result = std::make_shared<KdlFeedback::Result>();
        rclcpp::Rate rate(10);
        // ---------- INSPECTION START ----------
        std_msgs::msg::Bool active_msg;
        active_msg.data = true;
        inspection_active_pub_->publish(active_msg);

        // Reset inspection result state
        inspection_received_ = false;
        inspection_result_.clear();


        while(trajectory_executed == false)
        {
            if (goal_handle->is_canceling())
            {
            result->completed = false;
            goal_handle->canceled(result);
            return;
            }

            cmd_publisher();

            for(int i = 0; i<6;i++){
                error_feedback -> current_error[i] = total_error(i);
                RCLCPP_INFO(this->get_logger(),"%f",error_feedback -> current_error[i]);

            }  

            goal_handle->publish_feedback(error_feedback);
            rate.sleep();
        }
        // ---------- INSPECTION END ----------
        active_msg.data = false;
        inspection_active_pub_->publish(active_msg);

        RCLCPP_INFO(this->get_logger(), "Waiting for inspection result...");

        while (!inspection_received_ && rclcpp::ok())
        {
            //rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // ---------- ACTION RESULT ----------
        if (inspection_result_ == "ADMIT")
        {
            result->completed = true;
        }
        else
        {
            result->completed = false;
        }

        goal_handle->succeed(result);

        }

        //standard routine
        void cmd_publisher(){

            // Initialize controller
            KDLController controller_(*robot_);    //fix later

            iteration_ = iteration_ + 1;

            // define trajectory
            double total_time; // 
            int trajectory_len; // 
            double Kp = 5.0;
            double lambda;

            this->get_parameter("total_time",total_time);
            //RCLCPP_INFO(get_logger(),"Current total_time duration is: '%lf'", total_time); 
            
            this->get_parameter("trajectory_len",trajectory_len);
            //RCLCPP_INFO(get_logger(),"Current trajectory_len is: '%d'", trajectory_len); 

            this->get_parameter("Kp",Kp);
            this->get_parameter("lambda",lambda);

            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

            total_error.resize(6);      //resizing operational space error vector to be 6-dimentional

            if (t_ < total_time){

                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);

                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;

                // Retrieve the trajectory point based on the trajectory type
                if(traj_type_ == "linear"){
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.linear_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.linear_traj_cubic(t_);
                    }
                } 
                else if(traj_type_ == "circular")
                {
                    if(s_type_ == "trapezoidal")
                    {
                        p_ = planner_.circular_traj_trapezoidal(t_);
                    }else if(s_type_ == "cubic")
                    {
                        p_ = planner_.circular_traj_cubic(t_);
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = init_cart_pose_.M; desFrame.p = toKDL(p_.pos); 

                // compute errors
                Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;
                
                total_error << error, o_error;

                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = init_cart_pose_.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp*error))*dt; 

                    // Compute IK
                    joint_positions_cmd_ = joint_positions_;
                    robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
                }
                else if(cmd_interface_ == "velocity"){
                
                    //RCLCPP_INFO(get_logger(),"ENTERED VELOCITY CONTROLLER");    
                    
                    if(velocity_ctrl_type == "velocity_ctrl"){        //velocity controller previously implemented
                        // Compute differential IK
                        Vector6d cartvel; cartvel << p_.vel + Kp*error, o_error;
                        joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    }
                    else if(velocity_ctrl_type == "velocity_ctrl_null"){      //null space velocity controller
                        //RCLCPP_INFO(get_logger(),"ENTERED NULL SPACE CONTROLLER");         
                        joint_velocities_cmd_.data = controller_.velocity_ctrl_null(q_min, q_max, lambda, joint_positions_, Kp, total_error);
                    }
                    else if(velocity_ctrl_type == "vision_ctrl"){      //vision based controller
                        //RCLCPP_INFO(get_logger(),"ENTERED VISION BASED CONTROLLER"); 
                        joint_velocities_cmd_.data = controller_.vision_ctrl(q_min, q_max, lambda, joint_positions_, Kp, marker_pose);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time);
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);

                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
                // std::cout << "EE pose is: " << robot_->getEEFrame() <<std::endl;  
                // std::cout << "Jacobian: " << robot_->getEEJacobian().data <<std::endl;
                // std::cout << "joint_positions_: " << joint_positions_.data <<std::endl;
                // std::cout << "joint_velocities_: " << joint_velocities_.data <<std::endl;
                // std::cout << "iteration_: " << iteration_ <<std::endl <<std::endl;
                // std::cout << "/////////////////////////////////////////////////" <<std::endl <<std::endl;
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // Send joint velocity commands
                if(cmd_interface_ == "position"){
                    // Set joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Set joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    // Set joint effort commands
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
                            
                //set trajectory executed to true in order to terminate the action server routine loop
                trajectory_executed = true;
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }

            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& pose_msg_in){

            // Convert PoseStamped position to Eigen::Vector3d

            // or alternatively:
            marker_pose << pose_msg_in.pose.position.x,
                           pose_msg_in.pose.position.y,
                           pose_msg_in.pose.position.z;

            // RCLCPP_INFO(this->get_logger(), "Aruco Marker Position x: %f", pose_msg.pose.position.x);                
            // RCLCPP_INFO(this->get_logger(), "Aruco Marker Position y: %f", pose_msg.pose.position.y);                
            // RCLCPP_INFO(this->get_logger(), "Aruco Marker Position z: %f", pose_msg.pose.position.z);                
        }

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        KDL::JntArray q_min;        //made these variables shared
        KDL::JntArray q_max;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string velocity_ctrl_type;     //added this variable to store the parameter relative to which velocity controller is chosen

        KDL::Frame init_cart_pose_;
        Eigen::VectorXd total_error;
        Eigen::Vector3d marker_pose;

        bool trajectory_executed = false;
        
};

 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
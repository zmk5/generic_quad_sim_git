//
// Created by aniketrs on 2/23/18.
//

#ifndef GENERIC_QUAD_SIM_QUADROTOR_DRIVE_H
#define GENERIC_QUAD_SIM_QUADROTOR_DRIVE_H
//
// Created by aniketrs on 2/2/18.
//
#include <map>

// Gazebo
#include <gazebo/common/common.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
//ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

//Custom callback queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

//Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define GRAV 9.81
//#define K_MOTOR 0.0000191602
#define  K_MOTOR 2.980e-6
#define B_MOTOR 1.140e-7

namespace gazebo{
    class Joint;
    class Entity;

    class GazeboRosRotorPropulsion: public ModelPlugin{
    public:
        GazeboRosRotorPropulsion();
        ~GazeboRosRotorPropulsion();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Reset();

    protected:
        virtual void UpdateChild();
        virtual void FiniChild();

    private:
        /*
        double omega_[4];
        double Torque[3];
        double rot_x_;
        double rot_y_;
        double rot_z_;
        double p_;
        double q_;
        double r_;
        double x_dot_; //Brief: Body forward velocity
        double y_dot_; //Brief: Body sideways velocity
        double z_dot_; //Brief: Body upward velocity
        */

        bool alive_;
        //SECTION: Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

        geometry_msgs::Pose pose_encoder_;
//        geometry_msgs::Wrench wrench;
        common::Time last_odom_update_;
        /*
//        Eigen::VectorXd vel_vec;
//        Eigen::Vector3d vel_vec_lin;
//        Eigen::Vector3d vel_vec_ang;
         */
        //SECTION:Flags
        bool publishRotorTF_;
        bool publishRotorJointState_;

        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent_;
        event::ConnectionPtr update_connection_;
        //BRIEF: All these parameters have to be read from the urdf file using the getParameter() method
        double rotor_torque_;
        /*
//        double rotor_speed_[4];
//        double current_rotor_speed[4];
//        double eqm_rotor_speed[4];
         */
        double rotor_accel_;
        /*
//        double rotor_speed_instr_[4];
//        double force_joint_[4];
//        double force_joint_inst_[4];
//        double v_curr_lin_[3];
//        double v_curr_ang_[3];
         */
        double base_mass_;
        double arm_mass_;
        double motor_mass_;
        double rotor_mass_;
        double camera_mass_;
        double total_mass_;
        double arm_len_;
        double motor_radius_;
        double base_len_;
        double fulcrum;
        math::Vector3 link_force[4];
        math::Vector3 rotor_speed_instr_[4];
        math::Vector3 base_yaw_torque;
        math::Vector3 rotor_speed_[4];
        math::Vector3 force_joint_[4];
        math::Vector3 force_joint_inst_[4];
        math::Vector3 current_rotor_speed[4];

        double commanded_torque;
        double net_torque_yaw_base;
        /*
        double base_inertia_[3];

        double base_wid_;
        double base_hei_;
        double arm_inertia_[3];

        double arm_wid_;
        double arm_hei_;
        double motor_inertia_[3];

        double motor_hei_;
        double rotor_inertia_[3];
         */
        double rotor_radius_;
        /*
//        double rotor_height_;
//        struct{
//            double orientation[4];
//            double ang_vel[3];
//            double acc_lin[3];
//        }imu_data_;
        */
        struct{
            double force[3];
            double torque[3];
        }wrench_data_[4];
        double eqm_force_d[4];
        math::Vector3 eqm_force[4];
        math::Vector3 eqm_rotor_speed[4];
//        double curr_force[4];
        double total_force[4];
        double commanded_force[4];
        double net_force[4];
//        double force[4];
        double body_force[4];
        double body_ang_vel[4];
//        double Thrust;
//        double Roll_torque;
//        double Pitch_torque;
//        double Yaw_torque;
//        double force_vec[4];
        double omega[4];
        double curr_base_yaw_torque;

        std::string body_force_cal_flag;
//        Eigen::Vector4d ang_vel_sq;
//        Eigen::Matrix4d vel_to_rotor_force;
//        Eigen::Vector4d rotor_force;
        Eigen::Vector4d force_vec;
        Eigen::Matrix4d force_vec_to_curr_commanded_force;
//        Eigen::Vector4d curr_joint_force;
//        Eigen::Vector4d curr_joint_rpm;
//        Eigen::Vector4d curr_joint_rpm_to_force;
        Eigen::Vector4d curr_commanded_force;
        Eigen::Vector4d commanded_forces_eigen;
        Eigen::Vector4d commanded_rotor_rpm;
        Eigen::Matrix4d commanded_to_rotor_rpm;
        Eigen::Vector4d commanded_rotor_forces_eigen;
        Eigen::Matrix4d commanded_to_rotor_forces;



        std::vector<physics::JointPtr> joints_;
        physics::JointPtr base_joint_;
//        physics::JointWrench wrench_body[4];
        geometry_msgs::Twist vel_vec_twist;
        geometry_msgs::Quaternion total_force_quat;
        sensor_msgs::Imu imu_data_;

        //SECTION: ROS Stuff
        ros::NodeHandle nh_;
        ros::Publisher odometry_publisher_;
        ros::Subscriber cmd_vel_subscriber_;
        ros::Subscriber imu_data_subscriber_;
//        ros::Subscriber wrench_data_subscriber_front_;
//        ros::Subscriber wrench_data_subscriber_rear_;
//        ros::Subscriber wrench_data_subscriber_left_;
//        ros::Subscriber wrench_data_subscriber_right_;
        ros::Subscriber cmd_force_subscriber_;

        ros::Publisher joint_state_publisher_;
//        ros::Publisher wrench_data_publisher_front_;
//        ros::Publisher wrench_data_publisher_rear_;
//        ros::Publisher wrench_data_publisher_left_;
//        ros::Publisher wrench_data_publisher_right_;
        ros::Publisher cmd_vel_publisher_;
        ros::Publisher imu_publisher_;
        ros::Publisher cmd_force_publihser_;

        boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
        sensor_msgs::JointState joint_state_;
        nav_msgs::Odometry odom_;
        std::string tf_prefix_;

        std::string robot_namespace_;
        std::string command_topic_;
        std::string joint_states_;
//        std::string wrench_topic_front_;
//        std::string wrench_topic_rear_;
//        std::string wrench_topic_left_;
//        std::string wrench_topic_right_;
        std::string imu_topic_;
        std::string command_force_topic_;
        //std::string odometry_frame_;
        //std::string odometry_topic_;
        std::string robot_base_frame_;
        bool publish_tf_;
        bool publish_wrench_;

        //SECTION: Custom callback queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;
        boost::mutex lock;

        //SECTION: METHODS for the class
        //void publishOdometry();
        void QueueThread();
//        void getRotorVelocities();
//        void getRotorForces();
//        double *getLinearBodyVel(double []);
        void publishRotorTF();
        void publishRotorJointState();
//        void publishRotorWrench();
//        void publishRotorWrenchFront();
//        void publishRotorWrenchRear();
//        void publishRotorWrenchLeft();
//        void publishRotorWrenchRight();
        void publishOdometry(double);
        void publishCmdvel();
        void pubishImu();
        void publishCmdForce();
//        Eigen::Vector4d force_to_ang_vel();

        //SECTION:Rotor Propulsion
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
//        void frontwrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench_msg_front);
//        void rearwrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench_msg_rear);
//        void leftwrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench_msg_left);
//        void rightwrenchCallback(const geometry_msgs::Wrench::ConstPtr& wrench_msg_right);
        void commandforceCallback(const geometry_msgs::Quaternion::ConstPtr& cmd_force);
    };

}

#endif //GENERIC_QUAD_SIM_QUADROTOR_DRIVE_H

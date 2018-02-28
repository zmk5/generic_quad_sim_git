//
// Created by aniketrs on 2/23/18.
//


//
// Created by aniketrs on 2/2/18.
//
/*
     Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
     All rights reserved.

     Redistribution and use in source and binary forms, with or without
     modification, are permitted provided that the following conditions are met:
         * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
         * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
         * Neither the name of the <organization> nor the
         names of its contributors may be used to endorse or promote products
         derived from this software without specific prior written permission.

     THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
     EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
     DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
     DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
     ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \ A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */
/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */
#include <algorithm>
#include <assert.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
//#include <eigen3>
#include <math.h>




#include "../include/generic_quad_sim/quadrotor_drive.h"

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace gazebo
{
    enum {
        FRONT,
        REAR,
        LEFT,
        RIGHT
    };
    enum
    {
        X,
        Y,
        Z
    };
    enum
    {
        THRUST,
        TAU_PHI,
        TAU_THETA,
        TAU_PSI
    };
    GazeboRosRotorPropulsion::GazeboRosRotorPropulsion()
    {

    }

    GazeboRosRotorPropulsion::~GazeboRosRotorPropulsion()
    {

    }

    void GazeboRosRotorPropulsion::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->parent_ = _parent;
        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "RosRotorPropulsion"));
        gazebo_ros_->isInitialized();
        //BRIEF: Getting the various ros topic parameters
        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_vel");
        gazebo_ros_->getParameter<std::string>(robot_base_frame_, "robotBaseFrame", "base_link");
        gazebo_ros_->getParameter<std::string>(imu_topic_, "imu_data", "imu");
//        gazebo_ros_->getParameter<std::string>(wrench_topic_front_, "front_wrench", "wrench_front");
//        gazebo_ros_->getParameter<std::string>(wrench_topic_rear_, "rear_wrench", "wrench_rear");
//        gazebo_ros_->getParameter<std::string>(wrench_topic_left_, "left_wrench", "wrench_left");
//        gazebo_ros_->getParameter<std::string>(wrench_topic_right_, "right_wrench", "wrench_right");
        gazebo_ros_->getParameter<std::string>(command_force_topic_, "forceTopic", "cmd_force");
//        gazebo_ros_->getParameter<std::string>(robot_namespace_,"robotNameSpace","generic_quad");

        //BRIEF: Getting the boolean parameters for various publishers
        gazebo_ros_->getParameterBoolean(publish_tf_, "publishRotorTF", false);
        gazebo_ros_->getParameterBoolean(publishRotorJointState_, "publishRotorJointState", false);
//        gazebo_ros_->getParameterBoolean(publish_wrench_,"publishWrench", false);


        // BRIEF: Getting the simulation parameters
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);

        gazebo_ros_->getParameter<double>(camera_mass_,"cameraMass", 0.03);
        //BRIEF: Getting the physical dimensions from the urdf file :base/////////////////////////////////////////////
//        camera_mass_ = static_cast<double>(camera_mass_);
        gazebo_ros_->getParameter<double>(base_mass_, "baseMass", 0.3);
//        base_mass_ = static_cast<double>(base_mass_);
        gazebo_ros_->getParameter<double>(base_len_, "baseLength", 0.05);
//        base_len_ = static_cast<double>(base_len_);
//        gazebo_ros_->getParameter<double>(base_wid_, "baseWidth", 0.02);
//        gazebo_ros_->getParameter<double>(base_hei_, "baseHei", 0.02);
        //BRIEF: Getting the physical dimensions from the urdf file: arm//////////////////////////////////////////////
        gazebo_ros_->getParameter<double>(arm_mass_,"armMass", 0.01);
//        arm_mass_ = static_cast<double>(arm_mass_);
        gazebo_ros_->getParameter<double>(arm_len_,"armLength", 0.2);
//        arm_len_ = static_cast<double>(arm_len_);
//        gazebo_ros_->getParameter<double>(arm_wid_,"armWidth", 0.02);
//        gazebo_ros_->getParameter<double>(arm_hei_,"armHeight", 0.02);
        //BRIEF: Getting the physical dimensions from the urdf file: motor////////////////////////////////////////////
        gazebo_ros_->getParameter<double>(motor_mass_,"motorMass", 0.08);
        gazebo_ros_->getParameter<double>(motor_radius_, "motorRadius", 0.02);
//        gazebo_ros_->getParameter<double>(motor_hei_, "motorHeight", 0.02);
//        gazebo_ros_->getParameter<double>(motor_radius_, "motorRadius",0.0);

        //BRIEF: Getting the physical dimensions from the urdf file: Rotor////////////////////////////////////////////
        gazebo_ros_->getParameter<double>(rotor_mass_,"rotorMass", 0.001);
        gazebo_ros_->getParameter<double>(rotor_radius_,"rotorRadius", 0.1);
//        gazebo_ros_->getParameter<double>(rotor_height_,"rotorHeight", 0.005);
        gazebo_ros_->getParameter<double>(rotor_torque_, "rotorTorque", 0.0);
        gazebo_ros_->getParameter<double>(rotor_accel_, "rotorAccel", 0.0);

        //BRIEF: Getting the paramter for the calculation of model force
        // REVIEW: Incorporate Later in the next revision
//        gazebo_ros_->getParameter<std::string>(body_force_cal_flag,"calcFlag", "GetForce");

        //BRIEF: Resize the rotor joints vector///////////////////////////////////////////////////////////////////////
        joints_.resize(4);
        // Remap "frontJoint" to the one from sdf when we include the plugin in the sdf
        joints_[FRONT] = gazebo_ros_->getJoint(_parent,"frontJoint", "front_joint");
        // Remap "rearJoint" to the one from sdf when we include the plugin in the sdf
        joints_[REAR] = gazebo_ros_->getJoint(_parent, "rearJoint", "rear_joint");
        // Remap "leftJoint" to the one from sdf when we include the plugin in the sdf
        joints_[LEFT] = gazebo_ros_->getJoint(_parent, "leftJoint", "left_joint");
        // Remap "rightJoint" to the one from sdf when we include the plugin in the sdf
        joints_[RIGHT]= gazebo_ros_->getJoint(_parent, "rightJoint", "right_joint");

        //BRIEF: Get the base joint name for the reference control parameters
        base_joint_ = gazebo_ros_->getJoint(_parent,"base_link","base_link");

        //BRIEF: Setting the equilibrium force
        total_mass_ = base_mass_ + 4*(arm_mass_ + motor_mass_ + rotor_mass_);
//        eqm_force[FRONT] = total_mass_*GRAV/4.0;
//        eqm_force[REAR]  = total_mass_*GRAV/4.0;
//        eqm_force[LEFT]  = total_mass_*GRAV/4.0;
//        eqm_force[RIGHT] = total_mass_*GRAV/4.0;


        eqm_force[FRONT].x = 0.0;
        eqm_force[FRONT].y = 0.0;
        eqm_force[FRONT].z = total_mass_*GRAV/4.0;

        eqm_force[REAR].x = 0.0;
        eqm_force[REAR].y = 0.0;
        eqm_force[REAR].z = total_mass_*GRAV/4.0;

        eqm_force[RIGHT].x = 0.0;
        eqm_force[RIGHT].y = 0.0;
        eqm_force[RIGHT].z = total_mass_*GRAV/4.0;

        eqm_force[LEFT].x = 0.0;
        eqm_force[LEFT].y = 0.0;
        eqm_force[LEFT].z = total_mass_*GRAV/4.0;

        joints_[FRONT]->GetChild()->AddRelativeForce(eqm_force[FRONT]);
        joints_[REAR]->GetChild()->AddRelativeForce(eqm_force[REAR]);
        joints_[LEFT]->GetChild()->AddRelativeForce(eqm_force[RIGHT]);
        joints_[REAR]->GetChild()->AddRelativeForce(eqm_force[LEFT]);

        //BRIEF: Calculating the lever arm for torque for pitch and roll
        fulcrum = (base_len_/2.0) + arm_len_ - motor_radius_;

        //BRIEF: Setting the equilibrium angular velocities
        eqm_rotor_speed[FRONT].x = 0.0;
        eqm_rotor_speed[FRONT].y = 0.0;
        eqm_rotor_speed[FRONT].z = fabs(sqrt(eqm_force[FRONT].z/K_MOTOR));

        eqm_rotor_speed[REAR].x = 0.0;
        eqm_rotor_speed[REAR].y = 0.0;
        eqm_rotor_speed[REAR].z = fabs(sqrt(eqm_force[REAR].z/K_MOTOR));

        eqm_rotor_speed[LEFT].x = 0.0;
        eqm_rotor_speed[LEFT].y = 0.0;
        eqm_rotor_speed[LEFT].z = fabs(sqrt(eqm_force[LEFT].z/K_MOTOR));

        eqm_rotor_speed[RIGHT].x = 0.0;
        eqm_rotor_speed[RIGHT].y = 0.0;
        eqm_rotor_speed[RIGHT].z = fabs(sqrt(eqm_force[RIGHT].z/K_MOTOR));

        joints_[FRONT]->GetChild()->SetAngularVel(eqm_rotor_speed[FRONT]);//SetVelocity(2,eqm_rotor_speed[FRONT]);
        joints_[REAR]->GetChild()->SetAngularVel(eqm_rotor_speed[REAR]);
        joints_[RIGHT]->GetChild()->SetAngularVel(eqm_rotor_speed[RIGHT]);//SetVelocity(2,eqm_rotor_speed[RIGHT]);
        joints_[LEFT]->GetChild()->SetAngularVel(eqm_rotor_speed[LEFT]);//SetVelocity(2,eqm_rotor_speed[LEFT]);

        //BRIEF: Getting the children of the rotor joints
        //REVIEW: Check if there is a need for getting the children

        //BRIEF: Setting up the max forces on the joints//////////////////////////////////////////////////////////////
        joints_[FRONT]->SetEffortLimit(2,1000);//SetParam("fmax", 2, rotor_torque_);
        joints_[REAR]->SetEffortLimit(2,1000);//SetParam("fmax", 2, rotor_torque_);
        joints_[LEFT]->SetEffortLimit(2,1000);//SetParam("fmax", 2, rotor_torque_);
        joints_[RIGHT]->SetEffortLimit(2,1000);//SetParam("fmax", 2, rotor_torque_);

        //BRIEF:Publishing the transform for the robot////////////////////////////////////////////////////////////////
        this->publish_tf_ = true;
        if (!_sdf->HasElement("publishTF"))
        {
            ROS_WARN("GazeboRosRotorPropulsion Plugin (ns=%s) missing <publishTf>, defaults to %d",
                     this->robot_namespace_.c_str(), this->publish_tf_);
        }
        else
        {
            this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
        }

        //BRIEF:Initializing the update///////////////////////////////////////////////////////////////////////////////
        if(this->update_rate_ >0.0)
        {
            this->update_period_ = 1.0/this->update_rate_;
        } else
        {
            this->update_period_ = 0.0;
        }
        last_update_time_ = _parent->GetWorld()->GetSimTime();

        //BRIEF: Initialize the rotor velocities
        rotor_speed_[FRONT].x = 0.0;
        rotor_speed_[FRONT].y = 0.0;
        rotor_speed_[FRONT].z = 0.0;

        rotor_speed_[REAR].x = 0.0;
        rotor_speed_[REAR].y = 0.0;
        rotor_speed_[REAR].z = 0.0;

        rotor_speed_[RIGHT].x = 0.0;
        rotor_speed_[RIGHT].y = 0.0;
        rotor_speed_[RIGHT].z = 0.0;

        rotor_speed_[LEFT].x = 0.0;
        rotor_speed_[LEFT].y = 0.0;
        rotor_speed_[LEFT].z = 0.0;

        //BRIEF: Initialzing the velocity support ???
        rotor_speed_instr_[FRONT].x = 0.0;
        rotor_speed_instr_[FRONT].y = 0.0;
        rotor_speed_instr_[FRONT].z = 0.0;

        rotor_speed_instr_[RIGHT].x = 0.0;
        rotor_speed_instr_[RIGHT].y = 0.0;
        rotor_speed_instr_[RIGHT].z = 0.0;

        rotor_speed_instr_[LEFT].x = 0.0;
        rotor_speed_instr_[LEFT].y = 0.0;
        rotor_speed_instr_[LEFT].z = 0.0;

        rotor_speed_instr_[RIGHT].x = 0.0;
        rotor_speed_instr_[RIGHT].y = 0.0;
        rotor_speed_instr_[RIGHT].z = 0.0;

        //BRIEF: Initializing the rotor joint forces
        force_joint_[FRONT].x = 0.0;
        force_joint_[FRONT].y = 0.0;
        force_joint_[FRONT].z = 0.0;

        force_joint_[REAR].x = 0.0;
        force_joint_[REAR].y = 0.0;
        force_joint_[REAR].z = 0.0;

        force_joint_[LEFT].x = 0.0;
        force_joint_[LEFT].y = 0.0;
        force_joint_[LEFT].z = 0.0;

        force_joint_[RIGHT].x = 0.0;
        force_joint_[RIGHT].y = 0.0;
        force_joint_[RIGHT].z = 0.0;

        //BRIEF: Initializing the rotor joint forces instructed
        force_joint_inst_[FRONT].x = 0.0;
        force_joint_inst_[FRONT].y = 0.0;
        force_joint_inst_[FRONT].z = 0.0;

        force_joint_inst_[REAR].x = 0.0;
        force_joint_inst_[REAR].y = 0.0;
        force_joint_inst_[REAR].z = 0.0;

        force_joint_inst_[LEFT].x = 0.0;
        force_joint_inst_[LEFT].y = 0.0;
        force_joint_inst_[LEFT].z = 0.0;

        force_joint_inst_[RIGHT].x = 0.0;
        force_joint_inst_[RIGHT].y = 0.0;
        force_joint_inst_[RIGHT].z = 0.0;

        alive_ = true;

        /* SECTION: Setting up the publishers */
        //BRIEF: PUBLISHER-> Joint State
        if (this->publishRotorJointState_)
        {
            joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1000);
            ROS_INFO("%s: Advertise joint_states", gazebo_ros_->info());
        }
        //BRIEF: PUBLISHER-> Transform Broadcaster
        this->transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
        ROS_INFO("%s: Advertisiing transform broadcaster", gazebo_ros_->info());
        //BRIEF: PUBLISHER-> Wrench Publisher
        /*
        this->wrench_data_publisher_front_ = gazebo_ros_->node()->advertise<geometry_msgs::Wrench>(wrench_topic_front_, 1000);
        ROS_INFO("%s: Advetinsing Wrench for front rotor ", gazebo_ros_->info());
        this->wrench_data_publisher_rear_ = gazebo_ros_->node()->advertise<geometry_msgs::Wrench>(wrench_topic_rear_, 1000);
        ROS_INFO("%s: Advetinsing Wrench for rear rotor ", gazebo_ros_->info());
        this->wrench_data_publisher_left_ = gazebo_ros_->node()->advertise<geometry_msgs::Wrench>(wrench_topic_left_, 1000);
        ROS_INFO("%s: Advetinsing Wrench for left rotor ", gazebo_ros_->info());
        this->wrench_data_publisher_right_ = gazebo_ros_->node()->advertise<geometry_msgs::Wrench>(wrench_topic_right_, 1000);
        ROS_INFO("%s: Advetinsing Wrench for right rotor ", gazebo_ros_->info());
        */
        //BRIEF: PUBLISHER -> Command Velocity Publisher
        this->cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1000);
        ROS_INFO("%s: Advertising Command Velocity for the model", gazebo_ros_->info());

        //BRIEF: PUBLISHER -> Command force Publisher
        this->cmd_force_publihser_ = nh_.advertise<geometry_msgs::Quaternion>(command_force_topic_,1000);
        ROS_INFO("%s: Advertizing Command Force for the model", gazebo_ros_->info());

        //BRIEF:ROS-> Subscribe to the velocity command topic (usually "cmd_vel")



        /*SECTION: Creating the Subscribers Options */
        //BRIEF: SUBSCRIBER OPTNS-> Twist Subscriber Options
        ros::SubscribeOptions sub_opts = ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                        boost::bind(&GazeboRosRotorPropulsion::cmdVelCallback, this, _1),
                                        ros::VoidPtr(), &queue_);
        ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());
        //TODO: Write the cmdvelCallback function
        //REVIEW: Done
//        this->cmd_force_subscriber_ = nh_.subscribe(sub_opts);

        //BRIEF: PUBLISHER -> IMU Publisher
        this->imu_publisher_= nh_.advertise<sensor_msgs::Imu>(imu_topic_, 1000);
        ROS_INFO("%s: Advertising IMU values", gazebo_ros_->info());

        //BRIEF: SUBSCRIBER OPTNS-> IMU Subscriber Options
        ros::SubscribeOptions imu_opts = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_topic_, 1,
                                         boost::bind(&GazeboRosRotorPropulsion::imuCallback, this, _1),
                                          ros::VoidPtr(), &queue_);
        //TODO: Write the imuCallback function

        //BRIEF: SUBSCRIBER OPTNS->Wrench subscriber options
        /*
//        ros::SubscribeOptions wrench_opts_front = ros::SubscribeOptions::create<geometry_msgs::Wrench>(wrench_topic_front_,1,
//                        boost::bind(&GazeboRosRotorPropulsion::frontwrenchCallback, this, _1), ros::VoidPtr(), &queue_);
//
//        ros::SubscribeOptions wrench_opts_rear = ros::SubscribeOptions::create<geometry_msgs::Wrench>(wrench_topic_rear_, 1,
//                         boost::bind(&GazeboRosRotorPropulsion::rearwrenchCallback, this, _1), ros::VoidPtr(), &queue_);
//
//        ros::SubscribeOptions wrench_opts_left = ros::SubscribeOptions::create<geometry_msgs::Wrench>(wrench_topic_left_, 1,
//                         boost::bind(&GazeboRosRotorPropulsion::leftwrenchCallback, this, _1), ros::VoidPtr(), &queue_);
//
//        ros::SubscribeOptions wrench_opts_right = ros::SubscribeOptions::create<geometry_msgs::Wrench>(wrench_topic_right_, 1,
//                        boost::bind(&GazeboRosRotorPropulsion::rightwrenchCallback, this, _1), ros::VoidPtr(), &queue_);
        */
        ros::SubscribeOptions command_force_opts = ros::SubscribeOptions::create<geometry_msgs::Quaternion>(command_force_topic_, 4,
                                                                                                            boost::bind(&GazeboRosRotorPropulsion::commandforceCallback, this, _1), ros::VoidPtr(), &queue_);
        //TODO: Write the wrenchCallback function
        //REVIEW: Done below starting from line 488 (search tag: wrenchCallback)

        /*SECTION: Creating the Subscribers */
        //BRIEF: SUBSCRIBER-> Twist Subscribers
        this->cmd_vel_subscriber_ = nh_.subscribe(sub_opts);
        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

        //BRIEF: SUBSCRIBER-> IMU Subscriber
        this->imu_data_subscriber_ = nh_.subscribe(imu_opts);
        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), imu_topic_.c_str());

        //BRIEF: SUBSCRIBER-> Wrench Subscriber
        //TODO: Check to see if there is a need to write a wrench subscriber
        // REVIEW: Need a wrench subscriber
        /*
//        //Front Rotor joint wrench
//        this->wrench_data_subscriber_front_ = gazebo_ros_->node()->subscribe(wrench_opts_front);
//        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), wrench_topic_front_.c_str());
//
//        //Rear joint wrench
//        this->wrench_data_subscriber_rear_ = gazebo_ros_->node()->subscribe(wrench_opts_rear);
//        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), wrench_topic_rear_.c_str());
//
//        //Left joint wrench
//        this->wrench_data_subscriber_left_ = gazebo_ros_->node()->subscribe(wrench_opts_left);
//        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), wrench_topic_left_.c_str());
//
//        //Right joint wrench
//        this->wrench_data_subscriber_right_ = gazebo_ros_->node()->subscribe(wrench_opts_right);
//        ROS_INFO("%s: Subscribe to %s", gazebo_ros_->info(), wrench_topic_right_.c_str());
        */
        //BRIEF: SUBSCRIBER-> Command Force subscriber
        this->cmd_force_subscriber_ = nh_.subscribe(command_force_opts);
        ROS_INFO("%s: Subscribed to %s", gazebo_ros_->info(), command_force_topic_.c_str());

        //BRIEF: QUEUE Thread
        // Start custom queue for rotor propulsion
        this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosRotorPropulsion::QueueThread, this));

        //Listen to the update event (broadcast every simulation iteration)
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboRosRotorPropulsion::UpdateChild, this));

    }

    void GazeboRosRotorPropulsion::UpdateChild()
    {
        for(int i =0; i<4; i++)
        {
            if (fabs(rotor_torque_ - joints_[i]->GetEffortLimit(2) > 1e-6))
            {
                joints_[i]->SetEffortLimit(2, rotor_torque_);//SetParam("fmax", 2, rotor_torque_);
            }
        }
        //TODO: Check if you need to update the odometry update function
        common::Time current_time = this->parent_->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_)
        {
            this->publishCmdvel();
            this->pubishImu();
            this->publishCmdForce();
            if (this->publishRotorTF_)
            {
                this->publishRotorTF();
            }
            if (this->publishRotorJointState_)
            {
                this->publishRotorJointState();
            }
            //BRIEF: Getting the rotor velocities at different joints
            current_rotor_speed[FRONT].z = joints_[FRONT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
            current_rotor_speed[REAR].z =joints_[REAR]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
            current_rotor_speed[RIGHT].z =joints_[RIGHT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
            current_rotor_speed[LEFT].z =joints_[LEFT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);

            if ((this->rotor_accel_ == 0) ||
                (fabs(rotor_speed_[FRONT].z - current_rotor_speed[FRONT].z) < 0.01) ||
                (fabs(rotor_speed_[REAR].z - current_rotor_speed[REAR].z) < 0.01) ||
                (fabs(rotor_speed_[LEFT].z - current_rotor_speed[LEFT].z) < 0.01) ||
                (fabs(rotor_speed_[RIGHT].z - current_rotor_speed[RIGHT].z) < 0.01))
            {
                joints_[FRONT]->GetChild()->SetAngularVel(rotor_speed_[FRONT]);//SetVelocity(2, rotor_speed_[FRONT]);   //SetParam("vel", 2, rotor_speed_[FRONT]/rotor_radius_);
                joints_[REAR]->GetChild()->SetAngularVel(rotor_speed_[REAR]);//     //SetParam("vel", 2, rotor_speed_[REAR]/rotor_radius_);
                joints_[LEFT]->GetChild()->SetAngularVel(rotor_speed_[LEFT]);//     //SetParam("vel", 2, rotor_speed_[LEFT]/rotor_radius_);
                joints_[RIGHT]->GetChild()->SetAngularVel(rotor_speed_[FRONT]);//   //SetParam("vel", 2, rotor_speed_[RIGHT]/rotor_radius_);
            }
            else
            {
                //BRIEF:Check the front motor speed
                if (rotor_speed_[FRONT].z >= current_rotor_speed[FRONT].z)
                {
                    rotor_speed_instr_[FRONT].z += fmin(rotor_speed_[FRONT].z - current_rotor_speed[FRONT].z, 1000.0);
                }
                else
                {
                    rotor_speed_instr_[FRONT].z -= fmax(rotor_speed_[FRONT].z - current_rotor_speed[FRONT].z, -1000.0);
                }
                ROS_INFO("actual front rotor speed = %lf, issued front rotor speed= %lf",
                         current_rotor_speed[FRONT].z, rotor_speed_instr_[FRONT].z);
                //check rear motor speed
                if ( rotor_speed_[REAR].z >= current_rotor_speed[REAR].z)
                {
                    rotor_speed_instr_[REAR].z += fmin(rotor_speed_[REAR].z - current_rotor_speed[REAR].z, 1000.0);
                }
                else
                {
                    rotor_speed_instr_[REAR].z += fmax(rotor_speed_[REAR].z - current_rotor_speed[REAR].z, -1000.0);
                }
                ROS_INFO("actual Rear rotor speed = %lf, issued Rear rotor speed= %lf",
                         current_rotor_speed[REAR].z, rotor_speed_instr_[REAR].z);
                // Check right motor speed
                if ( rotor_speed_[RIGHT].z >= current_rotor_speed[RIGHT].z )
                {
                    rotor_speed_instr_[RIGHT].z += fmin(rotor_speed_[RIGHT].z - current_rotor_speed[RIGHT].z, 1000.0);
                }
                else
                {
                    rotor_speed_instr_[RIGHT].z += fmax(rotor_speed_[RIGHT].z - current_rotor_speed[RIGHT].z, -1000.0);
                }
                ROS_INFO("actual right rotor speed = %lf, issued right rotor speed= %lf",
                         current_rotor_speed[RIGHT].z, rotor_speed_instr_[RIGHT].z);
                //Check left motor speed
                if ( rotor_speed_[LEFT].z >= current_rotor_speed[LEFT].z )
                {
                    rotor_speed_instr_[LEFT].z += fmin(rotor_speed_[LEFT].z - current_rotor_speed[LEFT].z, 1000.0);
                }
                else
                {
                    rotor_speed_instr_[LEFT].z += fmax(rotor_speed_[LEFT].z - current_rotor_speed[LEFT].z, -1000.0);
                }
                ROS_INFO("actual left rotor speed = %lf, issued left rotor speed= %lf",
                         current_rotor_speed[LEFT].z, rotor_speed_instr_[LEFT].z);
                //BRIEF:Setting the individual rotor joint speeds
                joints_[FRONT]->GetChild()->SetAngularVel(rotor_speed_instr_[FRONT]);//SetVelocity(2,rotor_speed_instr_[FRONT]);//SetParam("vel",2, rotor_speed_instr_[FRONT]/(rotor_radius_));
                joints_[REAR]->GetChild()->SetAngularVel(rotor_speed_instr_[REAR]);//SetVelocity(2,rotor_speed_instr_[REAR]);//SetParam("vel",2, rotor_speed_instr_[REAR]/(rotor_radius_));
                joints_[RIGHT]->GetChild()->SetAngularVel(rotor_speed_instr_[RIGHT]);//SetVelocity(2,rotor_speed_instr_[RIGHT]);//SetParam("vel",2, rotor_speed_instr_[RIGHT]/(rotor_radius_));
                joints_[LEFT]->GetChild()->SetAngularVel(rotor_speed_instr_[LEFT]);//SetVelocity(2,rotor_speed_instr_[LEFT]);//SetParam("vel",2, rotor_speed_instr_[LEFT]/(rotor_radius_));

                //BRIEF: Calling the ang_vel_to_force function  to convert the commanded forces to the angular velocities
                // Step 1.1: Getting the current body forces

                body_force[FRONT] = joints_[FRONT]->GetParent()->GetRelativeForce().z;
                body_force[FRONT] = joints_[FRONT]->GetParent()->GetRelativeForce().z;
                body_force[FRONT] = joints_[FRONT]->GetParent()->GetRelativeForce().z;
                body_force[FRONT] = joints_[FRONT]->GetParent()->GetRelativeForce().z;
                /*
                body_force[FRONT] = joints_[FRONT]->GetForceTorque(2).body1Force.z;
                body_force[REAR] = joints_[REAR]->GetForceTorque(2).body1Force.z;
                body_force[LEFT] = joints_[LEFT]->GetForceTorque(2).body1Force.z;
                body_force[RIGHT] = joints_[RIGHT]->GetForceTorque(2).body1Force.z;
                */
                // Step 1.2: Getting the current body velocities
                body_ang_vel[FRONT] = joints_[FRONT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
                body_ang_vel[REAR] = joints_[REAR]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
                body_ang_vel[LEFT] = joints_[LEFT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);
                body_ang_vel[RIGHT] = joints_[RIGHT]->GetChild()->GetRelativeAngularVel().z;//GetVelocity(2);

                //Step 2: Convert the body forces into the current comman))d forces like thrust, roll,pitch and yaw torques
                force_vec<< body_force[FRONT],
                        body_force[REAR],
                        body_force[LEFT],
                        body_force[RIGHT];

                force_vec_to_curr_commanded_force<<1, 1, 1,1,
                        0, 0, -1, 1,
                        -1, 1, 0, 0,
                        1, 1, -1, -1;

                curr_commanded_force = force_vec_to_curr_commanded_force* force_vec;

                //Step 3.1: Get the commanded forces
                commanded_forces_eigen<<commanded_force[THRUST],
                        commanded_force[TAU_PHI],
                        commanded_force[TAU_THETA],
                        commanded_force[TAU_PSI];

                //Step 3.2: Calculating the commanded rotor rpm

                commanded_to_rotor_rpm<<(1/(4*K_MOTOR)), 0, -(1/(2*K_MOTOR*fulcrum)), (1/(4*B_MOTOR)),
                        (1/(4*K_MOTOR)), 0,  (1/(2*K_MOTOR*fulcrum)), (1/(4*B_MOTOR)),
                        (1/(4*K_MOTOR)), (1/(2*K_MOTOR*fulcrum)), 0, -(1/(4*B_MOTOR)),
                        (1/(4*K_MOTOR)), -(1/(2*K_MOTOR*fulcrum)), 0, -(1/(4*B_MOTOR));


                commanded_rotor_rpm = commanded_to_rotor_rpm*commanded_forces_eigen;

                //Step 4: Calculating the commanded rotor joint torques
                /*
                Eigen::Vector4d commanded_rotor_forces_eigen;
                Eigen::Matrix4d commanded_to_rotor_forces;
                 */
                commanded_to_rotor_forces<<0.25, 0 -0.5, 0.25,
                        0.25, 0, 0.5, 0.25,
                        0.25, 0.5, 0, -0.25,
                        0.25, -0.5, 0, -0.25;

                commanded_rotor_forces_eigen = commanded_to_rotor_forces*commanded_forces_eigen;


                /*
                // Writing the difference between the commanded forces from the current forces
                //REVIEW: Forces are additive according to the documentation.
                // REVIEW: May be we need to just write the current force - the requested force
//                //Step 5.1: Current joint forces from the model ( using GetForce)
//                curr_joint_force<<joints_[FRONT]->GetForce(2),
//                                  joints_[REAR]->GetForce(2),
//                                  joints_[LEFT]->GetForce(2),
//                                  joints_[RIGHT]->GetForce(2);
//
//                //Step 5.2: Current Joint forces from the model (using GetVelocity)
//                curr_joint_rpm<<joints_[FRONT]->GetVelocity(2),
//                                joints_[REAR]->GetVelocity(2),
//                                joints_[LEFT]->GetVelocity(2),
//                                joints_[RIGHT]->GetVelocity(2);
//
//                curr_joint_rpm_to_force = K_MOTOR *curr_joint_rpm;
                */

                //Brief: current rotor joint forces
                /*
                curr_force[FRONT] = wrench_data_[FRONT].force[Z];
                curr_force[REAR] = wrench_data_[REAR].force[Z];
                curr_force[LEFT] = wrench_data_[LEFT].force[Z];
                curr_force[RIGHT] = wrench_data_[RIGHT].force[Z];
                */
                //Step 6: Calculating the net force change to be applied
                net_force[FRONT] = commanded_rotor_forces_eigen(FRONT) - body_force[FRONT];
                net_force[REAR] = commanded_rotor_forces_eigen(REAR) - body_force[REAR];
                net_force[LEFT] = commanded_rotor_forces_eigen(LEFT) - body_force[LEFT];
                net_force[RIGHT] = commanded_rotor_forces_eigen(RIGHT) - body_force[RIGHT];

                total_force[FRONT]= eqm_force[FRONT].z + net_force[FRONT];
                total_force[REAR]= eqm_force[REAR].z + net_force[REAR];
                total_force[LEFT]= eqm_force[LEFT].z + net_force[LEFT];
                total_force[RIGHT]= eqm_force[RIGHT].z + net_force[RIGHT];

                link_force[FRONT].x = 0.0;
                link_force[FRONT].y = 0.0;
                link_force[FRONT].z = total_force[FRONT];
                joints_[FRONT]->GetChild()->AddRelativeForce(link_force[FRONT]);

                link_force[REAR].x = 0.0;
                link_force[REAR].y = 0.0;
                link_force[REAR].z = total_force[REAR];
                joints_[REAR]->GetChild()->AddRelativeForce(link_force[REAR]);

                link_force[LEFT].x = 0.0;
                link_force[LEFT].y = 0.0;
                link_force[LEFT].z = total_force[LEFT];
                joints_[LEFT]->GetChild()->AddRelativeForce(link_force[LEFT]);

                link_force[RIGHT].x = 0.0;
                link_force[RIGHT].y = 0.0;
                link_force[RIGHT].z = total_force[RIGHT];
                joints_[RIGHT]->GetChild()->AddRelativeForce(link_force[RIGHT]);

                /*
                joints_[FRONT]->SetForce(2,total_force[FRONT]);
                joints_[REAR]->SetForce(2,total_force[REAR]);
                joints_[RIGHT]->SetForce(2,total_force[RIGHT]);
                joints_[LEFT]->SetForce(2,total_force[LEFT]);
                 */
                //REVIEW: FIgure out a way to write yaw torque
                // Maybe ::::::: Write the torque to the centre of body of the quadrotor

                curr_base_yaw_torque = base_joint_->GetParent()->GetRelativeTorque().z;
                commanded_torque = B_MOTOR*(total_force[FRONT] + total_force[REAR] - total_force[LEFT] - total_force[RIGHT]);

                net_torque_yaw_base = commanded_torque - curr_base_yaw_torque;
                base_yaw_torque.x = 0.0;
                base_yaw_torque.y = 0.0;
                base_yaw_torque.z = net_torque_yaw_base;
                base_joint_->GetParent()->AddRelativeTorque(base_yaw_torque);

                total_force_quat.x = total_force[FRONT];
                total_force_quat.y = total_force[REAR];
                total_force_quat.z = total_force[LEFT];
                total_force_quat.w = total_force[RIGHT];

            }
            last_update_time_ +=common::Time(update_period_);
        }

    }

    void GazeboRosRotorPropulsion::Reset()
    {
        last_update_time_ = this->parent_->GetWorld()->GetSimTime();
        pose_encoder_.position.x = 0;
        pose_encoder_.position.y = 0;
        pose_encoder_.position.z = 0;
        pose_encoder_.orientation.x = 0;
        pose_encoder_.orientation.y = 0;
        pose_encoder_.orientation.z = 0;
        pose_encoder_.orientation.w = 0;
        joints_[FRONT]->SetParam("fmax", 0, rotor_torque_);
        joints_[REAR]->SetParam("fmax", 0, rotor_torque_);
        joints_[RIGHT]->SetParam("fmax", 0, rotor_torque_);
        joints_[LEFT]->SetParam("fmax", 0, rotor_torque_);
        ModelPlugin::Reset();
    }

    void GazeboRosRotorPropulsion::publishRotorJointState()
    {
        ros::Time current_time = ros::Time::now();

        joint_state_.header.stamp = current_time;
        joint_state_.name.resize(joints_.size());
        joint_state_.position.resize(joints_.size());

        for (int i=0; i<4; i++)
        {
            physics::JointPtr joint = joints_[i];
            math::Angle angle = joint->GetAngle(2);
            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = angle.Radian();
        }
        joint_state_publisher_.publish(joint_state_);
    }

    void GazeboRosRotorPropulsion::publishRotorTF()
    {
        ros::Time current_time = ros::Time::now();
        for(int i =0; i<4; i++)
        {
            std::string rotor_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName());
            std::string rotor_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName());

            math::Pose poseRotor = joints_[i]->GetChild()->GetRelativePose();

            tf::Quaternion qt(poseRotor.rot.x, poseRotor.rot.y, poseRotor.rot.z, poseRotor.rot.w);
            tf::Vector3 vt(poseRotor.pos.x, poseRotor.pos.y, poseRotor.pos.z); // ????

            tf::Transform tfRotor (qt, vt);
            transform_broadcaster_->sendTransform(tf::StampedTransform(tfRotor, current_time, rotor_parent_frame, rotor_frame));
        }
    }
    //TODO: Finish writing the wrench publisher
    /*
    void GazeboRosRotorPropulsion::publishRotorWrenchFront()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);
        wrench_data_publisher_front_.publish(wrench_data_[FRONT]);
    }
/*
    void GazeboRosRotorPropulsion::frontwrenchCallback(const geometry_msgs::Wrench::ConstPtr &wrench_msg_front)
    {
        boost::mutex::scoped_lock(lock);

//        wrench_body[FRONT].body1Force.x = wrench_msg_front->force.x;
//        wrench_body[FRONT].body1Force.y = wrench_msg_front->force.y;
//        wrench_body[FRONT].body1Force.z = wrench_msg_front->force.z;

        wrench_data_[FRONT].force[X] = wrench_msg_front->force.x;
        wrench_data_[FRONT].force[Y] = wrench_msg_front->force.y;
        wrench_data_[FRONT].force[Z] = wrench_msg_front->force.z;

//        wrench_body[FRONT].body1Torque.x = wrench_msg_front->torque.x;
//        wrench_body[FRONT].body1Torque.y = wrench_msg_front->torque.y;
//        wrench_body[FRONT].body1Torque.z = wrench_msg_front->torque.z;

        wrench_data_[FRONT].torque[X] = wrench_msg_front->torque.x;
        wrench_data_[FRONT].torque[Y] = wrench_msg_front->torque.y;
        wrench_data_[FRONT].torque[Z] = wrench_msg_front->torque.z;
    }

    void GazeboRosRotorPropulsion::publishRotorWrenchRear()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);
        wrench_data_publisher_rear_.publish(wrench_data_[REAR]);
    }

    void GazeboRosRotorPropulsion::rearwrenchCallback(const geometry_msgs::Wrench::ConstPtr &wrench_msg_rear)
    {
        boost::mutex::scoped_lock(lock);
        wrench_data_[REAR].force[X] = wrench_msg_rear->force.x;
        wrench_data_[REAR].force[Y] = wrench_msg_rear->force.y;
        wrench_data_[REAR].force[Z] = wrench_msg_rear->force.z;

        wrench_data_[REAR].torque[X] = wrench_msg_rear->torque.x;
        wrench_data_[REAR].torque[Y] = wrench_msg_rear->torque.y;
        wrench_data_[REAR].torque[Z] = wrench_msg_rear->torque.z;
    }

    void GazeboRosRotorPropulsion::publishRotorWrenchLeft()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);
        wrench_data_publisher_left_.publish(wrench_data_[LEFT]);
    }

    void GazeboRosRotorPropulsion::leftwrenchCallback(const geometry_msgs::Wrench::ConstPtr &wrench_msg_left)
    {
        boost::mutex::scoped_lock(lock);
        wrench_data_[LEFT].force[X] = wrench_msg_left->force.x;
        wrench_data_[LEFT].force[Y] = wrench_msg_left->force.y;
        wrench_data_[LEFT].force[Z] = wrench_msg_left->force.z;

        wrench_data_[LEFT].torque[X] = wrench_msg_left->torque.x;
        wrench_data_[LEFT].torque[Y] = wrench_msg_left->torque.y;
        wrench_data_[LEFT].torque[Z] = wrench_msg_left->torque.z;
    }

    void GazeboRosRotorPropulsion::publishRotorWrenchRight()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);
        wrench_data_publisher_right_.publish(wrench_data_[RIGHT]);
    }

    void GazeboRosRotorPropulsion::rightwrenchCallback(const geometry_msgs::Wrench::ConstPtr &wrench_msg_right)
    {
        boost::mutex::scoped_lock(lock);
        wrench_data_[RIGHT].force[X] = wrench_msg_right->force.x;
        wrench_data_[RIGHT].force[Y] = wrench_msg_right->force.y;
        wrench_data_[RIGHT].force[Z] = wrench_msg_right->force.z;

        wrench_data_[RIGHT].torque[X] = wrench_msg_right->torque.x;
        wrench_data_[RIGHT].torque[Y] = wrench_msg_right->torque.y;
        wrench_data_[RIGHT].torque[Z] = wrench_msg_right->torque.z;
    }
*/
    void GazeboRosRotorPropulsion::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg)
    {
        boost::mutex::scoped_lock(lock);
        vel_vec_twist.linear.x = cmd_msg->linear.x;
        vel_vec_twist.linear.y = cmd_msg->linear.y;
        vel_vec_twist.linear.z = cmd_msg->linear.z;

        vel_vec_twist.angular.x = cmd_msg->angular.x;
        vel_vec_twist.angular.y = cmd_msg->angular.y;
        vel_vec_twist.angular.z = cmd_msg->angular.z;

        /*
        v_curr_lin_[X] = cmd_msg->linear.x;
        v_curr_lin_[Y] = cmd_msg->linear.y;
        v_curr_lin_[Z] = cmd_msg->linear.z;

        v_curr_ang_[X] = cmd_msg->angular.x;
        v_curr_ang_[Y] = cmd_msg->angular.y;
        v_curr_ang_[Z] = cmd_msg->angular.z;
        vel_vec(6);
        vel_vec(0) = v_curr_lin_[X];
        vel_vec(1) = v_curr_lin_[Y];
        vel_vec(2) = v_curr_lin_[Z];
        vel_vec(3) = v_curr_ang_[X];
        vel_vec(4) = v_curr_ang_[Y];
        vel_vec(5) = v_curr_ang_[Z];

        vel_vec(0,0) = v_curr_lin_[X];
        vel_vec(1,0) = v_curr_lin_[Y];
        vel_vec(2,0) = v_curr_lin_[Z];
        vel_vec(3,0) = v_curr_ang_[X];
        vel_vec(4,0) = v_curr_ang_[Y];
        vel_vec(5,0) = v_curr_ang_[Z];
         */
    }

    void GazeboRosRotorPropulsion::publishCmdvel()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);

        this->cmd_vel_publisher_.publish(vel_vec_twist);
        /*
//        double vel_cg[6];
//        geometry_msgs::Twist vel_cg;
//        vel_cg.linear.x = parent_->GetRelativeLinearVel().x; //base_joint->GetVelocity(0);
//        vel_cg.linear.y= parent_->GetRelativeLinearVel().y; //base_joint->GetVelocity(0);
//        vel_cg.linear.z = parent_->GetRelativeLinearVel().z; //base_joint->GetVelocity(0);
//        vel_cg.angular.x = parent_->GetRelativeAngularVel().x;
//        vel_cg.angular.y = parent_->GetRelativeAngularVel().y;
//        vel_cg.angular.z = parent_->GetRelativeAngularVel().z;
//        this->cmd_vel_publisher_.publish(vel_cg);
         */
    }

    void GazeboRosRotorPropulsion::commandforceCallback(const geometry_msgs::Quaternion::ConstPtr &cmd_force)
    {
        boost::mutex::scoped_lock(lock);
        commanded_force[THRUST] = cmd_force->x;
        commanded_force[TAU_PHI] = cmd_force->y;
        commanded_force[TAU_THETA] = cmd_force->z;
        commanded_force[TAU_PSI] = cmd_force->w;
    }


    void GazeboRosRotorPropulsion::publishCmdForce()
    {
        ros::Time current_time = ros::Time::now();
        boost::mutex::scoped_lock(lock);
        this->cmd_force_publihser_.publish(total_force_quat);

    }

    void GazeboRosRotorPropulsion::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        boost::mutex::scoped_lock(lock);
        imu_data_.orientation.x = imu_msg->orientation.x;
        imu_data_.orientation.y = imu_msg->orientation.y;
        imu_data_.orientation.z = imu_msg->orientation.z;
        imu_data_.orientation.w = imu_msg->orientation.w;

        imu_data_.linear_acceleration.x = imu_msg->linear_acceleration.x;
        imu_data_.linear_acceleration.y = imu_msg->linear_acceleration.y;
        imu_data_.linear_acceleration.z = imu_msg->linear_acceleration.z;

        imu_data_.angular_velocity.x = imu_msg->angular_velocity.x;
        imu_data_.angular_velocity.y = imu_msg->angular_velocity.y;
        imu_data_.angular_velocity.z = imu_msg->angular_velocity.z;
    }

    void GazeboRosRotorPropulsion::pubishImu()
    {
        boost::mutex::scoped_lock(lock);
        this->imu_publisher_.publish(imu_data_);
    }

    void GazeboRosRotorPropulsion::QueueThread()
    {
        static const double timeout = 0.01;
        while (alive_ and gazebo_ros_->node()->ok())
        {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void GazeboRosRotorPropulsion::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosRotorPropulsion)

}
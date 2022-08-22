/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

/*
 * Desc: Torque controlled three-wheeled omni wheel driver.
 * Author: Brian Tu
 * Date: 15 July 2022
 */

#ifndef GAZEBO_ROS_PLANAR_MOVE_HH
#define GAZEBO_ROS_PLANAR_MOVE_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <control_toolbox/pid.h>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class GazeboRosOmniMove : public ModelPlugin {

    public:
      GazeboRosOmniMove();
      ~GazeboRosOmniMove();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      physics::ModelPtr parent_;
      physics::JointPtr wheel_joint_r;
      physics::JointPtr wheel_joint_l;
      physics::JointPtr wheel_joint_b;
      bool wheel_islock_r;
      bool wheel_islock_l;
      bool wheel_islock_b;
      double wheel_lock_r;
      double wheel_lock_l;
      double wheel_lock_b;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Publisher w_r_pub_;
      ros::Publisher w_l_pub_;
      ros::Publisher w_b_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;
      double cmd_timeout_;
      double wheel_radius_;
      double wheel_track_;
      ros::Time last_cmd_received_time_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
      void Reset();

      double x_;
      double y_;
      double rot_;
      double x_target;
      double y_target;
      double rot_target;
      bool alive_;
      common::Time last_odom_publish_time_;
      ignition::math::Pose3d last_odom_pose_;

  };

}

#endif /* end of include guard: GAZEBO_ROS_PLANAR_MOVE_HH */
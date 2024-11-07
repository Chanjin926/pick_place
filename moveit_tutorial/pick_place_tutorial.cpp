/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <math.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <array>


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].effort.resize(2);
  posture.points[0].effort[0] = 0.0;
  posture.points[0].effort[1] = 0.0;
  /* Set them as open, wide enough for the object to fit. */
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;

  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{ 
   posture.joint_names.resize(2); 
   posture.joint_names[0] = "panda_finger_joint1"; 
   posture.joint_names[1] = "panda_finger_joint2";

   posture.points.resize(1);
   posture.points[0].positions.resize(2);
   posture.points[0].positions[0] = 0.02; // Close gripper
   posture.points[0].positions[1] = 0.02;

   posture.points.resize(1);
   posture.points[0].effort.resize(2);
   posture.points[0].effort[0] = 10.0;
   posture.points[0].effort[1] = 10.0;
   posture.points[0].time_from_start = ros::Duration(5);
}



void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Allow collisions of panda_hand_sc with object
  grasps[0].allowed_touch_objects.push_back("panda_hand_sc");
  //grasps[0].allowed_touch_objects.push_back("peg"); // 'peg'과의 접촉을 허용


  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  // closedGripper(grasps[0].grasp_posture);
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("peg", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("peg", place_location);
  // END_SUB_TUTORIAL
}

void genSpiral(const std::vector<float>& params, std::vector<float>& xX, std::vector<float>& yY, std::vector<float>& zZ)
{
  // params: [radius_start, radius_end, height, num_points, turns]
  float radius_start = params[0];
  float radius_end = params[1];
  float height = params[2];
  int num_points = static_cast<int>(params[3]);
  float turns = params[4];

  for (int i = 0; i < num_points; i++)
  {
    float t = static_cast<float>(i) / static_cast<float>(num_points);
    float angle = t * turns * 2 * M_PI;
    float radius = radius_start * (1.0f - t) + radius_end * t;
    float z = height * t;

    float x = radius * cos(angle);
    float y = radius * sin(angle);

    xX.push_back(x);
    yY.push_back(y);
    zZ.push_back(-z);
  }
}

float getAngle(std::array<float,2> start, std::array<float,2> center, std::array<float,2> curr)
{
  float x0 = start[0] - center[0];
  float y0 = start[1] - center[1];

  float x = curr[0] - center[0];
  float y = curr[1] - center[1];

  float ct = (x0*x + y0*y) / (sqrt(pow(x0, 2) + pow(y0, 2))*sqrt(pow(x, 2) + pow(y, 2)));
  float st = sqrt(1 - pow(ct, 2));
  float theta = atan2(st, ct);
  
  return theta;
}

void spiralTrajectory(moveit::planning_interface::MoveGroupInterface& group)
{
  geometry_msgs::Pose current_pose = group.getCurrentPose().pose;

  // Get the starting point from the current pose
  float start_x = current_pose.position.x;
  float start_y = current_pose.position.y;
  float start_z = current_pose.position.z;

  // Get the waypoints of conical spiral
  std::vector<float> params = {0.05, 0.01, 0.1, 100, 3.0};
  std::vector<float> xX, yY, zZ;
  genSpiral(params, xX, yY, zZ);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  float end_x = start_x + xX.back();
  float end_y = start_y + yY.back();
  float end_z = start_z + zZ.back();

  for (int i = 0; i < xX.size(); i++)
  {
    // Planning for the next pose
    geometry_msgs::Pose next_pose = current_pose;
    next_pose.position.x = start_x + xX[i];
    next_pose.position.y = start_y + yY[i];
    next_pose.position.z = start_z + zZ[1];
    
    /* // Add orientation & setRPY
    // Compute direction vector
    float dx = end_x - next_pose.position.x;
    float dy = end_y - next_pose.position.y;
    float dz = end_z - next_pose.position.z;

    // Compute Roll, Pitch, Yaw
    float roll = atan2(dy, dz);
    float pitch = atan2(dz, sqrt(dx * dx + dy * dy));
    float yaw = atan2(dy, dx);
    

    // Set orientation
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    next_pose.orientation = tf2::toMsg(orientation); */

    waypoints.push_back(next_pose);
  }
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  group.execute(trajectory);
}

void partialSFT(moveit::planning_interface::MoveGroupInterface& group, float tlim)
{
  geometry_msgs::Pose current_pose = group.getCurrentPose().pose;

  // Get the starting point from the current pose
  float start_x = current_pose.position.x;
  float start_y = current_pose.position.y;
  float start_z = current_pose.position.z;

  // Get the waypoints of conical spiral
  std::vector<float> params = {0.05, 0.01, 0.1, 100, 3.0};
  std::vector<float> xX, yY, zZ;
  genSpiral(params, xX, yY, zZ);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  float end_x = start_x + xX.back();
  float end_y = start_y + yY.back();
  float end_z = start_z + zZ.back();

  std::array<float, 2> start = {start_x, start_y};
  std::array<float, 2> center = {end_x, end_y};

  for (int i = 0; i < xX.size(); i++)
  {
    // Planning for the next pose
    geometry_msgs::Pose next_pose = current_pose;
    std::array<float, 2> curr = {xX[i], yY[i]};
    float theta = getAngle(start, center, curr);
    if (theta < tlim)
    {

      next_pose.position.x = start_x + xX[i];
      next_pose.position.y = start_y + yY[i];
      next_pose.position.z = start_z + zZ[i];

      waypoints.push_back(next_pose);
    }
  

  }
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  group.execute(trajectory);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(7);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "peg";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.515;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  // Define the hole
  collision_objects[3].header.frame_id = "panda_link0";
  collision_objects[3].id = "hole1";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.02;
  collision_objects[3].primitives[0].dimensions[1] = 0.065;
  collision_objects[3].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.0225;
  collision_objects[3].primitive_poses[0].position.y = 0.5;
  collision_objects[3].primitive_poses[0].position.z = 0.425;

  collision_objects[3].operation = collision_objects[3].ADD;

  // Define the hole
  collision_objects[4].header.frame_id = "panda_link0";
  collision_objects[4].id = "hole2";

  /* Define the primitive and its dimensions. */
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 0.02;
  collision_objects[4].primitives[0].dimensions[1] = 0.065;
  collision_objects[4].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = -0.0225;
  collision_objects[4].primitive_poses[0].position.y = 0.5;
  collision_objects[4].primitive_poses[0].position.z = 0.425;

  collision_objects[4].operation = collision_objects[4].ADD;

  // Define the hole
  collision_objects[5].header.frame_id = "panda_link0";
  collision_objects[5].id = "hole3";

  /* Define the primitive and its dimensions. */
  collision_objects[5].primitives.resize(1);
  collision_objects[5].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[5].primitives[0].dimensions.resize(3);
  collision_objects[5].primitives[0].dimensions[0] = 0.025;
  collision_objects[5].primitives[0].dimensions[1] = 0.02;
  collision_objects[5].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[5].primitive_poses.resize(1);
  collision_objects[5].primitive_poses[0].position.x = 0;
  collision_objects[5].primitive_poses[0].position.y = 0.4775;
  collision_objects[5].primitive_poses[0].position.z = 0.425;

  collision_objects[5].operation = collision_objects[5].ADD;

  // Define the hole
  collision_objects[6].header.frame_id = "panda_link0";
  collision_objects[6].id = "hole4";

  /* Define the primitive and its dimensions. */
  collision_objects[6].primitives.resize(1);
  collision_objects[6].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[6].primitives[0].dimensions.resize(3);
  collision_objects[6].primitives[0].dimensions[0] = 0.025;
  collision_objects[6].primitives[0].dimensions[1] = 0.02;
  collision_objects[6].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[6].primitive_poses.resize(1);
  collision_objects[6].primitive_poses[0].position.x = 0;
  collision_objects[6].primitive_poses[0].position.y = 0.5225;
  collision_objects[6].primitive_poses[0].position.z = 0.425;

  collision_objects[6].operation = collision_objects[6].ADD;
  // END_SUB_TUTORIAL

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  // moveit::planning_interface::MoveGroupInterface gripper("panda_hand");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);
  // Synchronize the planning scene with Gazebo
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);
  //spiralTrajectory(group);
  //partialSFT(group, M_PI/12);

  ros::waitForShutdown();
  return 0;
}

// BEGIN_TUTORIAL
// CALL_SUB_TUTORIAL table1
// CALL_SUB_TUTORIAL table2
// CALL_SUB_TUTORIAL object
//
// Pick Pipeline
// ^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL pick1
// openGripper function
// """"""""""""""""""""
// CALL_SUB_TUTORIAL open_gripper
// CALL_SUB_TUTORIAL pick2
// closedGripper function
// """"""""""""""""""""""
// CALL_SUB_TUTORIAL closed_gripper
// CALL_SUB_TUTORIAL pick3
//
// Place Pipeline
// ^^^^^^^^^^^^^^
// CALL_SUB_TUTORIAL place
// END_TUTORIAL

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "baxter_moveit_test");
 	ros::NodeHandle node_handle;  
 	ros::AsyncSpinner spinner(1);
 	spinner.start();
 	moveit::planning_interface::MoveGroup group("right_arm");
 	moveit::planning_interface::MoveGroup::Plan my_plan;

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
 	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
 	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
 	moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
 	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
 	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());



  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
 	ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
 	ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

 	geometry_msgs::Pose my_pose;
 	my_pose.position.x = 0.54666548495;
 	my_pose.position.y = -0.102305563037;
 	my_pose.position.z = 0.306209878016;
 	my_pose.orientation.x = -0.547922120529;
 	my_pose.orientation.y = -0.0239951476795;
 	my_pose.orientation.z = -0.0745942473519;
 	my_pose.orientation.w = 0.832851295841;

 	//group.setPoseTarget(my_pose);

 	//moveit::planning_interface::MoveGroup::Plan my_plan;
	//bool success = group.plan(my_plan);
	//std::vector<double> group_variable_values;
 	//group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
 	//group_variable_values[4] = -1.0;  
 	/*
 	std::vector<double> group_variable_values;
 	std::vector<double> my_angle = {-0.687578798801038, -1.2286859555710585, 1.7230128939734446, 1.4960121051119204, -0.3406034304607237, 1.6922049512515933, -2.6852473873167133};
 	group.setJointValueTarget(my_angle);
 	group.move();

 	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
 	group_variable_values[0] = 1.0;  
	group.setJointValueTarget(group_variable_values);
	group.move();
*/
	moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "torso";

  /* The id of the object is used to identify it. */
  collision_object.id = "kinect";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.3;
  primitive.dimensions[1] = 0.29;
  primitive.dimensions[2] = 0.06;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0;
  box_pose.position.y = 0.20;
  box_pose.position.z =  0.74;


  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);

  // Now when we plan a trajectory it will avoid the obstacle
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(my_pose);
  bool success = group.plan(my_plan);


  // Now, let's attach the collision object to the robot.
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);


  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");  
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);  
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);

 	return 0;
 }

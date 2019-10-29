#pragma once

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace uav_navigation
{

class NavigationPipeline
{
public:
	NavigationPipeline() {
		spinner_.start();
		setupMoveGroup();
		geometry_msgs::Pose target_pose1;
		target_pose1.orientation.w = 1.0;
		target_pose1.position.x = 0.0;
		target_pose1.position.y = 0.0;
		target_pose1.position.z = 1.5;
		if (plan(target_pose1)) {
			ROS_INFO("Planning %s.", plan(target_pose1) ? "sucessful" : "failed");
			move_group_interface_->execute(plan_);
		}
	}
	
	~NavigationPipeline() {
	}

	void setupMoveGroup() {
		ros::NodeHandle p_nh("~");
		p_nh.getParam("move_group", move_group_);
		p_nh.getParam("workspace/min_x", workspace_bounds_[0]);
		p_nh.getParam("workspace/min_y", workspace_bounds_[1]);
		p_nh.getParam("workspace/min_z", workspace_bounds_[2]);
		p_nh.getParam("workspace/max_x", workspace_bounds_[3]);
		p_nh.getParam("workspace/max_y", workspace_bounds_[4]);
		p_nh.getParam("workspace/max_z", workspace_bounds_[5]);
		p_nh.getParam("planning/max_plan_time", max_plan_time_);
		p_nh.getParam("planning/max_plan_attempts", max_plan_attempts_);
		p_nh.getParam("planning/max_vel_scale", max_vel_scale_);
		p_nh.getParam("planning/max_acc_scale", max_acc_scale_);
		using namespace moveit::planning_interface;
		move_group_interface_ = MoveGroupInterfacePtr(
			new MoveGroupInterface(move_group_));
		if (move_group_interface_) {
			move_group_interface_->setWorkspace(
				workspace_bounds_[0], workspace_bounds_[1], workspace_bounds_[2],
				workspace_bounds_[3], workspace_bounds_[4], workspace_bounds_[5]);
			move_group_interface_->setPlanningTime(max_plan_time_);	
			move_group_interface_->setNumPlanningAttempts(max_plan_attempts_);
			move_group_interface_->setMaxVelocityScalingFactor(max_vel_scale_);
			move_group_interface_->setMaxAccelerationScalingFactor(max_acc_scale_);
		} else {
			ROS_FATAL("No move group with the name '%s' found.", move_group_);
			nh_.shutdown();
		}
	}

	bool plan(const geometry_msgs::Pose& target_pose) {
		move_group_interface_->setStartStateToCurrentState();
		std::vector<double> target_joints(7, 0.0); // virtual joint 
		target_joints[0] = target_pose.position.x;
		target_joints[1] = target_pose.position.y;
		target_joints[2] = target_pose.position.z;
		target_joints[3] = target_pose.orientation.x;
		target_joints[4] = target_pose.orientation.y;
		target_joints[5] = target_pose.orientation.z;
		target_joints[6] = target_pose.orientation.w;
		move_group_interface_->setJointValueTarget(target_joints);
		auto ret = move_group_interface_->plan(plan_);
		if (ret == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
			return true;
		} else {
			if (ret == moveit_msgs::MoveItErrorCodes::PLANNING_FAILED)
				ROS_WARN("Planning failed.");
			else if (ret == moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION)
				ROS_WARN("Planning failed with error: Start state in collision.");
			else if (ret == moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS)
				ROS_WARN("Planning failed with error: Start state violates path constraints.");
			else if (ret == moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION)
				ROS_WARN("Planning failed with error: Goal state in collision.");
			else if (ret == moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS)
				ROS_WARN("Planning failed with error: Goal state violates path constraints.");
			else if (ret == moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED)
				ROS_WARN("Planning failed with error: Goal state violates goal constraints.");
			return false;
		}
	}

private:
	// ros
	ros::NodeHandle nh_;
	ros::AsyncSpinner spinner_ = ros::AsyncSpinner(1);

	// Moveit
	std::string move_group_;
	moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_; // move group interface
	moveit::planning_interface::MoveGroupInterface::Plan plan_; // plan container
	planning_scene::PlanningScenePtr planning_scene_interface_; // Planning scene

	// Planning config
	std::vector<double> workspace_bounds_ = std::vector<double>(6, 0.0);
	double max_plan_time_;
	int max_plan_attempts_;
	double max_vel_scale_;
	double max_acc_scale_;
};

}
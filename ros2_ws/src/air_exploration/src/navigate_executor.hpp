#pragma once
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/detail/navigate_to_pose__struct.hpp>
#include <nav2_msgs/msg/detail/speed_limit__struct.hpp>

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/Executor/ExecutionStatus.h>
#include <TstML/TSTNode.h>

class NavigateExecutor : public TstML::Executor::AbstractNodeExecutor
{
public:
	using DriveTo = nav2_msgs::action::NavigateToPose;
	using GoalHandleDriveTo = rclcpp_action::ClientGoalHandle<DriveTo>;

	NavigateExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context);
	~NavigateExecutor();

	TstML::Executor::ExecutionStatus start() override;
	TstML::Executor::ExecutionStatus pause() override;
	TstML::Executor::ExecutionStatus resume() override;
	TstML::Executor::ExecutionStatus stop() override;
	TstML::Executor::ExecutionStatus abort() override;

private:
        std::shared_ptr<rclcpp::Node> m_node;
	rclcpp::executors::MultiThreadedExecutor m_executor;
	std::thread m_executor_thread;
	rclcpp_action::Client<DriveTo>::SharedPtr navigate_client;
	GoalHandleDriveTo::SharedPtr m_goal_handle;
	rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;

	void goal_response_callback(std::shared_future<GoalHandleDriveTo::SharedPtr> future);
	void feedback_callback(GoalHandleDriveTo::SharedPtr goal_handle,const std::shared_ptr<const DriveTo::Feedback> feedback);
	void result_callback(const GoalHandleDriveTo::WrappedResult &result);
};

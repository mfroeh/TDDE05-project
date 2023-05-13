#include "navigate_executor.hpp"

#include <string>
#include <QPoint>

char const *NAVIGATE_EXECUTOR_NODE_NAME = "drive_to_node";

using namespace rclcpp;
using namespace rclcpp_action;

using DriveTo = nav2_msgs::action::NavigateToPose;
using GoalHandleDriveTo = rclcpp_action::ClientGoalHandle<DriveTo>;


NavigateExecutor::NavigateExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context) : TstML::Executor::AbstractNodeExecutor{tst_node, context}
{

	static int counter{};

	m_node = Node::make_shared(NAVIGATE_EXECUTOR_NODE_NAME + std::to_string(++counter));
	m_executor.add_node(m_node);
	m_executor_thread = std::thread([this]()
	{ m_executor.spin(); });

	navigate_client = rclcpp_action::create_client<DriveTo>(m_node, "navigate_to_pose");
}

NavigateExecutor::~NavigateExecutor()
{
	m_executor.cancel();
	m_executor_thread.join();
}

TstML::Executor::ExecutionStatus NavigateExecutor::start()
{
	using namespace std::placeholders;

	auto goal_msg = DriveTo::Goal();
	    
	QVariant p = node()->getParameter(TstML::TSTNode::ParameterType::Specific, "p");

	auto map{p.toMap()};
	double x{map["x"].toDouble()};
	double y{map["y"].toDouble()};
	double z{map["z"].toDouble()};
	geometry_msgs::msg::Point point{};
	point.x = x;
	point.y = y;
	point.z = z;
	goal_msg.pose.pose.position = point;
	goal_msg.pose.header.frame_id = "map";

	auto send_goal_options = rclcpp_action::Client<DriveTo>::SendGoalOptions();
	send_goal_options.goal_response_callback =
	std::bind(&NavigateExecutor::goal_response_callback, this, _1);
	send_goal_options.feedback_callback = std::bind(&NavigateExecutor::feedback_callback, this, _1, _2);
	send_goal_options.result_callback = std::bind(&NavigateExecutor::result_callback, this, _1);
	navigate_client->async_send_goal(goal_msg, send_goal_options);

	return TstML::Executor::ExecutionStatus::Started();
}

void NavigateExecutor::goal_response_callback(std::shared_future<GoalHandleDriveTo::SharedPtr> future)
{
	m_goal_handle = future.get();
	if (!m_goal_handle) 
	{
		executionFinished(TstML::Executor::ExecutionStatus::Aborted());
	        RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
	}
       	else
       	{
		RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server, waiting for result");
	}
}

void NavigateExecutor::feedback_callback(GoalHandleDriveTo::SharedPtr goal_handle,const std::shared_ptr<const DriveTo::Feedback> feedback)
{}

void NavigateExecutor::result_callback(const GoalHandleDriveTo::WrappedResult &result)
{
	switch (result.code)
       	{
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(m_node->get_logger(), "Goal was succeeded");
			executionFinished(TstML::Executor::ExecutionStatus::Finished());
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted: %d", (int)result.code);
			executionFinished(TstML::Executor::ExecutionStatus::Aborted());
			return;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(m_node->get_logger(), "Goal was canceled");
			executionFinished(TstML::Executor::ExecutionStatus::Aborted());
			return;
		default:
			RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
			executionFinished(TstML::Executor::ExecutionStatus::Aborted());
			return;
	}
}

TstML::Executor::ExecutionStatus NavigateExecutor::pause()
{
	return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus NavigateExecutor::resume()
{
	return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus NavigateExecutor::stop()
{
	navigate_client->async_cancel_goal(m_goal_handle);
	return TstML::Executor::ExecutionStatus::Finished();
}

TstML::Executor::ExecutionStatus NavigateExecutor::abort()
{
	navigate_client->async_cancel_goal(m_goal_handle);
	return TstML::Executor::ExecutionStatus::Aborted();
}

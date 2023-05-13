#include "undock_executor.hpp"

#include <string>

char const *UNDOCK_EXECUTOR_NODE_NAME = "undock_node";

using namespace rclcpp;
using namespace rclcpp_action;

using irobot_create_msgs::action::Undock;

UndockExecutor::UndockExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context) : TstML::Executor::AbstractNodeExecutor{tst_node, context}
{
    using namespace std::placeholders;

    static int counter{};

    node = Node::make_shared(UNDOCK_EXECUTOR_NODE_NAME + std::to_string(++counter));
    executor.add_node(node);
    executor_thread = std::thread([this]()
                                  { executor.spin(); });

    // Undock client
    undock_client = rclcpp_action::create_client<Undock>(node, "undock");
}

UndockExecutor::~UndockExecutor()
{
    executor.cancel();
    executor_thread.join();
}

TstML::Executor::ExecutionStatus UndockExecutor::start()
{
    using namespace std::placeholders;

    Undock::Goal goal_msg = Undock::Goal();

    rclcpp_action::Client<Undock>::SendGoalOptions send_goal_options{};
    send_goal_options.goal_response_callback = std::bind(&UndockExecutor::handle_undock_response, this, _1);
    send_goal_options.feedback_callback = std::bind(&UndockExecutor::handle_undock_feedback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&UndockExecutor::handle_undock_result, this, _1);
    undock_client->async_send_goal(goal_msg, send_goal_options);

    return TstML::Executor::ExecutionStatus::Started();
}

TstML::Executor::ExecutionStatus UndockExecutor::pause()
{
    return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus UndockExecutor::resume()
{
    return TstML::Executor::ExecutionStatus::Running();
}

TstML::Executor::ExecutionStatus UndockExecutor::stop()
{
    undock_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Finished();
}

TstML::Executor::ExecutionStatus UndockExecutor::abort()
{
    undock_client->async_cancel_goal(goal_handle);
    return TstML::Executor::ExecutionStatus::Aborted();
}

void UndockExecutor::handle_undock_response(std::shared_future<rclcpp_action::ClientGoalHandle<Undock>::SharedPtr> future)
{
    goal_handle = future.get();
    if (!goal_handle)
    {
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        RCLCPP_ERROR(node->get_logger(), "Undock: Goal was rejected, aborting goal...");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Undock: Goal accepted by server, waiting for result");
    }
}

void UndockExecutor::handle_undock_feedback(rclcpp_action::ClientGoalHandle<Undock>::SharedPtr,
                                            const std::shared_ptr<const Undock::Feedback>)
{
    RCLCPP_INFO(node->get_logger(), "Undock: Feedback received");
}

void UndockExecutor::handle_undock_result(rclcpp_action::ClientGoalHandle<Undock>::WrappedResult const &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node->get_logger(), "Undock: Goal was succeeded");
        executionFinished(TstML::Executor::ExecutionStatus::Finished());
        return;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node->get_logger(), "Undock: Goal was aborted: %d", static_cast<int>(result.code));
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node->get_logger(), "Undock: Goal was canceled");
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    default:
        RCLCPP_ERROR(node->get_logger(), "Unknown result code");
        executionFinished(TstML::Executor::ExecutionStatus::Aborted());
        return;
    }
}
#pragma once
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/Executor/ExecutionStatus.h>
#include <TstML/TSTNode.h>

#include <irobot_create_msgs/action/undock.hpp>

class UndockExecutor : public TstML::Executor::AbstractNodeExecutor
{
public:
    UndockExecutor(TstML::TSTNode const *tst_node, TstML::Executor::AbstractExecutionContext *context);
    ~UndockExecutor();

    TstML::Executor::ExecutionStatus start() override;
    TstML::Executor::ExecutionStatus pause() override;
    TstML::Executor::ExecutionStatus resume() override;
    TstML::Executor::ExecutionStatus stop() override;
    TstML::Executor::ExecutionStatus abort() override;

private:
    std::shared_ptr<rclcpp::Node> node{};
    rclcpp::executors::MultiThreadedExecutor executor{};
    std::thread executor_thread{};

    rclcpp_action::Client<irobot_create_msgs::action::Undock>::SharedPtr undock_client{};
    rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::SharedPtr goal_handle{};

    void handle_undock_response(std::shared_future<rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::SharedPtr> future);

    void handle_undock_feedback(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::SharedPtr, const std::shared_ptr<const irobot_create_msgs::action::Undock::Feedback>);

    void handle_undock_result(rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::Undock>::WrappedResult const &result);
};
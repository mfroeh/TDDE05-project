#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <QString>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>
#include <QFileInfo>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <TstML/Executor/DefaultNodeExecutor/Concurrent.h>
#include <TstML/Executor/DefaultNodeExecutor/Sequence.h>
#include <TstML/Executor/ExecutionStatus.h>
#include <TstML/Executor/Executor.h>
#include <TstML/Executor/NodeExecutorRegistry.h>
#include <TstML/TSTNode.h>
#include <TstML/TSTNodeModelsRegistry.h>
#include <rmw/qos_profiles.h>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <std_srvs/srv/empty.hpp>

#include "air_interfaces/srv/execute_tst.hpp"
#include "explore_executor.hpp"

using namespace rclcpp;
using namespace TstML;
using namespace TstML::Executor;

using air_interfaces::srv::ExecuteTst;
using std_srvs::srv::Empty;

char const *TST_EXECUTOR_NODE = "tst_executor";
char const *EXECUTE_TST_SERVICE_TOPIC = "execute_tst";

class TstExecutor : public Node
{
    using Self = TstExecutor;

public:
    TstExecutor()
        : Node{TST_EXECUTOR_NODE},
          tst_registry{std::make_shared<TSTNodeModelsRegistry>()},
          tst_executor_registry{std::make_shared<NodeExecutorRegistry>()},
          tst_executor{}
    {
        using namespace std::placeholders;

        // TODO: Change this to load our specific tsts
        tst_registry->loadDirectory(QFileInfo("./tst").absoluteFilePath());
        service = create_service<ExecuteTst>(EXECUTE_TST_SERVICE_TOPIC, std::bind(&Self::execute, this, _1, _2));

        CallbackGroup::SharedPtr group{create_callback_group(CallbackGroupType::Reentrant)};
        abort_service = create_service<std_srvs::srv::Empty>("abort", std::bind(&Self::abort, this, _1, _2), rmw_qos_profile_services_default, group);
        stop_service = create_service<std_srvs::srv::Empty>("stop", std::bind(&Self::stop, this, _1, _2), rmw_qos_profile_services_default, group);
        pause_service = create_service<std_srvs::srv::Empty>("pause", std::bind(&Self::pause, this, _1, _2), rmw_qos_profile_services_default, group);
        resume_service = create_service<std_srvs::srv::Empty>("resume", std::bind(&Self::resume, this, _1, _2), rmw_qos_profile_services_default, group);

        // Setup the executors
        tst_executor_registry->registerNodeExecutor<DefaultNodeExecutor::Sequence>(tst_registry->model("seq"));
        tst_executor_registry->registerNodeExecutor<DefaultNodeExecutor::Concurrent>(tst_registry->model("conc"));
        tst_executor_registry->registerNodeExecutor<ExploreExecutor>(tst_registry->model("explore"));

        // TODO: Register driver
        // tst_executor_registry->registerNodeExecutor<UndockExecutor>(
        //     tst_registry->model("undock"));
        // tst_executor_registry->registerNodeExecutor<DockExecutor>(
        //     tst_registry->model("dock"));
        // tst_executor_registry->registerNodeExecutor<DriveToExecutor>(
        //     tst_registry->model("drive-to"));
        // tst_executor_registry->registerNodeExecutor<RecordSemanticExecutor>(
        //     tst_registry->model("record-semantic"));
    }

private:
    Service<ExecuteTst>::SharedPtr service;
    Service<std_srvs::srv::Empty>::SharedPtr abort_service;
    Service<std_srvs::srv::Empty>::SharedPtr stop_service;
    Service<std_srvs::srv::Empty>::SharedPtr pause_service;
    Service<std_srvs::srv::Empty>::SharedPtr resume_service;

    std::shared_ptr<TSTNodeModelsRegistry> tst_registry;
    std::shared_ptr<NodeExecutorRegistry> tst_executor_registry;
    std::unique_ptr<Executor::Executor> tst_executor;

    void execute(std::shared_ptr<ExecuteTst::Request> const request, std::shared_ptr<ExecuteTst::Response> response)
    {
        std::string tst_filename{request->tst_file};

        std::string tst_json{request->tst};
        RCLCPP_INFO(get_logger(), "Received TST:\n%s", tst_json.c_str());

        TstML::TSTNode *tst_node{};
        if (tst_filename != "")
        {
            tst_node = TstML::TSTNode::load(QUrl::fromLocalFile(QString::fromStdString(tst_filename)), tst_registry.get());
            RCLCPP_INFO(get_logger(), "Loaded TSTNode from File");
        }
        else
        {
            QJsonDocument tst_document{QJsonDocument::fromJson(QString::fromStdString(tst_json).toUtf8())};
            RCLCPP_INFO(get_logger(), "Created QJsonDocument from json");
            tst_node = TSTNode::fromJson(tst_document.object(), tst_registry.get());
            RCLCPP_INFO(get_logger(), "Loaded TSTNode from QJsonDocument");
        }

        if (tst_node == nullptr)
            RCLCPP_ERROR(get_logger(), "TSTNode is null!");

        // Create an executor using the executors defined in tst_executor_registry
        tst_executor = std::make_unique<Executor::Executor>(tst_node, tst_executor_registry.get());

        // Start execution
        tst_executor->start();

        RCLCPP_INFO(get_logger(), "Before waitForFinished");
        // Block until the execution has finished
        ExecutionStatus status{tst_executor->waitForFinished()};
        RCLCPP_INFO(get_logger(), "After waitForFinished");

        // Cleanup
        tst_executor = nullptr;

        // Display the result of execution
        response->success = status == ExecutionStatus::Finished();
        if (response->success)
        {
            RCLCPP_INFO(get_logger(), "Execution successful");
        }
        else
        {
            std::string message = status.message().toStdString();
            response->error = message;
            RCLCPP_INFO(get_logger(), "Execution error '%s'", message.c_str());
        }
    }

    void abort(std::shared_ptr<Empty::Request> const, std::shared_ptr<Empty::Response>)
    {
        if (tst_executor != NULL)
            tst_executor->abort();
    }

    void stop(std::shared_ptr<Empty::Request> const, std::shared_ptr<Empty::Response>)
    {
        if (tst_executor != NULL)
            tst_executor->stop();
    }

    void pause(std::shared_ptr<Empty::Request> const, std::shared_ptr<Empty::Response>)
    {
        if (tst_executor != NULL)
            tst_executor->pause();
    }

    void resume(std::shared_ptr<Empty::Request> const, std::shared_ptr<Empty::Response>)
    {
        if (tst_executor != NULL)
            tst_executor->resume();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<TstExecutor>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
}
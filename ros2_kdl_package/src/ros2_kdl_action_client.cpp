#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/kdl_feedback.hpp"

class StartActionClient : public rclcpp::Node
{
public:
  using KdlFeedback = ros2_kdl_package::action::KdlFeedback;
  using GoalHandleStart = rclcpp_action::ClientGoalHandle<KdlFeedback>;

  StartActionClient() : Node("ros2_kdl_action_client")
  {
    client_ = rclcpp_action::create_client<KdlFeedback>(this, "kdl_feedback");
    send_goal();
  }

private:
  rclcpp_action::Client<KdlFeedback>::SharedPtr client_;

  void send_goal()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = KdlFeedback::Goal();
    goal_msg.start = true;

    auto send_goal_options = rclcpp_action::Client<KdlFeedback>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleStart> handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for feedback...");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandleStart::SharedPtr, const std::shared_ptr<const KdlFeedback::Feedback> feedback) {
        RCLCPP_INFO(
          get_logger(),
          "Error Feedback: [%f, %f, %f, %f, %f, %f, %f]\n",
          feedback->current_error[0],
          feedback->current_error[1],
          feedback->current_error[2],
          feedback->current_error[3],
          feedback->current_error[4],
          feedback->current_error[5],
          feedback->current_error[6]
        );
      };

    send_goal_options.result_callback =
      [this](const GoalHandleStart::WrappedResult &result) {
        RCLCPP_INFO(get_logger(), "Result received: completed=%d", result.result->completed);
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StartActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

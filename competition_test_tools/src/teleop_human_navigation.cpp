#include "teleop_base.hpp"
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>

namespace MsgHn 
{
  constexpr const char* TEST_MSG = "This is a test message.";
}

class HumanNavigationScenario : public TeleopScenarioInterface
{
public:
  HumanNavigationScenario() = default;

  void setup_ros(const rclcpp::Node::SharedPtr& node) override
  {
    node_ = node;
    pub_msg_hn_   = node_->create_publisher   <human_navigation_msgs::msg::HumanNaviMsg>        ("/human_navigation/message/to_moderator", 10);
    pub_guid_msg_ = node_->create_publisher   <human_navigation_msgs::msg::HumanNaviGuidanceMsg>("/human_navigation/message/guidance_message", 10);
    sub_msg_hn_   = node_->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>        ("/human_navigation/message/to_robot", 10, std::bind(&HumanNavigationScenario::recv_message, this, std::placeholders::_1));
  }

  void show_help(WINDOW* win_header, int start_row) override
  {
    if (!win_header) { return; }

    mvwprintw(win_header, start_row + 0, 2, " 0: Send Msg: %s", MsgCm::I_AM_READY);
    mvwprintw(win_header, start_row + 1, 2, " 1: Send Msg: %s", MsgHn::TEST_MSG);
    mvwprintw(win_header, start_row + 2, 2, " 9: Send Msg: %s", MsgCm::GIVE_UP);
  }

  bool handle_numeric_key(int key) override
  {
    if (!pub_msg_hn_) { return false; }

    switch (key)
    {
      case '0': send_message (MsgCm::I_AM_READY);      return true;
      case '1': send_guid_msg(MsgHn::TEST_MSG, "All"); return true;
      case '9': send_message (MsgCm::GIVE_UP);         return true;
      default:                                         return false;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher   <human_navigation_msgs::msg::HumanNaviMsg>        ::SharedPtr pub_msg_hn_;
  rclcpp::Publisher   <human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pub_guid_msg_;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviMsg>        ::SharedPtr sub_msg_hn_;

  void send_message(const std::string& message)
  {
    RCLCPP_INFO(node_->get_logger(), "HN:Send msg:%s", message.c_str());
    human_navigation_msgs::msg::HumanNaviMsg human_navigation_msg;
    human_navigation_msg.message = message;
    pub_msg_hn_->publish(human_navigation_msg);
  }

  void send_guid_msg(const std::string &message, const std::string display_type)
  {
    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidance_msg;
    guidance_msg.message = message;
    guidance_msg.display_type = display_type;
    pub_guid_msg_->publish(guidance_msg);

    RCLCPP_INFO(node_->get_logger(), "HN:Send guid msg: %s : %s", guidance_msg.message.c_str(), guidance_msg.display_type.c_str());
  }

  void recv_message(const human_navigation_msgs::msg::HumanNaviMsg::SharedPtr message)
  {
    RCLCPP_INFO(node_->get_logger(),"HN:Subscribe:%s, %s", message->message.c_str(), message->detail.c_str());
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto scenario = std::make_unique<HumanNavigationScenario>();
  CompetitionTeleopKey teleop_key(std::move(scenario), "teleop_human_navigation");

  return teleop_key.run();
}

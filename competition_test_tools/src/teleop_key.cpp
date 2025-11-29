#include <memory>
#include <cmath>
#include <termios.h>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <handyman_msgs/msg/handyman_msg.hpp>
#include <interactive_cleanup_msgs/msg/interactive_cleanup_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>
#include <ncurses.h> 

class CompetitionTeleopKey
{
private:
  const int WINDOW_HEADER_HEIGHT = 10;

  const std::string ARM_LIFT_JOINT_NAME   = "arm_lift_joint";
  const std::string ARM_FLEX_JOINT_NAME   = "arm_flex_joint";
  const std::string ARM_ROLL_JOINT_NAME   = "arm_roll_joint";
  const std::string WRIST_FLEX_JOINT_NAME = "wrist_flex_joint";
  const std::string WRIST_ROLL_JOINT_NAME = "wrist_roll_joint";

  // Message Common
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_I_AM_READY       = "I_am_ready";
  const std::string MSG_OBJECT_GRASPED   = "Object_grasped";
  const std::string MSG_TASK_FINISHED    = "Task_finished";
  const std::string MSG_GIVE_UP          = "Give_up";

  // Handyman 
  const std::string MSG_ENVIRONMENT    = "Environment";

  const std::string MSG_ROOM_REACHED   = "Room_reached";
  const std::string MSG_DOES_NOT_EXIST = "Does_not_exist";

  // Interactive Cleanup
  const std::string MSG_IS_THIS_CORRECT = "Is_this_correct?";
  const std::string MSG_POINT_IT_AGAIN  = "Point_it_again";

  // Human Navigation
  const std::string MSG_TEST_MSG = "This is a test message.";

public:
  CompetitionTeleopKey();

  void message_callback(const std_msgs::msg::String::SharedPtr message);
  void message_callback_hm(const handyman_msgs::msg::HandymanMsg::SharedPtr message);
  void message_callback_ic(const interactive_cleanup_msgs::msg::InteractiveCleanupMsg::SharedPtr message);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void send_message(const std::string &message);
  void send_message_hm(const std::string &message);
  void move_base_twist(const double linear_x, const double linear_y, const double angular_z);
  void move_base_joint_trajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec);
  void operate_arm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec);
  void operate_arm(const std::string &name, const double position, const double duration_sec);
  void operate_arm_flex(const double arm_flex_pos, const double wrist_flex_pos);
  double get_duration_rot(const double next_pos, const double current_pos);
  void operate_hand(const bool is_hand_open);
  void send_suction_goal(const bool &sution_on);

  void show_help();

  void display_message_in_window(const std::string& text);
  void update_window_layout();
  void init_window();
  void resize_window();
  void shutdown_window();

  int run();
  
private:
  // Window settings
  WINDOW* win_header_ = nullptr;
  static std::atomic<bool> need_redraw_window_; // set from existing SIGWINCH handler
  int header_height_ = WINDOW_HEADER_HEIGHT;

  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double arm_roll_joint_pos_;
  double wrist_flex_joint_pos_;
  double wrist_roll_joint_pos_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr pub_msg_hm_;
//  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_ic_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr sub_msg_hm_;
//  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_ic_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
};

std::atomic<bool> CompetitionTeleopKey::need_redraw_window_{false};

CompetitionTeleopKey::CompetitionTeleopKey()
{
  arm_lift_joint_pos1_   = 0.0;
  arm_lift_joint_pos2_   = 0.0;
  arm_flex_joint_pos_    = 0.0;
  arm_roll_joint_pos_    = 0.0;
  wrist_flex_joint_pos_  = 0.0;
  wrist_roll_joint_pos_  = 0.0;
  
  node_ = rclcpp::Node::make_shared("hsr_teleop_key");
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  pub_msg_            = node_->create_publisher<std_msgs::msg::String>                          ("/hsrb/message/to_human", 10);
  pub_msg_hm_         = node_->create_publisher<handyman_msgs::msg::HandymanMsg>                     ("/handyman/message/to_moderator", 10);
//  pub_msg_ic_         = node_->create_publisher<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/interactive_cleanup/message/to_moderator", 10);
//  pub_msg_hn_         = node_->create_publisher<human_navigation::msg::HumanNaviMsg>            ("/human_navigation/message/to_moderator", 10);
//  pub_msg_hn_guid_    = node_->create_publisher<human_navigation::msg::HumanNaviGuidanceMsg>    ("/human_navigation/message/guidance_message", 10);

  pub_base_twist_         = node_->create_publisher<geometry_msgs::msg::Twist>            ("/hsrb/command_velocity", 10);
  pub_base_trajectory_    = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/omni_base_controller/command", 10);
  pub_arm_trajectory_     = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/gripper_controller/command", 10);

  sub_msg_         = node_->create_subscription<std_msgs::msg::String>                              ("/hsrb/message/to_robot",                10, std::bind(&CompetitionTeleopKey::message_callback,    this, std::placeholders::_1));
  sub_msg_hm_      = node_->create_subscription<handyman_msgs::msg::HandymanMsg>                     ("/handyman/message/to_robot",            10, std::bind(&CompetitionTeleopKey::message_callback_hm,    this, std::placeholders::_1));
//  sub_msg_ic_      = node_->create_subscription<interactive_cleanup_msgs::msg::InteractiveCleanupMsg>("/interactive_cleanup/message/to_robot", 10, std::bind(&CompetitionTeleopKey::message_callback,    this, std::placeholders::_1));

  sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>("/hsrb/joint_states",     10, std::bind(&CompetitionTeleopKey::joint_state_callback, this, std::placeholders::_1));
}

void CompetitionTeleopKey::message_callback(const std_msgs::msg::String::SharedPtr message)
{
  RCLCPP_INFO(node_->get_logger(), "Subscribe message: %s", message->data.c_str());
}

void CompetitionTeleopKey::message_callback_hm(const handyman_msgs::msg::HandymanMsg::SharedPtr message)
{
  // if(message->message.c_str()==MSG_ARE_YOU_READY && is_received_are_you_ready_){ return; }
  // if(message->message.c_str()==MSG_ENVIRONMENT   && is_received_environment_)  { return; }

  RCLCPP_INFO(node_->get_logger(), "HM:Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

  // if(message->message.c_str()==MSG_ARE_YOU_READY){ is_received_are_you_ready_ = true; }
  // if(message->message.c_str()==MSG_ENVIRONMENT  ){ is_received_environment_   = true; }

  // if(message->message.c_str()==MSG_TASK_SUCCEEDED || message->message.c_str()==MSG_TASK_FAILED || message->message.c_str()==MSG_MISSION_COMPLETE)
  // {
  //   is_received_are_you_ready_ = false;
  //   is_received_environment_   = false;
  // }
}

void CompetitionTeleopKey::message_callback_ic(const interactive_cleanup_msgs::msg::InteractiveCleanupMsg::SharedPtr message)
{
  // if(message->message.c_str()==MSG_ARE_YOU_READY && is_received_are_you_ready_){ return; }

  RCLCPP_INFO(node_->get_logger(), "IC:Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

  // if(message->message.c_str()==MSG_ARE_YOU_READY){ is_received_are_you_ready_ = true; }

  // if(message->message.c_str()==MSG_TASK_SUCCEEDED || message->message.c_str()==MSG_TASK_FAILED || message->message.c_str()==MSG_MISSION_COMPLETE)
  // {
  //   is_received_are_you_ready_ = false;
  // }
}

void CompetitionTeleopKey::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  for(size_t i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i] == ARM_LIFT_JOINT_NAME)
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == ARM_FLEX_JOINT_NAME)
    {
      arm_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == ARM_ROLL_JOINT_NAME)
    {
      arm_roll_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == WRIST_FLEX_JOINT_NAME)
    {
      wrist_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == WRIST_ROLL_JOINT_NAME)
    {
      wrist_roll_joint_pos_ = joint_state->position[i];
    }
  }
}

void CompetitionTeleopKey::send_message(const std::string &message)
{
  RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

  std_msgs::msg::String string_msg;
  string_msg.data = message;
  pub_msg_->publish(string_msg);
}

void CompetitionTeleopKey::send_message_hm(const std::string &message)
{
  RCLCPP_INFO(node_->get_logger(), "HM:Send message:%s", message.c_str());

  handyman_msgs::msg::HandymanMsg handyman_msg;
  handyman_msg.message = message;
  pub_msg_hm_->publish(handyman_msg);
}

void CompetitionTeleopKey::move_base_twist(const double linear_x, const double linear_y, const double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void CompetitionTeleopKey::move_base_joint_trajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec)
{
  if(!tf_buffer_->canTransform("/odom", "/base_footprint", tf2::TimePointZero))
  {
    return;
  }

  geometry_msgs::msg::PointStamped basefootprint_2_target;
  geometry_msgs::msg::PointStamped odom_2_target;
  basefootprint_2_target.header.frame_id = "/base_footprint";
  basefootprint_2_target.header.stamp = rclcpp::Time(0);
  basefootprint_2_target.point.x = linear_x;
  basefootprint_2_target.point.y = linear_y;
  odom_2_target = tf_buffer_->transform(basefootprint_2_target, "odom", tf2::Duration(0)); 

  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(node_->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(transform.transform.rotation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("odom_x");
  joint_trajectory.joint_names.push_back("odom_y");
  joint_trajectory.joint_names.push_back("odom_t");

  trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
  omni_joint_point.positions = {odom_2_target.point.x, odom_2_target.point.y, yaw + theta};
  omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  joint_trajectory.points.push_back(omni_joint_point);
  pub_base_trajectory_->publish(joint_trajectory);
}

void CompetitionTeleopKey::operate_arm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back(ARM_LIFT_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_ROLL_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_ROLL_JOINT_NAME);

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos};

  arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_->publish(joint_trajectory);
}

void CompetitionTeleopKey::operate_arm(const std::string &name, const double position, const double duration_sec)
{
  double arm_lift = arm_lift_joint_pos1_;
  double arm_flex = arm_flex_joint_pos_;
  double arm_roll = arm_roll_joint_pos_;
  double wrist_flex = wrist_flex_joint_pos_;
  double wrist_roll = wrist_roll_joint_pos_;

  if     (name == ARM_LIFT_JOINT_NAME)  { arm_lift   = position; }
  else if(name == ARM_FLEX_JOINT_NAME)  { arm_flex   = position; }
  else if(name == ARM_ROLL_JOINT_NAME)  { arm_roll   = position; }
  else if(name == WRIST_FLEX_JOINT_NAME){ wrist_flex = position; }
  else if(name == WRIST_ROLL_JOINT_NAME){ wrist_roll = position; }

  this->operate_arm(arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll, duration_sec);
}

void CompetitionTeleopKey::operate_arm_flex(const double arm_flex_pos, const double wrist_flex_pos)
{
  double duration = std::max(this->get_duration_rot(arm_flex_pos, arm_flex_joint_pos_), this->get_duration_rot(wrist_flex_pos, wrist_flex_joint_pos_));

  this->operate_arm(arm_lift_joint_pos1_, arm_flex_pos, arm_roll_joint_pos_, wrist_flex_pos, wrist_roll_joint_pos_, duration);
}

double CompetitionTeleopKey::get_duration_rot(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

void CompetitionTeleopKey::operate_hand(const bool is_hand_open)
{
  std::vector<std::string> joint_names {"hand_motor_joint"};
  std::vector<double> positions;

  if(is_hand_open)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Grasp");
    positions.push_back(-0.105);
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Open hand");
    positions.push_back(+1.239);
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(2);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);
  pub_gripper_trajectory_->publish(joint_trajectory);
}
  
void CompetitionTeleopKey::show_help()
{
  if (!win_header_){ return; }
  wclear(win_header_);
  box(win_header_, 0, 0);
  mvwprintw(win_header_, 1, 2, "arrow keys : Move");
  mvwprintw(win_header_, 2, 2, "Space: Stop");
  mvwprintw(win_header_, 3, 2, "q/z : Increase/Decrease Moving Speed");
  mvwprintw(win_header_, 4, 2, "y/h/n: Up/Stop/Down Torso");
  mvwprintw(win_header_, 5, 2, "a/b/c/d: Rotate Arm Vert/Up/Horiz/Down");
  mvwprintw(win_header_, 6, 2, "g : Open/Close Gripper");
  mvwprintw(win_header_, 7, 2, "  0: Send Msg: %s", MSG_I_AM_READY.c_str());
  mvwprintw(win_header_, 8, 2, "  1: Send Msg: %s", MSG_ROOM_REACHED.c_str());
  mvwprintw(win_header_, 9, 2, "  2: Send Msg: %s", MSG_OBJECT_GRASPED.c_str());
  mvwprintw(win_header_,10, 2, "  3: Send Msg: %s", MSG_TASK_FINISHED.c_str());
  mvwprintw(win_header_,11, 2, "  6: Send Msg: %s", MSG_DOES_NOT_EXIST.c_str());
  mvwprintw(win_header_,12, 2, "  9: Send Msg: %s", MSG_GIVE_UP.c_str());
  wrefresh(win_header_);
  move(header_height_, 0);
  refresh();
}

void CompetitionTeleopKey::display_message_in_window(const std::string& text)
{
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  if (rows <= WINDOW_HEADER_HEIGHT + 6) { return; }
  resize_window();

  mvprintw(WINDOW_HEADER_HEIGHT + 2, 0, "%s", text.c_str());
  move(WINDOW_HEADER_HEIGHT + 3, 0);

  refresh();
}

void CompetitionTeleopKey::update_window_layout()
{
  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  if (rows <= 2) { return; } // terminal unavailable
  if (rows <= header_height_ + 2) { header_height_ = rows - 1; }
  else                            { header_height_ = WINDOW_HEADER_HEIGHT + 2; }

  if (win_header_) 
  {
    wresize(win_header_, header_height_, cols);
    mvwin(win_header_, 0, 0);
    werase(stdscr);
  }
  else 
  {
    win_header_ = newwin(header_height_, cols, 0, 0);
    if (!win_header_) { return; }
    wrefresh(win_header_);
    move(header_height_, 0);
  }

  refresh();

  std::printf("\033[%d;%dr", header_height_ + 1, rows); // limit scrolling to [header+1 .. rows]
  std::fflush(stdout);
}

void CompetitionTeleopKey::init_window() 
{
  setlocale(LC_ALL, "");
  initscr();
  cbreak();              // Disable line buffering
  noecho();              // Don't echo input characters
  nodelay(stdscr, TRUE); // Make getch() non-blocking
  keypad(stdscr, TRUE);  // Enable special keys (arrow keys, etc.)
  refresh();
  update_window_layout();
}

void CompetitionTeleopKey::resize_window() 
{
  update_window_layout();
  show_help();
}

void CompetitionTeleopKey::shutdown_window() 
{
  std::printf("\033[r");
  std::fflush(stdout);

  if (win_header_) { delwin(win_header_); win_header_ = nullptr; }
  endwin();
}


int CompetitionTeleopKey::run()
{
  try
  {
    init_window();
    show_help();

    signal(SIGWINCH, [](int){ need_redraw_window_.store(true, std::memory_order_relaxed); });

    auto logger = node_->get_logger();

    rclcpp::Rate loop_rate(50);

    const float linear_coef  = 0.2f;
    const float angular_coef = 0.5f;

    float move_speed = 1.0f;
    bool is_hand_open = false;

    while (rclcpp::ok())
    {
      if (need_redraw_window_.exchange(false)) { resize_window(); }

      int c = getch();
      if(c != ERR)  // Key was pressed
      {
        // Check for multi-byte characters (e.g., full-width input), But allow ncurses special keys.
        if(c > 127 && (c < KEY_MIN || c > KEY_MAX))
        {
          display_message_in_window("Please use half-width input mode!");
          continue;
        }

        switch(c)
        {
          case KEY_UP:
          {
            RCLCPP_DEBUG(logger, "Go Forward");
            move_base_twist(+linear_coef*move_speed, 0.0, 0.0);
            break;
          }
          case KEY_DOWN:
          {
            RCLCPP_DEBUG(logger, "Go Backward");
            move_base_twist(-linear_coef*move_speed, 0.0, 0.0);
            break;
          }
          case KEY_RIGHT:
          {
            RCLCPP_DEBUG(logger, "Go Right");
            move_base_twist(0.0, 0.0, -angular_coef*move_speed);
            break;
          }
          case KEY_LEFT:
          {
            RCLCPP_DEBUG(logger, "Go Left");
            move_base_twist(0.0, 0.0, +angular_coef*move_speed);
            break;
          }
          case ' ':
          {
            RCLCPP_DEBUG(logger, "Stop");
            move_base_twist(0.0, 0.0, 0.0);
            break;
          }
          case 'q':
          {
            RCLCPP_DEBUG(logger, "Move Speed Up");
            move_speed *= 2;
            if(move_speed > 2  ){ move_speed=2; }
            break;
          }
          case 'z':
          {
            RCLCPP_DEBUG(logger, "Move Speed Down");
            move_speed /= 2;
            if(move_speed < 0.125){ move_speed=0.125; }
            break;
          }
          case 'y':
          {
            RCLCPP_DEBUG(logger, "Up Torso");
            operate_arm(ARM_LIFT_JOINT_NAME, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
            break;
          }
          case 'h':
          {
            RCLCPP_DEBUG(logger, "Stop Torso");
            operate_arm(ARM_LIFT_JOINT_NAME, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
            break;
          }
          case 'n':
          {
            RCLCPP_DEBUG(logger, "Down Torso");
            operate_arm(ARM_LIFT_JOINT_NAME, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
            break;
          }
          case 'a':
          {
            RCLCPP_DEBUG(logger, "Rotate Arm - Vertical");
            operate_arm_flex(0.0, -1.57);
            break;
          }
          case 'b':
          {
            RCLCPP_DEBUG(logger, "Rotate Arm - Upward");
            operate_arm_flex(-0.785, -0.785);
            break;
          }
          case 'c':
          {
            RCLCPP_DEBUG(logger, "Rotate Arm - Horizontal");
            operate_arm_flex(-1.57, 0.0);
            break;
          }
          case 'd':
          {
            RCLCPP_DEBUG(logger, "Rotate Arm - Downward");
            operate_arm_flex(-2.2, 0.35);
            break;
          }
          case 'g':
          {
            operate_hand(is_hand_open);

            is_hand_open = !is_hand_open;
            break;
          }
          case '0': { send_message_hm(MSG_I_AM_READY);     break; }
          case '1': { send_message_hm(MSG_ROOM_REACHED);   break;}
          case '2': { send_message_hm(MSG_OBJECT_GRASPED); break; }
          case '3': { send_message_hm(MSG_TASK_FINISHED);  break;}
          case '6': { send_message_hm(MSG_DOES_NOT_EXIST); break; }
          case '9': { send_message_hm(MSG_GIVE_UP);        break;}
        }
      }

      rclcpp::spin_some(node_);

      loop_rate.sleep();
    }
  }
  catch(...)
  {
    puts("An exception occurred!");
  }

  shutdown_window();
  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  CompetitionTeleopKey teleop_key;
  return teleop_key.run();
}


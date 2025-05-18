#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

class JoyToExtendedPosition
{
public:
  JoyToExtendedPosition(ros::NodeHandle& nh) : nh_(nh), initialized_(false)
  {
    nh_.param("axis_indices", axis_indices_, std::vector<int>{1, 4});
    nh_.param("scale", scale_, 300.0);
    nh_.param("motor_count", motor_count_, 2);
    nh_.param("control_freq", control_freq_, 50.0);

    if (axis_indices_.size() != motor_count_)
      ROS_FATAL("axis_indicesとmotor_countのサイズ不一致");

    goal_positions_.resize(motor_count_, 0.0);
    input_values_.resize(motor_count_, 0.0);

    // --- Subscribers ---
    sub_joy_ = nh_.subscribe("/joy", 10, &JoyToExtendedPosition::joyCB, this);
    sub_init_ = nh_.subscribe("/sensor/motor/output/position", 1,
                              &JoyToExtendedPosition::initPositionCB, this);

    // --- Publisher ---
    pub_goal_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    // A button
    pub_button_ = nh_.advertise<std_msgs::String>("/joy/button_pressed", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_freq_),
                             &JoyToExtendedPosition::timerCB, this);
  }

private:
  void initPositionCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    if (msg->data.size() < static_cast<size_t>(motor_count_))
    {
      ROS_WARN("初期位置取得失敗: 要素数 %zu", msg->data.size());
      return;
    }

    for (int i = 0; i < motor_count_; ++i)
      goal_positions_[i] = static_cast<double>(msg->data[i]);

    initialized_ = true;
    sub_init_.shutdown();  // 初期化が完了したら購読を解除
    ROS_INFO("初期位置を受信しました。制御開始。");
  }

  void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
  {
    for (int i = 0; i < motor_count_; ++i)
    {
      int axis = axis_indices_[i];
      if (msg->axes.size() > axis)
        input_values_[i] = msg->axes[axis];
    }

    // ボタン data[2] 押下検出（エッジ検出）
    if (msg->buttons.size() > 2)
    {
      bool current_state = msg->buttons[2];

      if (current_state && !last_button_state_)
      {
        std_msgs::String out;
        out.data = "Button 2 pressed!";
        pub_button_.publish(out);
      }

      last_button_state_ = current_state;
    }
  }

  void timerCB(const ros::TimerEvent&)
  {
    if (!initialized_) return;

    for (int i = 0; i < motor_count_; ++i)
    {
      double vel = input_values_[i] * scale_;
      double delta = vel / control_freq_;
      goal_positions_[i] += delta;
    }

    std_msgs::Int32MultiArray msg;
    msg.data.resize(motor_count_);
    for (int i = 0; i < motor_count_; ++i)
      msg.data[i] = static_cast<int32_t>(goal_positions_[i]);

    pub_goal_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_joy_, sub_init_;
  ros::Publisher pub_goal_;
  ros::Timer timer_;

  std::vector<int> axis_indices_;
  std::vector<double> input_values_;
  std::vector<double> goal_positions_;

  int motor_count_;
  double scale_;
  double control_freq_;
  bool initialized_;

  // A button
  ros::Publisher pub_button_;
  bool last_button_state_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_to_extended_position");
  ros::NodeHandle nh("~");
  JoyToExtendedPosition node(nh);
  ros::spin();
  return 0;
}

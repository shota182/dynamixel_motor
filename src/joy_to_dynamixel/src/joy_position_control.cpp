// joyでpositionベースの制御

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

class JoyMirrorPosition
{
public:
  JoyMirrorPosition(ros::NodeHandle& nh) : nh_(nh)
  {
    // ---- Parameters -------------------------------------------------------
    nh_.param("axis_index", axis_index_, 0);          // 1 本だけ
    nh_.param("scale",       scale_,      300.0);     // [counts / full deflection]
    nh_.param("control_freq", control_freq_, 500.0);   // [Hz]

    motor_count_ = 2;                                 // 固定
    nh_.param("motor_signs", signs_, std::vector<int>{+1, +1});
    if (static_cast<int>(signs_.size()) != motor_count_)
      ROS_FATAL("motor_signs の要素数は %d である必要があります", motor_count_);

    input_value_ = 0.0;

    // ---- ROS I/O ----------------------------------------------------------
    sub_joy_  = nh_.subscribe("/joy", 10, &JoyMirrorPosition::joyCB, this);
    sub_init_ = nh_.subscribe("/sensor/motor/output/position", 1, &JoyMirrorPosition::initPositionCB, this);
    pub_goal_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    // 任意：ボタン 2 エッジ検出
    pub_button_ = nh_.advertise<std_msgs::String>("/joy/button_pressed", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_freq_),
                             &JoyMirrorPosition::timerCB, this);
  }

private:
  // ---- コールバック --------------------------------------------------------
  void initPositionCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    latest_position_ = *msg;  // 最新の位置を保存
    if(!initialized_){
        pub_position.data = latest_position_.data;  // 初期位置を設定
        initialized_ = true;
    }
    sub_init_.shutdown();
    ROS_INFO("初期位置を受信しました。制御開始。");
  }

  void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if (msg->axes.size() > axis_index_)
        input_value_ = msg->axes[axis_index_];

    // scaleの増減処理
    if (msg->axes.size() > 5) {
        if (msg->axes[5] == 1) {
            scale_ += 10.0; // scaleを増加
            ROS_INFO("Scale increased: %f", scale_);
        } else if (msg->axes[5] == -1) {
            scale_ -= 10.0; // scaleを減少
            ROS_INFO("Scale decreased: %f", scale_);
        }
    }

    // ID4のモータを動かす処理
    if (msg->axes.size() > 4 && std::abs(msg->axes[3]) > 0.01) {
        double vel = msg->axes[3] * scale_; // counts/s
        double step = vel / control_freq_;  // counts per cycle

        pub_position.data[3] += static_cast<int>(signs_[1] * step); // ID4のモータを動かす
        ROS_INFO("Motor ID4 moved: %d", pub_position.data[3]);
    }

    // ボタン 2（index=2）エッジ検出（任意）
    if (msg->buttons.size() > 2) {
        bool cur = msg->buttons[2];
        if (cur && !last_button_state_) {
            std_msgs::String out;
            out.data = "Button 2 pressed!";
            pub_button_.publish(out);
        }
        last_button_state_ = cur;
    }
  }

  void timerCB(const ros::TimerEvent&)
  {
    if (!initialized_) return;

    double vel  = input_value_ * scale_;     // counts/s
    double step = vel / control_freq_;       // counts per cycle

    if(std::abs(input_value_) > 0.01) {
      pub_position.data[0] += static_cast<int>(signs_[0] * step);
      pub_position.data[3] += static_cast<int>(signs_[1] * step);
    }

    pub_goal_.publish(pub_position);
  }

  // ---- メンバ変数 ----------------------------------------------------------
  ros::NodeHandle nh_;
  ros::Subscriber sub_joy_, sub_init_;
  ros::Publisher pub_goal_, pub_button_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray pub_position;
  std_msgs::Int32MultiArray latest_position_;

  // params / state
  int    axis_index_;
  int    motor_count_;
  double scale_;
  double control_freq_;
  std::vector<int>    signs_;          // {+1,-1}
  double input_value_;
  bool   initialized_ = false;
  bool   last_button_state_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_position_control");
  ros::NodeHandle nh("~");
  JoyMirrorPosition node(nh);
  ros::spin();
  return 0;
}

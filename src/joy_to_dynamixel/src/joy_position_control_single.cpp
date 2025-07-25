// joy_position_control_single.cpp
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>

class JoySingleMotorPosition
{
public:
  explicit JoySingleMotorPosition(ros::NodeHandle& nh) : nh_(nh)
  {
    /* --- parameters ---------------------------------------------------- */
    nh_.param("motor_id",       motor_id_,      4);        // 操作対象モータの ID (配列添字)
    nh_.param("axis_index",     axis_index_,    0);        // Joy の軸番号
    nh_.param("scale",          scale_,       50.0);      // [counts / full-deflection]
    nh_.param("control_freq",   ctrl_freq_,   500.0);      // [Hz]
    nh_.param("motor_sign",     motor_sign_,      1);      // 向き反転が必要な場合 −1

    if (motor_id_ < 0) {
      ROS_FATAL("motor_id は 0 以上で指定してください"); 
      ros::shutdown();
    }

    /* --- ROS I/O ------------------------------------------------------- */
    sub_joy_  = nh_.subscribe("/joy", 10, &JoySingleMotorPosition::joyCB, this);
    sub_init_ = nh_.subscribe("/sensor/motor/output/position", 1,
                              &JoySingleMotorPosition::initCB, this);
    pub_goal_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    timer_ = nh_.createTimer(ros::Duration(1.0 / ctrl_freq_),
                             &JoySingleMotorPosition::timerCB, this);
  }

private:
  /* --- 初期位置を 1 回だけ取得 --------------------------------------- */
  void initCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    pub_pos_ = *msg;        // 受け取った全モータの位置を保持
    initialized_ = true;
    sub_init_.shutdown();   // 初期位置がわかったら購読解除
    ROS_INFO_STREAM("初期位置を取得．モータ " << motor_id_
                    << " を Joy 軸 " << axis_index_ << " で操作します");
  }

  /* --- Joy コールバック ---------------------------------------------- */
  void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if (msg->axes.size() <= axis_index_) return;
    input_ = msg->axes[axis_index_];          // -1.0 ～ 1.0
  }

  /* --- 周期処理 ------------------------------------------------------- */
  void timerCB(const ros::TimerEvent&)
  {
    if (!initialized_) return;

    const double vel  = input_ * scale_;           // [counts/s]
    const double step = vel / ctrl_freq_;          // [counts / cycle]

    if (std::abs(input_) > 0.01) {                 // デッドゾーン
      if (motor_id_ >= static_cast<int>(pub_pos_.data.size())) {
        ROS_WARN_THROTTLE(1.0, "motor_id が position 配列の範囲外です");
        return;
      }
      pub_pos_.data[motor_id_] += static_cast<int>(motor_sign_ * step);
    }

    pub_goal_.publish(pub_pos_);
  }

  /* --- 変数 ----------------------------------------------------------- */
  ros::NodeHandle nh_;
  ros::Subscriber sub_joy_, sub_init_;
  ros::Publisher  pub_goal_;
  ros::Timer      timer_;

  std_msgs::Int32MultiArray pub_pos_;

  int    motor_id_;
  int    axis_index_;
  int    motor_sign_;
  double scale_;
  double ctrl_freq_;
  double input_        = 0.0;
  bool   initialized_  = false;
};

/* --- main ------------------------------------------------------------- */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_position_control_single");
  ros::NodeHandle nh("~");
  JoySingleMotorPosition node(nh);
  ros::spin();
  return 0;
}

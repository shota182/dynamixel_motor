// joyでスレーブ側の制御_
// スレーブの制御が反映されていないので使わないこと
// joy_slave_control_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <deque>
#include <vector>
#include <algorithm>

class JoySlaveControl
{
public:
  explicit JoySlaveControl(ros::NodeHandle& nh) : nh_(nh)
  {
    // ---------- parameters ----------
    nh_.param("axis_index",         axis_index_,         0);
    nh_.param("axis_deadzone",      axis_deadzone_,   0.05);       // │ジョイスティック用
    nh_.param("scale",              scale_,          300.0);       // │[counts / full deflection]
    nh_.param("control_loop_freq",  control_freq_,    500.0);      // │[Hz]

    nh_.param("tension_threshold",  tension_threshold_,  0);       // 張力初期化用
    nh_.param("tension_margin",     tension_margin_,    30.0);     // 弛緩側目標
    nh_.param("deadzone",           deadzone_cnt_,      10);       // position 差の不感帯 [counts]

    nh_.param("Kp", Kp_, 10.0);
    nh_.param("Ki", Ki_,  0.0);
    nh_.param("Kd", Kd_,  5.0);

    nh_.param("index_slave_1", index_slave_1_, 0);
    nh_.param("index_slave_2", index_slave_2_, 1);
    nh_.param("motor_inverse", motor_inverse_, std::vector<int>{+1, +1});
    nh_.param("avg_count", avg_count_, 10);

    if (motor_inverse_.size() < 2)
      ROS_FATAL("motor_inverse must have at least 2 elements (slave1, slave2)");

    dt_ = 1.0 / control_freq_;

    // ---------- ROS I/O ----------
    sub_pos_     = nh_.subscribe("/sensor/motor/output/position", 1, &JoySlaveControl::posCB, this);
    sub_tension_ = nh_.subscribe("/force/input/tension", 1, &JoySlaveControl::tensionCB, this);
    sub_joy_     = nh_.subscribe("/joy", 10, &JoySlaveControl::joyCB, this);
    pub_cmd_     = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);

    timer_ = nh_.createTimer(ros::Duration(dt_), &JoySlaveControl::timerCB, this);
  }

private:
  // ----- callbacks -----
  void posCB(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    latest_pos_ = *msg;
    if (!pos_init_done_) {
      init_pos_ = *msg;
      pos_init_done_ = true;
      pos_slave_1_ = init_pos_.data[index_slave_1_];
      pos_slave_2_ = init_pos_.data[index_slave_2_];
      pub_cmd_.publish(init_pos_);             // モータに初期値を流しておく
    }
    pos_updated_ = true;
  }

  void tensionCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    latest_ten_ = *msg;
    if (!ten_init_done_) {
      ten_buf_.push_back(*msg);
      if (ten_buf_.size() >= avg_count_) {
        init_ten_.data.resize(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); ++i) {
          double sum = 0;
          for (const auto& m : ten_buf_) sum += m.data[i];
          init_ten_.data[i] = sum / avg_count_ + tension_threshold_;
        }
        ten_init_done_ = true;
        ROS_INFO("Tension initialized.");
      }
    }
    ten_updated_ = true;
  }

  void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if (msg->axes.size() > static_cast<size_t>(axis_index_))
      axis_val_ = msg->axes[axis_index_];
  }

  // ----- main loop -----
  void timerCB(const ros::TimerEvent&)
  {
    if (!(pos_init_done_ && ten_init_done_ && pos_updated_ && ten_updated_)){
        // ROS_INFO("Waiting for initial position and tension data...");
        return;
    }

    const size_t n = latest_ten_.data.size();
    if (index_slave_1_ >= n || index_slave_2_ >= n) {
    ROS_ERROR_STREAM("tension array size=" << n
                    << ", index_slave_1=" << index_slave_1_
                    << ", index_slave_2=" << index_slave_2_
                    << "  → 配列外アクセスを防いでスキップします");
    return;               // これ以上処理しない
    }

    std_msgs::Int32MultiArray cmd = latest_pos_;   // 基本はそのまま

    // --- decide pulling side ----
    int pull_side = 0;   // 0: none, 1: slave1, 2: slave2
    if (axis_val_ >  axis_deadzone_) pull_side = 1;
    if (axis_val_ < -axis_deadzone_) pull_side = 2;
    ROS_INFO("Pulling side: %d", pull_side);

    // position increment from joystick
    double step = axis_val_ * scale_ * dt_;        // [counts]
    // ROS_INFO("Joystick axis value: %.2f, step: %.2f", axis_val_, step);

    // current tensions
    double ten1 = static_cast<double>(latest_ten_.data[index_slave_1_]);
    double ten2 = static_cast<double>(latest_ten_.data[index_slave_2_]);
    // ROS_INFO("Current tensions: slave1=%.2f, slave2=%.2f", ten1, ten2);

    // ---------- control law ----------
    if (pull_side == 1) {
      // (A) slave1 を巻く（位置増分），slave2 は margin 張力維持
      pos_slave_1_ += static_cast<int>(motor_inverse_[index_slave_1_] * step);
      cmd.data[index_slave_1_] = pos_slave_1_;
      slackPID(ten2, prev_ten2_, integral_err_[1], index_slave_2_, cmd);
    }
    else if (pull_side == 2) {
      // (B) slave2 を巻く
      pos_slave_2_ += static_cast<int>(motor_inverse_[index_slave_2_] * step);
      cmd.data[index_slave_2_] = pos_slave_2_;
      slackPID(ten1, prev_ten1_, integral_err_[0], index_slave_1_, cmd);
    }
    else {
      // (C) 両方 slack モード：両側とも tension_margin
      slackPID(ten1, prev_ten1_, integral_err_[0], index_slave_1_, cmd);
      slackPID(ten2, prev_ten2_, integral_err_[1], index_slave_2_, cmd);
    }

    // save previous tensions
    prev_ten1_ = ten1;
    prev_ten2_ = ten2;

    pos_updated_ = false;
    ten_updated_ = false;

    pub_cmd_.publish(cmd);
  }

  // ------ helper ------
  void slackPID(double tension, double prev_tension,
                double& integ, int idx,
                std_msgs::Int32MultiArray& cmd)
  {
    double err  = tension_margin_ - tension;
    double der  = (tension - prev_tension) / dt_;
    integ      += err * dt_;

    double delta = Kp_ * err + Ki_ * integ - Kd_ * der;
    cmd.data[idx] += static_cast<int>(
        motor_inverse_[idx] * delta);
  }

  // ----- members -----
  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_tension_, sub_joy_;
  ros::Publisher  pub_cmd_;
  ros::Timer      timer_;

  // parameters
  int    axis_index_;
  double axis_deadzone_;
  double scale_;
  double control_freq_;
  int    index_slave_1_, index_slave_2_;
  std::vector<int> motor_inverse_;
  double tension_margin_;
  int    tension_threshold_;
  int    deadzone_cnt_;
  int    avg_count_;
  double Kp_, Ki_, Kd_;
  double dt_;
  int pos_slave_1_, pos_slave_2_;

  // state
  bool pos_init_done_ = false, ten_init_done_ = false;
  bool pos_updated_ = false,  ten_updated_ = false;

  std_msgs::Int32MultiArray init_pos_, latest_pos_;
  std_msgs::Float64MultiArray init_ten_, latest_ten_;
  std::deque<std_msgs::Float64MultiArray> ten_buf_;

  // PID bookkeeping
  double prev_ten1_ = 0, prev_ten2_ = 0;
  double integral_err_[2] = {0.0, 0.0};

  // joystick
  double axis_val_ = 0.0;
};

// ---------- main ----------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_slave_control_node");
  ros::NodeHandle nh("~");
  JoySlaveControl node(nh);
  ros::spin();
  return 0;
}

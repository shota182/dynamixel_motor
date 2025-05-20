// bilateral_test_node.cpp (4-motor version)
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <deque>

class BilateralTestNode {
public:
  BilateralTestNode(ros::NodeHandle& nh)
    : nh_(nh) {
    nh_.param("tension_threshold", tension_threshold_, 10);
    nh_.param("pull_value_gain", pull_value_gain_, 10);
    nh_.param("Kp", Kp_, 10.0);
    nh_.param("Kf", Kf_, 1.0);
    nh_.param("Kd", Kd_, 5.0);
    nh_.param("index_master_1", index_master_1_, 0);
    nh_.param("index_master_2", index_master_2_, 1);
    nh_.param("index_slave_1", index_slave_1_, 2);
    nh_.param("index_slave_2", index_slave_2_, 3);
    nh_.param("tension_margin", tension_margin_, 30);
    nh_.param("avg_count", avg_count_, 5);

    initialized_position_ = false;
    initialized_tension_ = false;
    initialized_position_second_ = false;
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &BilateralTestNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/sensor/mag", 1, &BilateralTestNode::tensionCallback, this);
    pub_cmd_ = nh_.advertise<std_msgs::Int32MultiArray>("/sensor/motor/input/position", 10);
    timer_ = nh_.createTimer(ros::Duration(0.02), &BilateralTestNode::controlLoop, this);
  }

private:
  void positionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_position_ = *msg;
    if (!initialized_position_) {
      initial_position_ = *msg;
      initialized_position_ = true;
    } else if (initialized_tension_ && !initialized_position_second_){
      initial_position_ = *msg;
      initialized_position_second_ = true;
    }
  }

  void tensionCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    latest_tension_ = *msg;
    if (!initialized_tension_) {
      tension_buffer_.push_back(*msg);
      if (tension_buffer_.size() >= avg_count_) {
        initial_tension_.data.resize(msg->data.size());
        for (size_t i = 0; i < msg->data.size(); ++i) {
          int sum = 0;
          for (const auto& m : tension_buffer_) sum += m.data[i];
          initial_tension_.data[i] = sum / avg_count_ + tension_threshold_;
        }
        T_max_1_ = tension_margin_ + initial_tension_.data[index_master_1_];
        T_max_2_ = tension_margin_ + initial_tension_.data[index_master_2_];
        initialized_tension_ = true;
        ROS_INFO_STREAM("Initial tension averaged over " << avg_count_ << " samples.");
      }
    }
  }

  void controlLoop(const ros::TimerEvent&) {
    if (!(initialized_position_ && initialized_tension_)) return;
    std_msgs::Int32MultiArray cmd;
    cmd.data.resize(latest_position_.data.size());
    // マスタ側モータが引いた量
    int theta_m_1 = (-1) * (latest_position_.data[0] - initial_position_.data[0]);
    int theta_m_2 = latest_position_.data[2] - initial_position_.data[2];
    // マスタ側磁気センサ値の変化量
    int Tm_1 = latest_tension_.data[index_master_1_] - initial_tension_.data[index_master_1_];
    int Tm_2 = latest_tension_.data[index_master_2_] - initial_tension_.data[index_master_2_];

    // スレーブ側モータがワイヤを引いた量
    int theta_s_1 = (-1) * (latest_position_.data[1] - initial_position_.data[1]);
    int theta_s_2 = latest_position_.data[3] - initial_position_.data[3];
    // スレーブ側磁気センサ値の変化量
    int Ts_1 = latest_tension_.data[index_slave_1_] - initial_tension_.data[index_slave_1_];
    int Ts_2 = latest_tension_.data[index_slave_2_] - initial_tension_.data[index_slave_2_];

    // ログ出力
    // ROS_INFO_STREAM_THROTTLE(1.0,
    //     "initial_position_.data:" << initial_position_.data[0] << "," << initial_position_.data[1] << "," << initial_position_.data[2] << "," << initial_position_.data[3] << "\n" <<
    //     "latest_position_.data:" << latest_position_.data[0] << "," << latest_position_.data[1] << "," << latest_position_.data[2] << "," << latest_position_.data[3] << "\n" <<
    //     "initial_tension_.data:" << initial_tension_.data[index_master_1_] << "," << initial_tension_.data[index_slave_1_] << "," << initial_tension_.data[index_master_2_] << "," << initial_tension_.data[index_slave_2_] << "\n" <<
    //     "latest_tension_.data:" << latest_tension_.data[index_master_1_] << "," << latest_tension_.data[index_slave_1_] << "," << latest_tension_.data[index_master_2_] << "," << latest_tension_.data[index_slave_2_] << "\n");

    // スレーブ1
    if (latest_tension_.data[index_slave_1_] < initial_tension_.data[index_slave_1_]){
      // tension is low
      cmd.data[1] = latest_position_.data[1] + pull_value_gain_*(latest_tension_.data[index_slave_1_] - initial_tension_.data[index_slave_1_]);
      ROS_INFO_STREAM_THROTTLE(1.0, "slave 1 is low (cmd[1] = " << cmd.data[1] << ")\n");
    } else{
      // tension is good
      cmd.data[1] = static_cast<int>(-1 * Kp_ * (theta_m_1 - theta_s_1) + latest_position_.data[1]);
      // ROS_INFO_STREAM_THROTTLE(1.0, "slave 1 is good (cmd[1] = " << cmd.data[1] << ")\n");
      // ROS_INFO_STREAM_THROTTLE(1.0,
      //     "theta_m_1((-1)*(latest_position_.data[0]-initial_position_.data[0])) = " <<
      //     theta_m_1 << "((-1)*(" << latest_position_.data[0] << "-" << initial_position_.data[0] << "))\n");
      // ROS_INFO_STREAM_THROTTLE(1.0,
      //     "theta_s_1((-1)*(latest_position_.data[1]-initial_position_.data[1])) = " <<
      //     theta_s_1 << "((-1)*(" << latest_position_.data[1] << "-" << initial_position_.data[1] << "))\n");
      // ROS_INFO_STREAM_THROTTLE(1.0,
      //     "cmd.data[1] = -1 * Kp * (theta_m_1 - theta_s_1)" <<
      //     cmd.data[1] << "= -1 * " << Kp_ << " * (" << theta_m_1 << " - " << theta_s_1 << ")\n");
    }
    // スレーブ2
    if (latest_tension_.data[index_slave_2_] < initial_tension_.data[index_slave_2_]){
      // tension is low
      cmd.data[3] = latest_position_.data[3] - pull_value_gain_*(latest_tension_.data[index_slave_2_] - initial_tension_.data[index_slave_2_]);
      ROS_INFO_STREAM_THROTTLE(1.0, "slave 2 is low (cmd[3] = " << cmd.data[3] << ")\n");
    } else{
      // tension is good
      cmd.data[3] = static_cast<int>(Kp_ * (theta_m_2 - theta_s_2) + latest_position_.data[3]);
      ROS_INFO_STREAM_THROTTLE(1.0, "slave 2 is good (cmd[3] = " << cmd.data[3] << ")\n");
    }
    // マスタ1
    if (latest_tension_.data[index_master_1_] < initial_tension_.data[index_master_1_]){
        // tension is low
        cmd.data[0] = latest_position_.data[0] + pull_value_gain_*(latest_tension_.data[index_master_1_] - initial_tension_.data[index_master_1_]);
        ROS_INFO_STREAM_THROTTLE(1.0, "master1 low (cmd[0] = " << cmd.data[0] << ")\n");
    } else if (latest_tension_.data[index_master_1_] > T_max_1_){
        // tension is high
        cmd.data[0] = latest_position_.data[0] + pull_value_gain_*(latest_tension_.data[index_master_1_] - T_max_1_);  // 張りすぎ → 緩める
        ROS_INFO_STREAM_THROTTLE(1.0, "master1 high (cmd[0] = " << cmd.data[0] << ")\n");
    } else{
        // tension is good
        cmd.data[0] = static_cast<int>((-1)*(Kf_*Ts_2 + Kd_*(theta_s_1 - theta_m_1)) + latest_position_.data[0]);
        ROS_INFO_STREAM_THROTTLE(1.0, "master1 good: (cmd[0] = " << cmd.data[0] << ")\n");
    }
    // マスタ2
      if (latest_tension_.data[index_master_2_] < initial_tension_.data[index_master_2_]){
        // tension is low
        cmd.data[2] = latest_position_.data[2] - pull_value_gain_*(latest_tension_.data[index_master_2_] - initial_tension_.data[index_master_2_]);
        ROS_INFO_STREAM_THROTTLE(1.0, "master2 low (cmd[2] = " << cmd.data[2] << ")\n");
    } else if (latest_tension_.data[index_master_2_] > T_max_2_){
        // tension is high
        cmd.data[2] = latest_position_.data[2] - pull_value_gain_*(latest_tension_.data[index_master_2_] - T_max_2_);  // 張りすぎ → 緩める
        ROS_INFO_STREAM_THROTTLE(1.0, "master2 high (cmd[2] = " << cmd.data[2] << ")\n");
    } else{
        // tension is good
        cmd.data[2] = static_cast<int>(Kf_*Ts_1 + Kd_*(theta_s_2 - theta_m_2) + latest_position_.data[2]);
        ROS_INFO_STREAM_THROTTLE(1.0, "master2 good: (cmd[2] = " << cmd.data[2] << ")\n");
    }

    // ログ出力
    // ROS_INFO_STREAM_THROTTLE(1.0,
    //     "\n--- --- --- master --- --- ---\n" <<
    //     "-Kf_*Ts_2:" << (-1)*Kf_*Ts_2 << "\n" <<
    //     "Kd_*(theta_s_1 - theta_m_1):" << Kd_*(theta_s_1 - theta_m_1) << "\n" <<
    //     "master1 diff:" << (-1)*Kf_*Ts_2 << "+" << Kd_*(theta_s_1 - theta_m_1) << "=" << (-1)*Kf_*Ts_2 + Kd_*(theta_s_1 - theta_m_1) << "\n" <<
    //     "Kf_*Ts_1:" << Kf_*Ts_1 << "\n" <<
    //     "Kd_*(theta_s_2 - theta_m_2):" << Kd_*(theta_s_2 - theta_m_2) << "\n" <<
    //     "master2 diff:" << Kf_*Ts_1 << "+" << Kd_*(theta_s_2 - theta_m_2) << "=" << Kf_*Ts_1 + Kd_*(theta_s_2 - theta_m_2) << "\n" <<
    //     "--- --- --- slave --- --- ---\n" <<
    //     "Ts_1:" << Kp_ << "*" << theta_m_1 << "-" << theta_s_1 << "=" << Kp_ * (theta_m_1 - theta_s_1) << "\n" <<
    //     "slave1 diff:" << -Kp_ * (theta_m_1 - theta_s_1) << "\n" <<
    //     "Ts_2:" << Kp_ << "*" << theta_m_2 << "-" << theta_s_2 << "=" << Kp_ * (theta_m_2 - theta_s_2) << "\n" <<
    //     "slave2 diff:" << Kp_ * (theta_m_2 - theta_s_2) << "\n");

    pub_cmd_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_;
  std_msgs::Int32MultiArray initial_tension_;
  std_msgs::Int32MultiArray latest_position_;
  std_msgs::Int32MultiArray latest_tension_;

  std::deque<std_msgs::Int32MultiArray> tension_buffer_;

  bool initialized_position_, initialized_tension_, initialized_position_second_;
  int tension_threshold_;
  int pull_value_gain_;
  double Kp_, Kf_, Kd_;
  int index_master_1_, index_master_2_, index_slave_1_, index_slave_2_;
  int T_max_1_, T_max_2_, tension_margin_;
  int avg_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bilateral_force_node");
  ros::NodeHandle nh("~");
  BilateralTestNode node(nh);
  ros::spin();
  return 0;
}

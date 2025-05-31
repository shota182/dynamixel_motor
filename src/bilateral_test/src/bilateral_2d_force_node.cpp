// bilateral_test_node.cpp (4-motor version)
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
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
    nh_.param<std::vector<int>>("motor_inverse", motor_inverse_, std::vector<int>{0, 0, 0, 0});
    nh_.param("tension_margin", tension_margin_, 0.2);
    nh_.param("avg_count", avg_count_, 5);

    initialized_position_ = false;
    initialized_tension_ = false;
    initialized_position_second_ = false;
    sub_pos_ = nh_.subscribe("/sensor/motor/output/position", 1, &BilateralTestNode::positionCallback, this);
    sub_mag_ = nh_.subscribe("/force/input/tension", 1, &BilateralTestNode::tensionCallback, this);
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

  void tensionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
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
    // マスタ側の位置
    int position_master_1 = latest_position_.data[0] - initial_position_.data[0];
    int position_master_2 = latest_position_.data[2] - initial_position_.data[2];
    // マスタ側の張力
    int tension_master_1 = latest_tension_.data[index_master_1_];
    int tension_master_2 = latest_tension_.data[index_master_2_];
    // スレーブ側の位置
    int position_slave_1 = latest_position_.data[1] - initial_position_.data[1];
    int position_slave_2 = latest_position_.data[3] - initial_position_.data[3];
    // スレーブ側の張力
    int tension_slave_1 = latest_tension_.data[index_slave_1_];
    int tension_slave_2 = latest_tension_.data[index_slave_2_];
    // モータ回転向きに応じて正負を反転
    if (motor_inverse_[0]) position_master_1 = -position_master_1;
    if (motor_inverse_[1]) position_slave_1 = -position_slave_1;
    if (motor_inverse_[2]) position_master_2 = -position_master_2;
    if (motor_inverse_[3]) position_slave_2 = -position_slave_2;

    // ログ出力
    // ROS_INFO_STREAM_THROTTLE(1.0,
    //     "initial_position_.data:" << initial_position_.data[0] << "," << initial_position_.data[1] << "," << initial_position_.data[2] << "," << initial_position_.data[3] << "\n" <<
    //     "latest_position_.data:" << latest_position_.data[0] << "," << latest_position_.data[1] << "," << latest_position_.data[2] << "," << latest_position_.data[3] << "\n" <<
    //     "initial_tension_.data:" << initial_tension_.data[index_master_1_] << "," << initial_tension_.data[index_slave_1_] << "," << initial_tension_.data[index_master_2_] << "," << initial_tension_.data[index_slave_2_] << "\n" <<
    //     "latest_tension_.data:" << latest_tension_.data[index_master_1_] << "," << latest_tension_.data[index_slave_1_] << "," << latest_tension_.data[index_master_2_] << "," << latest_tension_.data[index_slave_2_] << "\n");

    // スレーブ1：マスタ1の位置に合わせる
    cmd.data[1] = static_cast<int>(Kp_ * (position_master_1 - position_slave_1) + initial_position_.data[1]);
    // スレーブ2：マスタ2の位置に合わせる
    cmd.data[3] = static_cast<int>(Kp_ * (position_master_2 - position_slave_2) + initial_position_.data[3]);
    // マスタ1：スレーブ2の張力に合わせる
    cmd.data[0] = static_cast<int>(Kf_ * (tension_slave_2 - tension_master_1) + latest_position_.data[0]);
    // マスタ2：スレーブ1の張力に合わせる
    cmd.data[2] = static_cast<int>(Kf_ * (tension_slave_1 - tension_master_2) + latest_position_.data[2]);

    // ログ出力
    // ROS_INFO_STREAM_THROTTLE(1.0,
    //     "\n--- --- --- master --- --- ---\n" <<
    //     "-Kf_*tension_slave_2:" << (-1)*Kf_*tension_slave_2 << "\n" <<
    //     "Kd_*(position_slave_1 - position_master_1):" << Kd_*(position_slave_1 - position_master_1) << "\n" <<
    //     "master1 diff:" << (-1)*Kf_*tension_slave_2 << "+" << Kd_*(position_slave_1 - position_master_1) << "=" << (-1)*Kf_*tension_slave_2 + Kd_*(position_slave_1 - position_master_1) << "\n" <<
    //     "Kf_*tension_slave_1:" << Kf_*tension_slave_1 << "\n" <<
    //     "Kd_*(position_slave_2 - position_master_2):" << Kd_*(position_slave_2 - position_master_2) << "\n" <<
    //     "master2 diff:" << Kf_*tension_slave_1 << "+" << Kd_*(position_slave_2 - position_master_2) << "=" << Kf_*tension_slave_1 + Kd_*(position_slave_2 - position_master_2) << "\n" <<
    //     "--- --- --- slave --- --- ---\n" <<
    //     "tension_slave_1:" << Kp_ << "*" << position_master_1 << "-" << position_slave_1 << "=" << Kp_ * (position_master_1 - position_slave_1) << "\n" <<
    //     "slave1 diff:" << -Kp_ * (position_master_1 - position_slave_1) << "\n" <<
    //     "tension_slave_2:" << Kp_ << "*" << position_master_2 << "-" << position_slave_2 << "=" << Kp_ * (position_master_2 - position_slave_2) << "\n" <<
    //     "slave2 diff:" << Kp_ * (position_master_2 - position_slave_2) << "\n");

    pub_cmd_.publish(cmd);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_pos_, sub_mag_;
  ros::Publisher pub_cmd_;
  ros::Timer timer_;

  std_msgs::Int32MultiArray initial_position_;
  std_msgs::Float64MultiArray initial_tension_;
  std_msgs::Int32MultiArray latest_position_;
  std_msgs::Float64MultiArray latest_tension_;

  std::deque<std_msgs::Float64MultiArray> tension_buffer_;

  bool initialized_position_, initialized_tension_, initialized_position_second_;
  int tension_threshold_;
  int pull_value_gain_;
  double Kp_, Kf_, Kd_;
  int index_master_1_, index_master_2_, index_slave_1_, index_slave_2_;
  int T_max_1_, T_max_2_;
  double tension_margin_;
  std::vector<int> motor_inverse_;
  int avg_count_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "bilateral_force_node");
  ros::NodeHandle nh("~");
  BilateralTestNode node(nh);
  ros::spin();
  return 0;
}
